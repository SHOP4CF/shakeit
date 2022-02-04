# ROSy stuff
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

# srvs and msgs
from shakeit_interfaces.msg import Plot
from shakeit_interfaces.action import Pick
from anyfeeder_interfaces.srv import StandardInput

# CoppeliaSim
from pyrep import PyRep
from pyrep.backend import sim
from pyrep.backend.utils import suppress_std_out_and_err
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.shape import Shape

# other
import numpy as np
from math import sin, pi, floor, sqrt
from random import randint, sample, choice
from collections import deque


class CoppeliaProxyNode(Node):

    def __init__(self):
        self.name = 'sim'
        super().__init__(self.name)
        self.get_logger().info(f"Initializing {self.name}...")

        # Configurable parameters
        scene = self.declare_parameter('scene', '')
        headless = self.declare_parameter('headless', True)
        self.pick_threshold = self.declare_parameter('pick_threshold', 0.005).value
        self.new_objects_limit = self.declare_parameter('add_new_limit', 5).value
        self.extra_stepping = self.declare_parameter('extra_stepping', 150).value

        # CoppeliaSim
        with suppress_std_out_and_err():
            self.simulator = PyRep()
            self.simulator.launch(scene_file=scene.value, headless=headless.value)
            self.simulator.start()
            self.step_n_times(n=1)

        # spawn
        OBJECT_BOUNDING_BOX = 0.035
        DISPENSER_Y_LIM = 0.06
        ROWS = 5
        ys = [i for i in np.arange(-DISPENSER_Y_LIM, DISPENSER_Y_LIM, OBJECT_BOUNDING_BOX/4)]
        xs = np.arange(0.0, (OBJECT_BOUNDING_BOX/4) * ROWS, OBJECT_BOUNDING_BOX/4)
        self.spawn_grid = [(x, y) for x in xs for y in ys]  # generate a grid based on object size and amount of rows
        self.spacing = 0.0275  # distance between spawn objects [m]
        self.spawn_trials = 30
        self.spawn_obj = Dummy('object_spawn')

        # objects
        self.motors = {'x': {'obj': Joint('vibrator_x'), 'pos': 0.0},
                       'y': {'obj': Joint('vibrator_y'), 'pos': 0.0},
                       'z': {'obj': Joint('vibrator_z'), 'pos': 0.0},
                       'disp_x': {'obj': Joint('dispenser_motor_x'), 'pos': 0.0},
                       'disp_z': {'obj': Joint('dispenser_motor_z'), 'pos': 0.0}}
        self.reference_object = Shape('reference_shape')
        self.objects = [obj.get_handle() for obj in self.spawn_obj.get_objects_in_tree()]
        self.t_pick_objects = deque(maxlen=30)
        self.n_discard_objects = 0

        self.get_logger().info('objects: {}'.format(self.objects))

        # ROS
        self.pick_action = ActionServer(
            self, Pick, f'{self.get_name()}/pick',
            execute_callback=self.pick_execute_callback,
            goal_callback=self.pick_goal_callback,
            cancel_callback=self.pick_cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.init_srv = self.create_service(StandardInput, self.name + '/init', self.init_cb)
        self.add_srv = self.create_service(StandardInput, self.name + '/add', self.add_objects_cb)
        self.purge_srv = self.create_service(StandardInput, self.name + '/purge', self.purge_shaker_cb)
        self.forward_srv = self.create_service(StandardInput, self.name + '/forward', self.forward_cb)
        self.backward_srv = self.create_service(StandardInput, self.name + '/backward', self.backward_cb)
        self.flip_srv = self.create_service(StandardInput, self.name + '/flip', self.flip_cb)
        self.validation_pub = self.create_publisher(Plot, 'visualizer', 10)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def move(self, shakes_todo: int, motors: list, phases: list, scales: list, periods: list, speeds: list):
        """
        Velocity based control of the motors.
        Each motor is controlled separately based on a phase, scale and period.
        After movement, the dispenser slowly returns to the starting position.
        """
        t0, shakes_done = sim.simGetSimulationTime(), 0
        t = t0

        while shakes_done < shakes_todo:
            if max(periods) < t - t0:  # count shakes
                shakes_done += 1
                self.step_n_times(n=50)  # cool down after each shake
                t0 = sim.simGetSimulationTime()
                t = t0

            dt = sim.simGetSimulationTime() - t
            t += dt
            for motor, phase, period, scale, speed in zip(motors, phases, periods, scales, speeds):
                v = speed * scale(t - t0, period) * self.fourier_triangle_wave(t - t0, period / 2, m=60, n=20, ph=phase)
                motor['pos'] += v * dt
                motor['obj'].set_joint_position(motor['pos'])
            self.simulator.step()

        iterations_to_return = 100
        delta_returns = [motor['pos']/iterations_to_return for motor in motors]
        for step in range(iterations_to_return):
            for motor, delta in zip(motors, delta_returns):
                motor['pos'] -= delta
                motor['obj'].set_joint_position(motor['pos'])
            self.simulator.step()

    @staticmethod
    def fourier_triangle_wave(t, tmax, m, n, ph=0):
        """
        Returns a value for time=t of a triangle wave.
        The wave is approximated with Fourier Series (n=number of approximations) for continuity (smoothness).
        The triangle wave is defined by:
            - period tmax,
            - symmetry m [m=2 for symmetry, m>2 for unsymmetrical],
            - phase shift ph
        """
        ph = (ph * pi) / 180
        bn = lambda n, m: ((2 * m ** 2 * (-1) ** n) / (n ** 2 * (m - 1) * pi ** 2)) * sin((n * pi * (m - 1)) / m)
        g = lambda n, t, tmax, bn: bn * sin(ph + (n * pi * t) / tmax)
        return sum([g(i, t, tmax, bn(i, m)) for i in range(1, n)])

    def get_dispenser_objects(self):
        return [handle for handle in self.objects if sim.simGetObjectPosition(handle, -1)[2] > 0.18]

    def spawn_new_objects(self):
        """Spawns n=self.new_objects_limit new objects randomizing orientation & preventing overlapping."""
        new_objects = self.get_dispenser_objects()
        num_new_objects = randint(2, self.new_objects_limit) + len(new_objects)
        trials = 0
        while len(new_objects) < num_new_objects:
            x_new, y_new = sample(self.spawn_grid, 1)[0]

            objects = [sim.simGetObjectPosition(handle, self.spawn_obj.get_handle())[:-1] for handle in new_objects]
            distances = [sqrt(((x_new - x_o) ** 2) + ((y_new - y_o) ** 2)) > self.spacing for x_o, y_o in objects]
            if False in distances:
                trials += 1
                if trials > self.spawn_trials:  # try self.spawn_trials times to find a place in a grid
                    break
            else:
                trials = 0

                position = [x_new, y_new, 0]
                orientation = [pi * choice([0, 90, 180]) / 180 for _ in range(3)]

                new_convex_handle = self.reference_object.copy().get_handle()
                sim.simSetObjectOrientation(new_convex_handle, self.spawn_obj.get_handle(), orientation)
                sim.simSetObjectPosition(new_convex_handle, self.spawn_obj.get_handle(), position)
                sim.simSetObjectParent(new_convex_handle, self.spawn_obj.get_handle(), True)
                new_objects.append(new_convex_handle)
                self.step_n_times(n=1)
        self.objects += [obj for obj in new_objects if obj not in self.objects]

    def step_n_times(self, n):
        """Calls sim.step() n times."""
        for _ in range(n):
            self.simulator.step()

    def remove_lost_objects(self):
        """Remove all objects that are below the main bed. And those lost below the dispenser."""
        for idx, obj in enumerate(self.objects):
            pos = sim.simGetObjectPosition(obj, -1)
            if pos[2] < .1 or pos[0] > .17:
                sim.simRemoveObject(obj)
                del(self.objects[idx])

    def clear_dispenser_bed(self):
        """Remove all objects that are on the dispenser bed."""
        for idx, obj in enumerate(self.objects):
            if sim.simGetObjectPosition(obj, -1)[2] > .175 and sim.simGetObjectPosition(obj, -1)[0] > .095:
                sim.simRemoveObject(obj)
                del(self.objects[idx])

    def publish_discarded(self):
        self.validation_pub.publish(Plot(**{'key': 'discarded objects',
                                            'values': [self.n_discard_objects],
                                            'labels': ['number of discarded objects']}))

    def average_objs_positions(self):
        """Returns average position of all objects in self.objects. Useful for evaluating behaviors."""
        x, y, z, n = 0, 0, 0, len(self.objects)
        for obj in self.objects:
            x, y, z = [p + e for e, p in zip(sim.simGetObjectPosition(obj, -1), [x, y, z])]
        return x / n, y / n, z / n

    def purge_shaker_cb(self, request: StandardInput.Request, response: StandardInput.Response):
        """Removes all objects belonging to 'object_spawn' dummy tree in sim."""
        self.get_logger().info('{} has been called.'.format(self.purge_srv.srv_name))
        self.n_discard_objects = len(self.objects)
        for obj in self.objects:
            sim.simRemoveObject(obj)
        self.objects = []
        self.step_n_times(n=1)
        self.publish_discarded()
        return response

    def pick_goal_callback(self, goal: Pick.Goal):
        self.get_logger().info(f"Received pick goal request. {goal}.")
        return GoalResponse.ACCEPT

    def pick_cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID} (pick).")
        return CancelResponse.ACCEPT

    async def pick_execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Finds the closest point from coordinates given in Request.
        Removes and returns success if the distance < self.pick_threshold.
        """
        goal: Pick.Goal = goal_handle.request
        result = Pick.Result()
        self.get_logger().info(f'Pick action has been called.')
        self.clear_dispenser_bed()
        try:
            remove_obj = None
            for idx, obj in enumerate(self.objects):  # find an object close to requested picking location
                pos_sim = np.asarray(sim.simGetObjectPosition(obj, -1)[:-1])
                pos_request = np.asarray([goal.pose.position.x, goal.pose.position.y])
                difference = np.linalg.norm(pos_sim - pos_request)
                if float(difference) < self.pick_threshold:
                    remove_obj = idx
                    break
            if remove_obj is not None:  # pick and report success
                self.get_logger().info(f'Successful pick.')
                sim.simRemoveObject(self.objects[remove_obj])
                del self.objects[remove_obj]
                result.success = True
            else:  # report fail if no objects in proximity
                msg = f'Failed pick, no objects to pick in proximity {self.pick_threshold:.4f}[m].'
                self.get_logger().info(msg)
                result.success = False
                result.message = msg
        except Exception as e:
            msg = f'Failed pick, {e}'
            self.get_logger().info(msg)
            result.success = False
            result.message = msg
        finally:
            self.step_n_times(n=1)
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

    def init_cb(self, request: StandardInput.Request, response: StandardInput.Response):
        self.get_logger().info('Init has been called.')
        return response

    def add_objects_cb(self, request: StandardInput.Request, response: StandardInput.Response):
        """Spawns up to n=self.new_objects_limit objects and moves them forward on the dispenser bed."""
        self.get_logger().info('{} has been called.'.format(self.add_srv.srv_name))

        for _ in range(request.parameters.repetitions):
            if len(self.get_dispenser_objects()) < 4:
                self.spawn_new_objects()
            self.step_n_times(n=floor(self.extra_stepping / 3))  # let the newly spawn objects fall

            behavior = dict(motors=[self.motors['disp_x'], self.motors['disp_z']],
                            phases=[174, -8],
                            #phases=[-6, -8],
                            scales=[lambda t, p: .04, lambda t, p: .097],
                            periods=[.077, .077])
            self.move(**behavior, shakes_todo=4, speeds=[request.parameters.speed, 1])

        self.remove_lost_objects()
        self.step_n_times(n=self.extra_stepping)

        return response

    def forward_cb(self, request: StandardInput.Request, response: StandardInput.Response):
        self.get_logger().info(f'[forward_cb] Has been called. Request: {request}')
        self.clear_dispenser_bed()
        try:
            behavior = dict(motors=[self.motors['x'], self.motors['z']],
                            phases=[-6, -8],
                            scales=[lambda t, p: .0948, lambda t, p: .010],
                            periods=[.077, .077])
            self.move(**behavior, shakes_todo=request.parameters.repetitions, speeds=[request.parameters.speed,
                                                                                      request.parameters.speed])

            response.status = StandardInput.Response.OK
        except Exception as e:
            self.get_logger().warning(f"[forward_cb] Error: {e}")
            response.status = StandardInput.Response.ERROR
            response.status_msg = f"Error: {e}"
        finally:
            self.step_n_times(n=floor(self.extra_stepping * .3))
            self.remove_lost_objects()
            return response

    def backward_cb(self, request: StandardInput.Request, response: StandardInput.Response):
        self.get_logger().info(f'[backward_cb] Has been called. Request: {request}')
        self.clear_dispenser_bed()
        try:
            behavior = dict(motors=[self.motors['x'], self.motors['z']],
                            phases=[174, -8],
                            scales=[lambda t, p: .0948, lambda t, p: .010],
                            periods=[.077, .077])
            self.move(**behavior, shakes_todo=request.parameters.repetitions, speeds=[request.parameters.speed,
                                                                                      request.parameters.speed])

            response.status = StandardInput.Response.OK
        except Exception as e:
            self.get_logger().warning(f"[backward_cb] Error: {e}")
            response.status = StandardInput.Response.ERROR
            response.status_msg = f"Error: {e}"
        finally:
            self.step_n_times(n=floor(self.extra_stepping * .3))
            self.remove_lost_objects()
            return response

    def flip_cb(self, request: StandardInput.Request, response: StandardInput.Response):
        self.get_logger().info(f'[flip_cb] Has been called. Request: {request}')
        self.clear_dispenser_bed()
        try:
            behavior = dict(motors=[self.motors['x'], self.motors['z']],
                            phases=[90, 180],
                            scales=[lambda t, p: .1, lambda t, p: .008 if t < p else .021],
                            periods=[.077, .077 / 2])
            self.move(**behavior, shakes_todo=request.parameters.repetitions, speeds=[1, request.parameters.speed])

            response.status = StandardInput.Response.OK
        except Exception as e:
            self.get_logger().warning(f"[flip_cb] Error: {e}")
            response.status = StandardInput.Response.ERROR
            response.status_msg = f"Error: {e}"
        finally:
            self.step_n_times(n=floor(1.25 * self.extra_stepping))
            self.remove_lost_objects()
            return response


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.simulator.stop()
        node.simulator.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

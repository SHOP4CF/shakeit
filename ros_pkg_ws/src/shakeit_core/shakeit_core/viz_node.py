import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import hsv_to_rgb
from pickle import load, dump
from time import time

import rclpy
from rclpy.node import Node

from shakeit_interfaces.msg import Plot
from std_msgs.msg import String


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        # ROS
        self.visualizer_sub = self.create_subscription(Plot, 'visualizer', self.update_cb, 10)
        self.title_sub = self.create_subscription(String, 'visualizer/title', self.title_cb, 10)

        # dynamic parameters
        self.title = self.declare_parameter('title', 'Training progress')
        self.window_size = self.declare_parameter('window_size', [20, 50, 100])
        self.save_plot = self.declare_parameter('save_plot', '')
        self.max_cols = self.declare_parameter('max_cols', 2)
        self.color_set = [hsv_to_rgb(eval(c)) for c in self.declare_parameter('color_set', []).value]
        figure_dpi = self.declare_parameter('figure_dpi', 200)

        # plt
        plt.rc('font', **{'family': 'normal', 'weight': 'light', 'size': 5})
        plt.ion()
        self.lw = 0.6  # line width
        try:
            self.fig = load(open(self.save_plot.value + '/plot_data.pickle', 'rb'))
        except Exception:
            self.fig = plt.figure(dpi=figure_dpi.value, figsize=[9.6, 7.0])
        self.fig.suptitle(self.title.value, fontsize=16)
        self.slots, self.rows, self.cols = 0, 1, 1

        # meta
        self.last_saved = 0
        self.saving_interval = 60

    def title_cb(self, msg):
        self.fig.suptitle(msg.data, fontsize=16)

    def update_cb(self, msg: Plot):
        """
        Update callback digest a new plot message.

        Create a new axis if non-existent, then update the plot, and save every now and then.
        :param msg: a key, labels and values
        """
        if msg.key not in [axis.get_label() for axis in self.fig.get_axes()]:
            self.get_logger().info(f'Creating {msg.key} plot.')
            self.slots += 1
            self.new_axis(msg.key, msg.labels)
        else:
            self.get_logger().info(f'Updating {msg.key} plot.')

        axis = self.get_axis_by_name(msg.key)
        lines = axis.get_lines()
        if len(msg.labels) == 1:  # for single lines calculate the running average
            main_line = lines[0]
            average_lines = lines[1:]
            x, y = main_line.get_data()
            new_x = np.concatenate((x, [len(x)]))
            new_y = np.concatenate((y, [msg.values[0]]))
            main_line.set_data((new_x, new_y))
            for size, line in zip(self.window_size.value, average_lines):
                if len(new_x) > size:
                    line.set_data((new_x, self.moving_average(new_y, n=size)))
        else:
            for line, value, label in zip(lines, msg.values, msg.labels):
                x, y = line.get_data()
                new_x, new_y = np.concatenate((x, [len(x)])), np.concatenate((y, [value]))
                line.set_data((new_x, new_y))

        axis.relim()
        axis.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        if time() - self.last_saved > self.saving_interval:
            self.save()

    def new_axis(self, new_axis_name, labels):
        """
        Make a slot for a new plot in the window.

        :param new_axis_name: of the new plot
        :param labels: labels for the existing plots
        """
        self.grid_update()
        self.redraw()

        new_axis = self.fig.add_subplot(self.rows, self.cols, self.slots)
        new_axis.set_label(new_axis_name)
        for idx, label in enumerate(labels):
            ls = ('-' * (idx % 3 == 0)) + ('--' * (idx % 3 == 1)) + (':' * (idx % 3 == 2))
            color = self.color_set[idx % len(self.color_set)] if len(self.color_set) else []
            new_axis.plot([], [], label=label, linewidth=self.lw, linestyle=ls, color=color)
        if len(labels) == 1:
            for idx, size in enumerate(self.window_size.value):
                new_axis.plot([], [], label=f'{new_axis_name} mean({size})', linestyle=':', linewidth=self.lw)

        for axis in self.fig.get_axes():
            _, labels = axis.get_legend_handles_labels()
            col = len(labels) if len(labels) <= 10 else 10  # more than 10 short labels don't display well
            axis.legend(loc='upper center', bbox_to_anchor=(0., 1.02, 1., .102), ncol=col,
                        framealpha=1, fontsize='xx-small', mode='expand')
            axis.axes.xaxis.set_ticks([])
            axis.set_ylabel(axis.get_label())
            axis.grid(True, alpha=0.2)

    def redraw(self):
        """
        Redraw all axes and plots based on current grid.

        For each axis: first copy lines, then delete a plot, finally remake a plot at the new location.
        """
        for position, axis in enumerate(self.fig.get_axes()):
            # copy data
            axis_name = axis.get_label()
            data, labels, colors, styles = [], [], [], []
            for line in axis.get_lines():
                data.append(line.get_data())
                labels.append(line.get_label())
                colors.append(line.get_c())
                styles.append(line.get_linestyle())
            # delete old axis
            self.fig.delaxes(axis)
            # make new axis
            plot = self.fig.add_subplot(self.rows, self.cols, position + 1)
            plot.set_label(axis_name)
            # redraw plots
            for idx, ((x, y), label, color, style) in enumerate(zip(data, labels, colors, styles)):
                plot.plot(x, y, label=label, linewidth=self.lw, linestyle=style, color=color)

    def get_axis_by_name(self, key):
        for axis in self.fig.get_axes():
            if axis.get_label() == key:
                return axis
        raise ValueError(f'Plot "{key}" does not exist.')

    def grid_update(self):
        """Recalculate rows and cols based on N of slots."""
        self.rows = int(self.slots / self.max_cols.value) + self.slots % self.max_cols.value
        self.cols = 1 if self.slots < self.max_cols.value else self.max_cols.value

    def moving_average(self, a, n):
        """Calculate the moving average of n = self.window_size values; also adds padding."""
        n = n if n % 2 != 0 else n + 1  # if n is even, make it odd
        padding = int((n - 1) / 2)
        ret = np.convolve(a, [1] * n, mode='same')
        ret[padding: -padding] = ret[padding: -padding]/n
        what = np.asarray([i for i in range(padding + 1, n)])
        ret[:padding] = ret[:padding]/what
        ret[-padding:] = ret[-padding:]/(what[::-1])
        return ret

    def save(self):
        """Save the plot and the data."""
        if not os.path.isdir(self.save_plot.value):
            os.makedirs(self.save_plot.value)
        self.fig.savefig(self.save_plot.value + '/plot.png')
        dump(self.fig, open(self.save_plot.value + '/plot_data.pickle', 'wb'))
        self.last_saved = time()


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

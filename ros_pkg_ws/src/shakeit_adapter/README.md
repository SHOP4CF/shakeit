# Shakeit adapters
This package contains adapter to bridge between the Shakeit-pipeline and physical hardware.

## Adapters
* fake_robot_node
* kuka_adapter_node
* sensopart_adapter_node


### Manual tests

#### SensoPart/Kuka test
Manually testing that:
* the SensoPart camera can be triggered (sensopart_node),
* data can be received and parsed (sensopart_adapter_node), 
* and that a pick position can be send to the robot (kuka_adapter_node)
can be done with the following command:
```
$ ros2 launch shakeit_adapter test_robot_camera.launch.py
```
Then in another terminal:
```
$ ros2 action send_goal /robot_camera_test_node/test shakeit_interfaces/action/Trigger {}
```
The robot should then pick-up a available object.
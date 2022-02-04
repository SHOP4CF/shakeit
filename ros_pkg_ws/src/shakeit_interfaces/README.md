# shakeit_interfaces
The package is used to define common messages, services, and actions for the project.

## Messages
* ActionInfo.msg: `int32 count, float32 avg, float32 median, float32 max`
* Observation.msg: `sensor_msgs/Image img, sensor_msgs/CameraInfo camera`
* Point.msg: `float32 x, float32 y`
* Update.msg: `Observation old_state, Observation new_state, float32 reward, int32 action, bool done`

## Services
* Act:  
  `{Request: {Observation: observation}}`  
  `{Response: {action: int}}`  
* FreeObjects:  
  `{Request: {}}`  
  `{Response: {empty: bool, count: int, points: geometry_msgs/Point[]}}`
* Pick:  
  `{Request: {point: geometry_msgs/Point}}`  
  `{Response: {success: bool}}`

* Shake: `{Request: {frequency: float32[], amplitude: float32[], time: float32}} -> {Response: {success: bool}}`

## Actions
* SeparateObjects.action:  
  `{Request: {free_objects: int, max_actions: int}}`  
  `{Feedback: {feedback: string, free_objects: int}}`  
  `{Result: {free_objects: int}}`  

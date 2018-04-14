# eurobot_localisation

This localisation function uses Apriltags2 to determine the location of our robot relative to the arena. 
A Jetson TX2 was used to perform CV in order to extract all tag information via Apriltags.
There are four parts to the localisation pipeline for each camera attached:

1. ROS camera driver (`gscam` for the Jetson onboard cam, `cv_camera` for any ordinary USB camera)
2. Image rectifier (corrects the raw image feed for any distortion caused by the camera)
3. Apriltags2 (Rectified image is used to calculate tag position relative to camera)
4. Custom localisation node (Uses the Apriltags information to calculate the pose of the robot, relative to the world)

Apriltags is also used to detect the enemy's position relative to the robot.

To run the localisation function, enter:
```
roslaunch eurobot_localisation multiple_camera.launch
```
This will launch all the components required for both cameras.

## ROS Package Dependencies
1. `Apriltags2_ros`
2. `gscam`
3. `cv_camera`
4. `tf2`
5. `tf2_ros`
6. `geometry_msgs`


## Published ROS topics

1. `eurobot/state/visual_measurements` (tf2_msgs/TFMessage)
2. `eurobot/enemy/relative_position' (tf2_msgs/TFMessage)




# QR code reader

# Commands

* launch roscore

```
roscore
```

* launch camera

May change depending on your camera 
```
roslaunch realsense2_camera rs_camera.launch 
```

* run camera node

Becareful depending on the camera you're using, the topic might be different.
To know the name of your topic use ``` rostopic list``` and copy past it in src/camera_subscriber.cpp.

```
rosrun qr_code_reader camera_subscriber
```
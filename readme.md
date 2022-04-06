# QR code reader

# Install 

* Go to ```cd catkin_ws/src```
* git clone ```git clone https://github.com/Thermay-Robotics/qr_code_reader.git```
* Then compile 
``` cd catkin_ws``` 
``` catkin_make ```

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

* run qrcode reading 

First you need to install zbar library ```sudo apt-get install -y zbar-tools```

Then run :

```
rosrun qr_code_reader qr_reader
```

you should see the data associated to qr code(s) on the terminal.
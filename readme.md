# QR code reader

# Install 

* Go to ```cd catkin_ws/src```
* git clone ```git clone https://github.com/Thermay-Robotics/qr_code_reader.git```
* Then compile 
``` cd catkin_ws``` 
``` catkin_make ```

# Run

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

First you need to install zbar library 

```
sudo apt-get install ros-(distro)-usb-cam
sudo apt-get install -y zbar-tools
```

Then run :

```
roslaunch usb_cam usb_cam-test.launch

rosrun qr_code_reader qr_reader
```

# Launch 

To launch the package, use the following command :

```roslaunch qr_code_reader qr_reader.launch```


To launch with the camera, you can use the following command :

```roslaunch qr_code_reader qr_reader_camera.launch```



you should see the data associated to qr code(s) on the terminal.
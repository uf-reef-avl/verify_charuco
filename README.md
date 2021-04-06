**Verify charuco**
=================

This package simulates the detection of charuco marker by a camera mounted on a quadrotor. 
To do so, it uses as input:

- the current position of the quad rotor in the optitrack space (**tf_cam** topic)
- the current postion of the charuco board in the optitrack space (**tf_calib** topic)
- and the static tranformation from the quad rotor position to the camera position (extresic parameter of the camera in the **params/body_to_camera.yaml**)

Thanks to these input, this package can compute the transformation from the camera to the board in the camera frame.
To simulate the fact that the camera cannot see the board from everywhere, we added some threshold parameters as 3 entries:
    - the azimut_threshold (max rad) which defines when the camera begin to detect the board depending if the quad is pointing at the board.
    - the elevation_threshold (max rad) which defines when the camera begin to detect the board depending on the x, y position of the quad in the space.
    - the z_threshold (m) which defines at what altitude the camera begin to detect the board.

To setup and compile this package use the following commands:

    mkdir -p ~/charuco_ws/src
    cd ~/charuco_ws/src
    git clone <this package url>
    git clone https://github.com/uf-reef-avl/reef_msgs
    cd ~/charuco_ws
    catkin_make #or catkin build 
    source devel/setup.bash

Notes: You will probably have to install the dependencies of this package to compile it. They are listed below:

  - geometry_msgs
  - roscpp
  - rospy
  - std_msgs
  - tf2
  - tf2_eigen
  - tf

To launch this package with a bag, change the **bag_path** value to the path of your bagfile in the **basic.launch** file and use the following command :

    cd ~/charuco_ws
    source devel/setup.bash
    roslaunch verify_aruco basic.launch




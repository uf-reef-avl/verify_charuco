//
// Created by prashant on 5/3/19.
//
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


void aruco_Callback(const geometry_msgs::PoseStampedConstPtr& aruco_pose)
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener listener(tfBuffer);
  geometry_msgs::TransformStamped tf_board;
  geometry_msgs::TransformStamped tf_cam;
  geometry_msgs::TransformStamped cam_to_board;
  ros::Time query_time = aruco_pose->header.stamp;
  try{
    tf_board = tfBuffer.lookupTransform("optitrack","tf_board",query_time, ros::Duration(1));
//    ROS_WARN_STREAM("TF BOARD in optitrack frame in "<< tf_board.transform);
    tf_cam = tfBuffer.lookupTransform("optitrack","tf_cam",query_time, ros::Duration(1));
//    ROS_WARN_STREAM("TF CAM in optitrack frame in "<< tf_cam.transform);

//    cam_to_board = tfBuffer.lookupTransform("aruco_marker_frame","tf_board",query_time, ros::Duration(1));
//    ROS_WARN_STREAM("TF CAM in marker frame in "<< cam_to_board.transform);

    cam_to_board = tfBuffer.lookupTransform("camera_rgb_optic_frame","tf_board",query_time, ros::Duration(1));
    ROS_WARN_STREAM("TF Board in Camera frame in "<< cam_to_board.transform);


  }
  catch(tf::TransformException ex) {}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "verify_aruco_tf");
  ros::NodeHandle node;
  std::string aruco_topic;

  node.param<std::string>("aruoc_topic", aruco_topic, "aruco_single/pose");

  ros::Subscriber sub = node.subscribe("aruco_single/pose", 1 , aruco_Callback);

  ros::spin();
  return 0;


}
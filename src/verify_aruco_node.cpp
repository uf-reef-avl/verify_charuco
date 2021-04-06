//
// Created by prashant on 5/3/19.
//

#include "../include/verify_aruco/verify_aruco_node.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>

VerifyAruco::VerifyAruco():
    nh_(""),
    private_nh("~"),
    initialized(false),
    tf_cam_subscriber_(nh_, "tf_cam",1),
    tf_board_subscriber_(nh_, "tf_calib",1),
    synchronizer_(SyncPolicy(10), tf_cam_subscriber_, tf_board_subscriber_){

  synchronizer_.registerCallback(boost::bind(&VerifyAruco::callback, this, _1, _2));

  reef_msgs::loadTransform("body_to_camera", body_to_camera);
    private_nh.param<double>("publishing_frequency", publish_frequency, 20);
    private_nh.param<double>("azimuth_threshold", azimuth_threshold, 25 * M_PI/180);
    private_nh.param<double>("elevation_threshold", elevation_threshold, 25*M_PI/180);
    private_nh.param<double>("z_threshold", z_threshold, 1.2);
  fake_charuco_publisher = nh_.advertise<geometry_msgs::PoseStamped>("fake_charuco", 1000);

  ROS_WARN_STREAM("Body To camera Rotation Matrix ");
  ROS_WARN_STREAM(body_to_camera.linear()<< "\n");
  ROS_WARN_STREAM("Body To camera Translation");
  ROS_WARN_STREAM(body_to_camera.translation()<< "\n");
  ROS_WARN_STREAM("azimuth_threshold");
  ROS_WARN_STREAM(azimuth_threshold<< "\n");
  ROS_WARN_STREAM("elevation_threshold");
  ROS_WARN_STREAM(elevation_threshold << "\n");
  ROS_WARN_STREAM("z_threshold");
  ROS_WARN_STREAM( z_threshold<< "\n");
}

void VerifyAruco::callback(const geometry_msgs::PoseStampedConstPtr &tf_cam,
                           const geometry_msgs::PoseStampedConstPtr &tf_board
                       ) {

    if(!initialized){
        last_time = tf_board->header.stamp;
        initialized = true;
        return;
    }
  tf2::fromMsg(tf_cam->pose, optitrack_to_camera);
  tf2::fromMsg(tf_board->pose, optitrack_to_board);

  camera_to_board_optitrack_frame = optitrack_to_camera.inverse() * optitrack_to_board;
  camera_to_board_camera_frame = body_to_camera.inverse() * camera_to_board_optitrack_frame;
    double azimuth_mocap = std::atan2(camera_to_board_camera_frame.translation().x(),camera_to_board_camera_frame.translation().y());
    double elevation_mocap = std::atan2(camera_to_board_camera_frame.translation().y(),camera_to_board_camera_frame.translation().z());

    if (abs(azimuth_mocap) <  azimuth_threshold && abs(elevation_mocap) < elevation_threshold && camera_to_board_camera_frame.translation().z()< z_threshold) {

        geometry_msgs::PoseStamped poseStamped_msg;
        poseStamped_msg.pose = tf2::toMsg(camera_to_board_camera_frame);

        if ((tf_board->header.stamp - last_time).toSec() > 1 / publish_frequency) {
            fake_charuco_publisher.publish(poseStamped_msg);
            last_time = tf_board->header.stamp;
        }
    }
}






int main(int argc, char **argv) {

  ros::init(argc, argv, "verify_aruco");
  VerifyAruco obj;

  ros::spin();

  return 0;

}

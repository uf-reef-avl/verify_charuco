//
// Created by prashant on 5/3/19.
//

#ifndef PROJECT_VERIFY_ARUCO_NODE_H
#define PROJECT_VERIFY_ARUCO_NODE_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <reef_msgs/matrix_operation.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class VerifyAruco{

 public:
  VerifyAruco();
  ~VerifyAruco(){}
 private:
//  tf::TransformListener listener;
  Eigen::Affine3d body_to_camera;
  Eigen::Affine3d optitrack_to_board;
  Eigen::Affine3d optitrack_to_camera;
  Eigen::Affine3d camera_to_board_optitrack_frame;
  Eigen::Affine3d camera_to_board_camera_frame;
  Eigen::Affine3d aruco_result_camera_frame;


  void callback(const geometry_msgs::PoseStampedConstPtr& tf_cam,
                const geometry_msgs::PoseStampedConstPtr& tf_board);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh;
  ros::Publisher fake_charuco_publisher; 

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                          geometry_msgs::PoseStamped> SyncPolicy;

  message_filters::Subscriber<geometry_msgs::PoseStamped> tf_cam_subscriber_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> tf_board_subscriber_;

  message_filters::Synchronizer<SyncPolicy> synchronizer_;

  ros::Time last_time;
  double publish_frequency;
  double azimuth_threshold;
  double elevation_threshold;
  double z_threshold;
  bool initialized;

};

#endif //PROJECT_VERIFY_ARUCO_NODE_H

#pragma once

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

class Robotiq2fGripperJointStatePublisher {
public:
  Robotiq2fGripperJointStatePublisher(ros::NodeHandle nh) {
    joint_name_vector.push_back("robotiq_2f140_left_outer_knuckle_joint");

    out_joint_state_.name.resize(joint_name_vector.size());
    out_joint_state_.position.resize(joint_name_vector.size());

    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>(
        "/robotiq_2f_140/joint_states", 1);

    for (size_t i = 0; i < joint_name_vector.size(); ++i) {
      out_joint_state_.name[i] = joint_name_vector[i];
      out_joint_state_.position[i] = 0.0;
      map_name_to_index_[joint_name_vector[i]] = i;
    }
  }

  ~Robotiq2fGripperJointStatePublisher() {}

  void updateJointStates(double pos) {
    out_joint_state_.position[0] = pos;

    out_joint_state_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(out_joint_state_);
  }

protected:
  std::vector<std::string> joint_name_vector;
  ros::Publisher joint_state_pub_;
  std::map<std::string, int> map_name_to_index_;

  sensor_msgs::JointState out_joint_state_;
};

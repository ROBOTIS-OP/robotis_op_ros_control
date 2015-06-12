#ifndef ROBOTIS_OP_ROS_CONTROLLER_H_
#define ROBOTIS_OP_ROS_CONTROLLER_H_

// Robotis Framework
#include <LinuxDARwIn.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// ROS Control
#include <controller_manager/controller_manager.h>
#include <robotis_op_ros_control/robotis_op_hardware_interface.h>

// IMU
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <robotis_op_ros_control/robotis_op_ros_controlConfig.h>

namespace robotis_op
{
class RobotisOPRosControllerNode
{
public:
    RobotisOPRosControllerNode();
    ~RobotisOPRosControllerNode();

    void update(ros::Time time, ros::Duration period);
    void dynamicReconfigureCb(robotis_op_ros_control::robotis_op_ros_controlConfig &config, uint32_t level);

protected:

    void setTorqueOn(std_msgs::BoolConstPtr enable);

    // Callbacks
    void enableWalkCb(std_msgs::BoolConstPtr enable);
    void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg);
    void startActionCb(std_msgs::Int32 action);
    void imuCb(const sensor_msgs::Imu& msg);

    // Subscriber
    ros::Subscriber torque_on_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber start_action_sub_;
    ros::Subscriber enable_walk_sub_;
    ros::Subscriber imu_sub_;

    // Publisher
    tf::TransformBroadcaster tf_broadcaster_;

    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

};
}

#endif


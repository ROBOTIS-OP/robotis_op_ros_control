#ifndef C_INTERFACE_H_
#define ROBOTIS_OP_INTERFACE_H_

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Eigen>

// Roobotis framework
#include <LinuxDARwIn.h>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

// ROS Control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>




namespace robotis_op
{
class RobotisOPHardwareInterface
        : public Robot::MotionModule
        , public hardware_interface::RobotHW
{
public:

    ~RobotisOPHardwareInterface();

    void Initialize();
    void Process();

    // ROS Control
    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);

    //Acess to ROBOTIS Interface
    void setPIDGains(int id, double p_gain, double i_gain, double d_gain);
    void setJointStateRate(double joint_state_rate);
    void setTorqueOn(int id, bool enable);
    void setTorqueOn(bool enable);
    void enableWalking(std_msgs::BoolConstPtr enable);
    void cmdWalking(const geometry_msgs::Twist::ConstPtr& msg);
    void startAction(std_msgs::Int32 action);
    void checkFall();

    // Typedefs
    typedef boost::shared_ptr<RobotisOPHardwareInterface> Ptr;
    typedef boost::shared_ptr<const RobotisOPHardwareInterface> ConstPtr;

    static RobotisOPHardwareInterface::Ptr& Instance();

    bool print_check_fall_debug_info_;

    static const double G_ACC = 9.80665;

protected:

    RobotisOPHardwareInterface();
    RobotisOPHardwareInterface(RobotisOPHardwareInterface const&);

    RobotisOPHardwareInterface& operator=(RobotisOPHardwareInterface const&);

    Robot::JointData* getJoint(int id);

    void setBlockWrite(bool block);

    static RobotisOPHardwareInterface::Ptr singelton;

    // UIDs of joints and sensors
    static const std::string jointUIDs[Robot::JointData::NUMBER_OF_JOINTS+3];


    /** joint offsets from ROS zero to Robotis zero
  /* Robotis zero: "ready stand pose"
   * ROS zero: "fully extended arms/legs"
   **/
    static const int ros_joint_offsets[Robot::JointData::NUMBER_OF_JOINTS+3];

    // parameters
    double joint_state_intervall_;
    ros::Time last_joint_state_read_;


    // ros control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface pos_joint_interface_;
    hardware_interface::ImuSensorInterface imu_sensor_interface_;

    // Status arrays containing values for each joint (+ dummy joints to model the gripper)
    // vel_dummy_ and eff_dummy_ are required to meet the interface, but not actually used
    double cmd_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double pos_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double vel_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double eff_dummy_[Robot::JointData::NUMBER_OF_JOINTS+3];

    // IMU
    hardware_interface::ImuSensorHandle::Data imu_data_;
    double imu_orientation_[4];
    double imu_angular_velocity_[3];
    double imu_linear_acceleration_[3];


    //Robot
    Robot::LinuxCM730 *linux_cm730_;
    Robot::CM730 *cm730_;
    Robot::LinuxMotionTimer *motion_timer_;
    std::string cm730_device_, action_file_, config_file_;


    //misc
    bool block_write_;
    bool controller_running_; //to handle conflicts between ROS control and motion manager


};
}

#endif


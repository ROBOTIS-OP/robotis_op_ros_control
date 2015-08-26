#include <robotis_op_ros_control/robotis_op_ros_controller.h>
#include <LinuxMotionTimer.h>
#include <sensor_msgs/Imu.h>

namespace robotis_op
{
RobotisOPRosControllerNode::RobotisOPRosControllerNode()
{

    ros::NodeHandle nh;

    // Initialize ros control
    controller_manager_.reset(new controller_manager::ControllerManager(RobotisOPHardwareInterface::Instance().get(), nh));

    // Subscribe topics
    torque_on_sub_ = nh.subscribe("torque_on", 1, &RobotisOPRosControllerNode::setTorqueOn, this);
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 100, &RobotisOPRosControllerNode::cmdVelCb, this);
    start_action_sub_ = nh.subscribe("start_action", 100, &RobotisOPRosControllerNode::startActionCb, this);
    enable_walk_sub_ = nh.subscribe("enable_walking", 100, &RobotisOPRosControllerNode::enableWalkCb, this);
    imu_sub_ = nh.subscribe("imu", 100, &RobotisOPRosControllerNode::imuCb, this);

    ROS_INFO("Initialization of ros controller completed!");
}

RobotisOPRosControllerNode::~RobotisOPRosControllerNode()
{
    ROS_INFO("Cleaning up RobotisOPRosControllerNode...");
}

void RobotisOPRosControllerNode::setTorqueOn(std_msgs::BoolConstPtr enable)
{
    RobotisOPHardwareInterface::Instance()->setTorqueOn(enable->data);
}

void RobotisOPRosControllerNode::cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    RobotisOPHardwareInterface::Instance()->cmdWalking(msg);
}

void RobotisOPRosControllerNode::startActionCb(std_msgs::Int32 action)
{
    RobotisOPHardwareInterface::Instance()->startAction(action);
}

void RobotisOPRosControllerNode::enableWalkCb(std_msgs::BoolConstPtr enable)
{
    RobotisOPHardwareInterface::Instance()->enableWalking(enable);
}

void RobotisOPRosControllerNode::imuCb(const sensor_msgs::Imu& msg)
{
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = msg.header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = msg.orientation.w;
    //ROS and ROBITIS x and y frame definitions deviate by 90 degree around z axis
    transform.transform.rotation.x = msg.orientation.y;
    transform.transform.rotation.y = -msg.orientation.x;
    transform.transform.rotation.z = msg.orientation.z;
    tf_broadcaster_.sendTransform(transform);
}

void RobotisOPRosControllerNode::dynamicReconfigureCb(robotis_op_ros_control::robotis_op_ros_controlConfig &config, uint32_t level)
{
    RobotisOPHardwareInterface::Instance()->setPIDGains(1,config.j_shoulder_r_position_controller_p_gain, config.j_shoulder_r_position_controller_i_gain, config.j_shoulder_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(2,config.j_shoulder_l_position_controller_p_gain, config.j_shoulder_l_position_controller_i_gain, config.j_shoulder_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(3,config.j_high_arm_r_position_controller_p_gain, config.j_high_arm_r_position_controller_i_gain, config.j_high_arm_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(4,config.j_high_arm_l_position_controller_p_gain, config.j_high_arm_l_position_controller_i_gain, config.j_high_arm_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(5,config.j_low_arm_r_position_controller_p_gain, config.j_low_arm_r_position_controller_i_gain, config.j_low_arm_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(6,config.j_low_arm_l_position_controller_p_gain, config.j_low_arm_l_position_controller_i_gain, config.j_low_arm_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(7,config.j_pelvis_r_position_controller_p_gain, config.j_pelvis_r_position_controller_i_gain, config.j_pelvis_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(8,config.j_pelvis_l_position_controller_p_gain, config.j_pelvis_l_position_controller_i_gain, config.j_pelvis_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(9,config.j_thigh1_r_position_controller_p_gain, config.j_thigh1_r_position_controller_i_gain, config.j_thigh1_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(10,config.j_thigh1_l_position_controller_p_gain, config.j_thigh1_l_position_controller_i_gain, config.j_thigh1_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(11,config.j_thigh2_r_position_controller_p_gain, config.j_thigh2_r_position_controller_i_gain, config.j_thigh2_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(12,config.j_thigh2_l_position_controller_p_gain, config.j_thigh2_l_position_controller_i_gain, config.j_thigh2_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(13,config.j_tibia_r_position_controller_p_gain, config.j_tibia_r_position_controller_i_gain, config.j_tibia_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(14,config.j_tibia_l_position_controller_p_gain, config.j_tibia_l_position_controller_i_gain, config.j_tibia_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(15,config.j_ankle1_r_position_controller_p_gain, config.j_ankle1_r_position_controller_i_gain, config.j_ankle1_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(16,config.j_ankle1_l_position_controller_p_gain, config.j_ankle1_l_position_controller_i_gain, config.j_ankle1_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(17,config.j_ankle2_r_position_controller_p_gain, config.j_ankle2_r_position_controller_i_gain, config.j_ankle2_r_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(18,config.j_ankle2_l_position_controller_p_gain, config.j_ankle2_l_position_controller_i_gain, config.j_ankle2_l_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(19,config.j_pan_position_controller_p_gain, config.j_pan_position_controller_i_gain, config.j_pan_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->setPIDGains(20,config.j_tilt_position_controller_p_gain, config.j_tilt_position_controller_i_gain, config.j_tilt_position_controller_d_gain);
    RobotisOPHardwareInterface::Instance()->print_check_fall_debug_info_ = config.print_check_fall_debug_info;
}

void RobotisOPRosControllerNode::update(ros::Time time, ros::Duration period)
{
    // ros control update  cycle
    RobotisOPHardwareInterface::Instance()->read(time, period);
    controller_manager_->update(time, period);
    RobotisOPHardwareInterface::Instance()->write(time, period);

    RobotisOPHardwareInterface::Instance()->checkFall();
}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotis_op_ros_controller");

    ros::NodeHandle nh;
    double control_rate;
    nh.param("robotis_op_ros_controller/control_rate", control_rate, 125.0);

    robotis_op::RobotisOPRosControllerNode robotis_op_ros_controller_node;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_time = ros::Time::now();
    ros::Rate rate(control_rate);


    dynamic_reconfigure::Server<robotis_op_ros_control::robotis_op_ros_controlConfig> srv;
    dynamic_reconfigure::Server<robotis_op_ros_control::robotis_op_ros_controlConfig>::CallbackType cb;
    cb = boost::bind(&robotis_op::RobotisOPRosControllerNode::dynamicReconfigureCb, &robotis_op_ros_controller_node, _1, _2);
    srv.setCallback(cb);

    while (ros::ok())
    {
        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        robotis_op_ros_controller_node.update(current_time, elapsed_time);
        last_time = current_time;
    }

    return 0;
}

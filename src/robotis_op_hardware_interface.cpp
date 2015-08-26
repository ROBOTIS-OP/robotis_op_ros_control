#include <robotis_op_ros_control/robotis_op_hardware_interface.h>
#include <angles/angles.h>
#include <math.h>

#include <std_msgs/Float64.h>

namespace robotis_op
{
using namespace Robot;

const std::string RobotisOPHardwareInterface::jointUIDs[JointData::NUMBER_OF_JOINTS+3] =
{
    "j_shoulder_r",
    "j_shoulder_l",
    "j_high_arm_r",
    "j_high_arm_l",
    "j_low_arm_r",
    "j_low_arm_l",
    "j_pelvis_r",
    "j_pelvis_l",
    "j_thigh1_r",
    "j_thigh1_l",
    "j_thigh2_r",
    "j_thigh2_l",
    "j_tibia_r",
    "j_tibia_l",
    "j_ankle1_r",
    "j_ankle1_l",
    "j_ankle2_r",
    "j_ankle2_l",
    "j_pan",
    "j_tilt",
    "j_wrist_r",
    "j_wrist_l",
    "j_gripper_r",
    "j_gripper_l",
};


const int RobotisOPHardwareInterface::ros_joint_offsets[JointData::NUMBER_OF_JOINTS+3] =
{
    0,        //j_shoulder_r
    0,        //j_shoulder_l
    0,        //j_high_arm_r
    0,        //j_high_arm_l
    0,        //j_low_arm_r
    0,        //j_low_arm_l
    0,        //j_pelvis_r
    0,        //j_pelvis_l
    0,        //j_thigh1_r
    0,        //j_thigh1_l
    0,        //j_thigh2_r
    0,        //j_thigh2_l
    0,        //j_tibia_r
    0,        //j_tibia_l
    0,        //j_ankle1_r
    0,        //j_ankle1_l
    0,        //j_ankle2_r
    0,        //j_ankle2_l
    0,        //j_pan
    0,        //j_tilt
    0,        //j_wrist_r
    0,        //j_wrist_l
    0,        //j_gripper_r
    0,        //j_gripper_l
};

RobotisOPHardwareInterface::Ptr RobotisOPHardwareInterface::singelton = RobotisOPHardwareInterface::Ptr();

RobotisOPHardwareInterface::RobotisOPHardwareInterface()
    : MotionModule()
    , hardware_interface::RobotHW()
    , joint_state_intervall_(20.0)
    , last_joint_state_read_(ros::Time::now())
{
    print_check_fall_debug_info_ = false;
    block_write_ = true;
    controller_running_ = false;

    ros::NodeHandle nh;
    int wakeup_motion;
    nh.getParam("/robotis_op/robotis_op_ros_controller/wake_up_motion",wakeup_motion);

    // Initialize ROBOTIS-OP  Framework
    cm730_device_ = std::string("/dev/ttyUSB0");
    action_file_ = std::string("/robotis/Data/motion_4096.bin");
    config_file_ = std::string("/robotis/Data/config.ini");

    if(false == Action::GetInstance()->LoadFile((char *)action_file_.c_str()))
    {
        ROS_ERROR("Reading Action File failed!");
    }


    linux_cm730_ = new LinuxCM730((char *)cm730_device_.c_str());
    cm730_ = new CM730(linux_cm730_);
    MotionManager::GetInstance();

    if(false == MotionManager::GetInstance()->Initialize(cm730_))
    {
        ROS_ERROR("Initializing Motion Manager failed! ");
    }

    minIni ini =  minIni(config_file_);
    MotionManager::GetInstance()->LoadINISettings(&ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());


    motion_timer_ = new LinuxMotionTimer(MotionManager::GetInstance());
    ROS_INFO("Starting Motion Timer...");
    motion_timer_->Start();
    ROS_INFO("Finished starting Motion Timer");

    MotionManager::GetInstance()->SetEnable(true);

    /** Init(stand up) pose */
    ROS_INFO("Wake up position is %i",wakeup_motion);
    if(Action::GetInstance()->Start(wakeup_motion))
        ROS_INFO("Moving to wake up position ...");
    else
        ROS_ERROR("Wake up action failed");
    while(Action::GetInstance()->IsRunning())
    {
        usleep(8*1000);
    }

    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionStatus::m_CurrentJoints.SetEnableBody(true,true);

    /** register joints */
    // dispatching joints
    for (unsigned int id_index = 0; id_index < JointData::NUMBER_OF_JOINTS+3; id_index++)
    {
        // connect and register the joint state interface
        hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id_index], &pos_[id_index], &vel_[id_index], &eff_dummy_[id_index]);
        joint_state_interface_.registerHandle(joint_state_handle);
        // connect and register the joint position interface
        hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd_[id_index]);
        pos_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&pos_joint_interface_);

    /** register sensors */
    // IMU
    imu_data_.name = "imu";
    imu_data_.frame_id = "MP_BODY";
    imu_data_.orientation = imu_orientation_;
    imu_data_.angular_velocity = imu_angular_velocity_;
    imu_data_.linear_acceleration = imu_linear_acceleration_;
    hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    registerInterface(&imu_sensor_interface_);

}

RobotisOPHardwareInterface::RobotisOPHardwareInterface(RobotisOPHardwareInterface const&)
{
}

RobotisOPHardwareInterface::~RobotisOPHardwareInterface()
{
    delete(linux_cm730_);
    delete(cm730_);
    delete(motion_timer_);
}

RobotisOPHardwareInterface& RobotisOPHardwareInterface::operator=(RobotisOPHardwareInterface const&)
{
}

RobotisOPHardwareInterface::Ptr& RobotisOPHardwareInterface::Instance()
{
    if (!singelton)
        singelton.reset(new RobotisOPHardwareInterface());
    return singelton;
}

void RobotisOPHardwareInterface::Initialize()
{
}

void RobotisOPHardwareInterface::Process()
{
}

double lowPassFilter(double alpha, double x_new, double x_old)
{
    return alpha*x_new + (1.0-alpha)*x_old;
}

void RobotisOPHardwareInterface::read(ros::Time time, ros::Duration period)
{
    for (unsigned int i = 0; i < JointData::NUMBER_OF_JOINTS-1; i++)
    {
        cmd_[i] = std::numeric_limits<double>::quiet_NaN();
       // pos_[i] = std::numeric_limits<double>::quiet_NaN();
        vel_[i] = std::numeric_limits<double>::quiet_NaN();
        eff_dummy_[i] = std::numeric_limits<double>::quiet_NaN();
    }

    for (unsigned int joint_index = 1; joint_index < JointData::NUMBER_OF_JOINTS; joint_index++)
    {
        int id_index = joint_index-1;
        double new_pos = angles::from_degrees(MotionStatus::m_CurrentJoints.GetAngle(joint_index)+ros_joint_offsets[joint_index]);
        vel_[id_index] = (new_pos - pos_[id_index])/(double)period.toSec();
        pos_[id_index]= new_pos;
        //reading velocity and acceleration with the current firmware is not possible without jamming the cm730, state 05/2015
    }

    //IMU
    double filter_alpha = 0.5;

    //in rad/s
    imu_angular_velocity_[0] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L)-512)*1600.0*M_PI/(512.0*180.0),imu_angular_velocity_[0]);
    imu_angular_velocity_[1] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L)-512)*1600.0*M_PI/(512.0*180.0),imu_angular_velocity_[1]);
    imu_angular_velocity_[2] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L)-512)*1600.0*M_PI/(512.0*180.0),imu_angular_velocity_[2]);

    //in m/s^2
    imu_linear_acceleration_[0] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[0]);
    imu_linear_acceleration_[1] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[1]);
    imu_linear_acceleration_[2] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[2]);

    //Estimation of roll and pitch based on accelometer data, see http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
    double sign = copysignf(1.0,  imu_linear_acceleration_[2]/G_ACC);
    double roll = atan2( imu_linear_acceleration_[1]/G_ACC, sign * sqrt( imu_linear_acceleration_[0]/G_ACC* imu_linear_acceleration_[0]/G_ACC +  imu_linear_acceleration_[2]/G_ACC* imu_linear_acceleration_[2]/G_ACC));
    double pitch = -atan2( imu_linear_acceleration_[0]/G_ACC, sqrt( imu_linear_acceleration_[1]/G_ACC* imu_linear_acceleration_[1]/G_ACC +  imu_linear_acceleration_[2]/G_ACC* imu_linear_acceleration_[2]/G_ACC));
    double yaw = 0.0;

    tf2::Quaternion imu_orient;
    imu_orient.setRPY(roll, pitch, yaw);

    imu_orientation_[0] = imu_orient.getX();
    imu_orientation_[1] = imu_orient.getY();
    imu_orientation_[2] = imu_orient.getZ();
}

void RobotisOPHardwareInterface::write(ros::Time time, ros::Duration period)
{
    if(!controller_running_)
    {
        if (!std::isnan(cmd_[0]))
        {
            setBlockWrite(false);
            controller_running_=true;
            return;
        }
    }

    if(block_write_)
    {
        return;
    }

    for (unsigned int joint_index = 0; joint_index < JointData::NUMBER_OF_JOINTS-1; joint_index++)
    {
        int id_index = joint_index+1;
        if(MotionStatus::m_CurrentJoints.GetEnable(id_index))
        {
            if (std::isnan(cmd_[joint_index]))
            {
                //ROS_WARN("Cmd %i NAN", joint_index);
                continue;
            }
            else
            {

                MotionStatus::m_CurrentJoints.SetAngle(id_index,angles::to_degrees(cmd_[joint_index])+ros_joint_offsets[joint_index]);
            }
        }
    }
}


void RobotisOPHardwareInterface::setPIDGains(int id, double p_gain, double i_gain, double d_gain)
{
    MotionStatus::m_CurrentJoints.SetPGain(id, p_gain);
    MotionStatus::m_CurrentJoints.SetIGain(id, i_gain);
    MotionStatus::m_CurrentJoints.SetDGain(id, d_gain);
}


void RobotisOPHardwareInterface::setJointStateRate(double joint_state_rate)
{
    this->joint_state_intervall_ = 1.0/joint_state_rate;
}

void RobotisOPHardwareInterface::setTorqueOn(int id, bool enable)
{
    int error;
    cm730_->WriteByte(id, MX28::P_TORQUE_ENABLE, enable ? 1 : 0, &error);
}

void RobotisOPHardwareInterface::setTorqueOn(bool enable)
{
    if (enable)
        ROS_INFO("Enable torque!");
    else
        ROS_INFO("Disable torque!");

    int error;
    cm730_->WriteByte(CM730::ID_BROADCAST, MX28::P_TORQUE_ENABLE, enable, &error);
}

void RobotisOPHardwareInterface::cmdWalking(const geometry_msgs::Twist::ConstPtr& msg)
{
    double period = Walking::GetInstance()->PERIOD_TIME;
    Walking::GetInstance()->X_MOVE_AMPLITUDE=(msg->linear.x/period*1000.0);
    Walking::GetInstance()->Y_MOVE_AMPLITUDE=(msg->linear.y/period*1000.0);
    // compute the angular motion parameters to achieve the desired angular speed
    Walking::GetInstance()->A_MOVE_AMPLITUDE=(msg->angular.z/period*180.0)/(2.0*3.14159);// in degrees per step
    // ROS_INFO("Walking: periode %f x %f y %f a %f",Walking::GetInstance()->PERIOD_TIME,Walking::GetInstance()->X_MOVE_AMPLITUDE,Walking::GetInstance()->Y_MOVE_AMPLITUDE,Walking::GetInstance()->A_MOVE_AMPLITUDE);
}


void RobotisOPHardwareInterface::enableWalking(std_msgs::BoolConstPtr enable)
{
    if(!Walking::GetInstance()->IsRunning() && enable->data)
    {
        setBlockWrite(true);
        Walking::GetInstance()->m_Joint.SetEnableLowerBody(true, true);
        Walking::GetInstance()->Start();
    }
    else if(Walking::GetInstance()->IsRunning() && !enable->data)
    {
        Walking::GetInstance()->m_Joint.SetEnableLowerBody(false, true);
        Walking::GetInstance()->Stop();
        setBlockWrite(false);
    }
}

void RobotisOPHardwareInterface::checkFall()
{
    if(Walking::GetInstance()->IsRunning() && MotionStatus::FALLEN != 0)
    {
        if(1 == MotionStatus::FALLEN)
        {
            ROS_INFO("Forward Fall! Getting up...");
            std_msgs::Int32 i;
            i.data=10;
            startAction(i);
        }
        else
        {
            ROS_INFO("Backwards Fall! Getting up...");
            std_msgs::Int32 i;
            i.data=11;
            startAction(i);
        }
    }
    if(print_check_fall_debug_info_)
        if(Walking::GetInstance()->IsRunning() || MotionStatus::FALLEN != 0)
            ROS_INFO("checkFall: walking %i fallen %i fb_acc %i rl_acc %i fb_gyro %i rl_gyro %i", Walking::GetInstance()->IsRunning(), MotionStatus::FALLEN,MotionStatus::FB_ACCEL,MotionStatus::RL_ACCEL,MotionStatus::FB_GYRO,MotionStatus::RL_GYRO);
}

void RobotisOPHardwareInterface::startAction(std_msgs::Int32 action_index)
{
    setBlockWrite(true);
    //enable MotionManager if necessary
    if(!MotionManager::GetInstance()->GetEnable())
        MotionManager::GetInstance()->SetEnable(true);
    // stop walking if necessary
    if(Walking::GetInstance()->IsRunning())
        Walking::GetInstance()->Stop();
    // stop any previous action
    if(Action::GetInstance()->IsRunning())
        Action::GetInstance()->Stop();

    // enable all the body servos to the action module
    Action::GetInstance()->m_Joint.SetEnableBody(true,true);
    if(!Action::GetInstance()->Start((int)action_index.data))
    {
        ROS_ERROR("RobotisOPHardwareInterface: Could not start Action %i",(int)action_index.data);
    }

    while(Action::GetInstance()->IsRunning())
    {
        usleep(8*1000);
    }

    setBlockWrite(false);
    Action::GetInstance()->m_Joint.SetEnableBody(false,true);
    MotionStatus::m_CurrentJoints.SetEnableBody(true,true);
}

void RobotisOPHardwareInterface::setBlockWrite(bool block)
{
    if(block)
    {
        block_write_ = true;
        //ROS_INFO("Blocked Write");
    }
    else
    {
        block_write_ = false;
        std::vector<ros::Publisher> joint_publisher;
        ros::NodeHandle n;

        std_msgs::Float64 angle_msg;

        ros::Publisher j15_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_l_position_controller/command", 100);
        ros::Publisher j14_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_r_position_controller/command", 100);
        ros::Publisher j17_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_l_position_controller/command", 100);
        ros::Publisher j16_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_r_position_controller/command", 100);
        ros::Publisher j03_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_high_arm_l_position_controller/command", 100);
        ros::Publisher j02_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_high_arm_r_position_controller/command", 100);
        ros::Publisher j05_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_low_arm_l_position_controller/command", 100);
        ros::Publisher j04_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_low_arm_r_position_controller/command", 100);
        ros::Publisher j18_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_pan_position_controller/command", 100);
        ros::Publisher j07_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_l_position_controller/command", 100);
        ros::Publisher j06_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_r_position_controller/command", 100);
        ros::Publisher j01_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_l_position_controller/command", 100);
        ros::Publisher j00_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_r_position_controller/command", 100);
        ros::Publisher j09_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_l_position_controller/command", 100);
        ros::Publisher j08_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_r_position_controller/command", 100);
        ros::Publisher j11_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_l_position_controller/command", 100);
        ros::Publisher j10_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_r_position_controller/command", 100);
        ros::Publisher j13_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_tibia_l_position_controller/command", 100);
        ros::Publisher j12_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_tibia_r_position_controller/command", 100);
        ros::Publisher j19_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_tilt_position_controller/command", 100);

        joint_publisher.push_back(j00_pub);
        joint_publisher.push_back(j01_pub);
        joint_publisher.push_back(j02_pub);
        joint_publisher.push_back(j03_pub);
        joint_publisher.push_back(j04_pub);
        joint_publisher.push_back(j05_pub);
        joint_publisher.push_back(j06_pub);
        joint_publisher.push_back(j07_pub);
        joint_publisher.push_back(j08_pub);
        joint_publisher.push_back(j09_pub);
        joint_publisher.push_back(j10_pub);
        joint_publisher.push_back(j11_pub);
        joint_publisher.push_back(j12_pub);
        joint_publisher.push_back(j13_pub);
        joint_publisher.push_back(j14_pub);
        joint_publisher.push_back(j15_pub);
        joint_publisher.push_back(j16_pub);
        joint_publisher.push_back(j17_pub);
        joint_publisher.push_back(j18_pub);
        joint_publisher.push_back(j19_pub);

        for(unsigned int i = 0; i<joint_publisher.size(); ++i)
        {

            angle_msg.data = pos_[i];
            joint_publisher[i].publish(angle_msg);

        }
        //ROS_INFO("Unblocked Write");
    }
}
}

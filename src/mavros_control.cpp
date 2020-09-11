#include <std_msgs/String.h>
#include <string>
#include "mavros_control.h"
#include <rosbag/bag.h>

static bool isInit;

static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped local_pose;
static geometry_msgs::TwistStamped local_velocity;
static geometry_msgs::PoseStamped goal_pose;

static ros::ServiceClient streamRate_client;
static ros::ServiceClient setMode_client;
static ros::ServiceClient arming_client;
static ros::ServiceClient takeoff_client;
static ros::ServiceClient land_client;
static ros::Time previous_time;


// RPYT
static float cmd_roll;   // rad
static float cmd_pitch;  // rad
static float cmd_yaw;    // rad/s
static float cmd_thrust; // [0,1]

static int status = false; 


// Quaternion to Euler angles (radian)
void quat_to_euler(geometry_msgs::Quaternion Orientation, float *roll, float *pitch, float *yaw)
{
    tfScalar roll_rad, pitch_rad, yaw_rad;
    tf::Matrix3x3(
        tf::Quaternion(
            Orientation.x,
            Orientation.y,
            Orientation.z,
            Orientation.w)
    ).getRPY(roll_rad, pitch_rad, yaw_rad);

    *roll  = float(roll_rad);
    *pitch = float(pitch_rad);
    *yaw   = float(yaw_rad);
}


// Euler angles (radian) to Quaternion
void euler_to_quat(geometry_msgs::Quaternion *Orientation, float roll_rad, float pitch_rad, float yaw_rad)
{
    tf2::Quaternion Quat;
    Quat.setRPY(roll_rad, pitch_rad, yaw_rad);
    
    *Orientation = tf2::toMsg(Quat); 
}

// Set flight mode
bool set_mode(const char *new_mode)
{   
    mavros_msgs::SetMode service;
    service.request.custom_mode = new_mode;
    setMode_client.call(service);

    return service.response.mode_sent;
}

// Set stream rate
void set_stream_rate(uint8_t stream_id, uint16_t message_rate)
{
    mavros_msgs::StreamRate service;
    service.request.stream_id = stream_id;
    service.request.message_rate = message_rate;
    service.request.on_off = true;
    streamRate_client.call(service);
}

// Callback function
void stateCallback(const mavros_msgs::State::ConstPtr& msg) 
{
    current_state = *msg;
}

// Callback function
void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pose = *msg;
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_roll   = msg->linear.x;
    cmd_pitch  = -msg->linear.y;
    cmd_yaw    = msg->angular.z;

    float MAX_UP_SPEED = 2.5;
    float MAX_DOWN_SPEED = 1.5;

    if (msg->linear.z > 0.0) 
    {
        cmd_thrust = (msg->linear.z / MAX_UP_SPEED) * 0.5 + 0.5;

    } else if (msg->linear.z < 0.0) 
    {
        cmd_thrust = (msg->linear.z / MAX_DOWN_SPEED) * 0.5 + 0.5;

    } else 
    {
        cmd_thrust = msg->linear.z;
    }
}

void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    local_velocity = *msg;
}


// Initialize
void mavrosControlInit()
{
    if (isInit) return;

    previous_time = ros::Time::now();
    while (!current_state.connected) {
        if (ros::Time::now() - previous_time > ros::Duration(10.0)) { // exit if the connection time is too long
            ROS_ERROR("Unable to connect!");
            ROS_INFO("Exit the program.");
            ros::shutdown();
            exit(1);
        }
        ros::spinOnce(); // Otherwise, wait until connect to the vehicle
    }

    previous_time = ros::Time::now();    
    isInit = true;
}


bool mavrosControlCheck()
{
    previous_time = ros::Time::now();
    
    // if (!current_state.armed) { // cannot run the program when the vehicle is not armed
    //     ROS_ERROR("Arm the vehicle first!");
    //     ROS_INFO("Exit the program.");
    //     ros::shutdown();
    //     exit(1);
    // }

    while (current_state.mode != "GUIDED")
    {
        if (ros::Time::now() - previous_time > ros::Duration(5.0)) {
            ROS_WARN("Unable to switch the flight mode!");
            return false;
        }

        if (set_mode("GUIDED"))
        {
            ROS_INFO("Switch to GUIDED mode.");
            ros::Duration(0.5).sleep();
        } else
        {
            ROS_INFO("Attemped to switch to GUIDED mode");
        }
        ros::spinOnce();
    }

    previous_time = ros::Time::now();
    return true;
}


//
mavros_msgs::AttitudeTarget set_attitudeTarget(float roll, float pitch, float yaw, float thrust)
{
    mavros_msgs::AttitudeTarget attitudeDesired;
    attitudeDesired.header.stamp = ros::Time::now();
    attitudeDesired.header.seq++;

    euler_to_quat(&attitudeDesired.orientation, roll, pitch, yaw);
    
    attitudeDesired.body_rate.x = 0.0f;
    attitudeDesired.body_rate.y = 0.0f;
    attitudeDesired.body_rate.z = 0.0f;

    attitudeDesired.type_mask = 0b00000111;
        
    attitudeDesired.thrust = thrust;

    return attitudeDesired;
}


int main(int argc, char **argv)
{
    // ROS node
    ros::init(argc, argv, "mavros_control"); 
    ros::NodeHandle n;

    ros::Rate loop_rate(10.0); // loop rate in Hz

    // Subscriber
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 5, stateCallback);
    ros::Subscriber localPose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 5, localPoseCallback);
    ros::Subscriber cmd_sub = n.subscribe<geometry_msgs::Twist>("/mavros_control/cmd_vel", 5, cmdCallback);
    ros::Subscriber localVel_sub = n.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 5, localVelCallback);


    // ServiceClient
    streamRate_client = n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    setMode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    // land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
 
    // Publisher
    ros::Publisher attitude_pub = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 5);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros_control/goal", 5);
   

    // // bag file definition
    // std::time_t now = time(0);
    // struct tm * timeinfo = localtime(&(now));
    // char buffer [30];
    // strftime(buffer,30,"%Y_%h_%d_%H_%M_%S.bag", timeinfo);
    // char filename[50] = "/home/peng/SITL_LOG/";
    // //char filename[50] = "/media/peng/Samsung/";
    // //char filename[50] = "/home/lab/Documents/";
    // std::strcat(filename, buffer);

    // rosbag::Bag bag;
    // bag.open(filename, rosbag::bagmode::Write);

    // Start the program
    mavrosControlInit();
    ROS_INFO("Connected to the vehicle!");
    
    mavrosControlCheck();
    ROS_INFO("Current flight mode: %s", current_state.mode.c_str());
    ROS_INFO("Ready to Start!");

    ros::Duration(2.0).sleep();

    ros::spinOnce();
    ros::param::set("/mavros_control/status", status);
    ros::param::set("/mavros_control/reset", true);
    goal_pose = local_pose;
    goal_pose.pose.position.y = goal_pose.pose.position.y + 20.0;
    goal_pose.pose.position.x = goal_pose.pose.position.x + 20.0;
    goal_pose.pose.position.z = goal_pose.pose.position.z + 15.0;
    goal_pub.publish(goal_pose);
    goal_pub.publish(goal_pose);
        

    // Main loop
    while (ros::ok())
    {
        status = 1;
        // Adjust the stream rate
        set_stream_rate((uint8_t)6, (uint16_t)10);  // STREAM_POSITION
        set_stream_rate((uint8_t)10, (uint16_t)10); // STREAM_EXTRA1
        
        goal_pub.publish(goal_pose);

        // bag.write("/local_pose", ros::Time::now(), local_pose);
        // bag.write("/local_vel", ros::Time::now(), local_velocity);

        ros::param::set("/mavros_control/status", status);

        // ROS_INFO("%f", cmd_yaw);
        attitude_pub.publish(set_attitudeTarget(cmd_roll * (MY_PI/180.0f), 
                                                cmd_pitch * (MY_PI/180.0f), 
                                                cmd_yaw * (MY_PI/180.0f), 
                                                cmd_thrust));
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    // bag.close();
    return 0;
}


// bool arm_vehicle(ros::ServiceClient& client)
// {
//     mavros_msgs::CommandBool service;
//     service.request.value = true;
//     client.call(service);
//     return service.response.success;
// }

// bool takeoff(ros::ServiceClient& client)
// {
//     mavros_msgs::CommandTOL service;
//     service.request.altitude = 5.0f;
//     service.request.latitude = 0.0f;
//     service.request.longitude = 0.0f;
//     service.request.min_pitch = 0.0f;
//     service.request.yaw = 0.0f;
//     client.call(service);
//     return service.response.success;
// }


// // Land
// bool land(ros::ServiceClient& client)
// {
//     mavros_msgs::CommandTOL service;
//     service.request.altitude = 0.0f;
//     service.request.latitude = 0.0f;
//     service.request.longitude = 0.0f;
//     service.request.min_pitch = 0.0f;
//     service.request.yaw = 0.0f;
//     client.call(service);
//     return service.response.success;
// }
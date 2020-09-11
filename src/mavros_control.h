#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#ifndef MAVROS_CONTROL_H_
#define MAVROS_CONTROL_H_

// #include <stdbool.h>

#define MY_PI   3.1415926f

enum class Number : uint8_t {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
    ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
    AUTOROTATE =   26,  // Autonomous autorotation
    NEPTUNE =      27,  // Custom flight mode to lock pitch and yaw and set a constant velocity
};



void mavrosControlInit();

/**
 * Euler angles (radian) to Quaternion
 */
void euler_to_quat(geometry_msgs::Quaternion *Orientation, float roll_rad, float pitch_rad, float yaw_rad);

/**
 * Quaternion to Euler angles (radian)
 */
void quat_to_euler(geometry_msgs::Quaternion Orientation, float *roll, float *pitch, float *yaw);

/**
 * Set a new flight mode
 */
bool set_mode(const char *new_mode, ros::ServiceClient& client);

/**
 * Arm the vehicle
 */
bool arm_vehicle(ros::ServiceClient& client);

/**
 * Take-off
 */
bool takeoff(ros::ServiceClient& client);

/**
 * Land
 */
bool land(ros::ServiceClient& client);

#endif
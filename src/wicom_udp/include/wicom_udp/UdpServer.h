#include <ros/ros.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <thread>

// Robotic Arm library
#include <wicom_roboarm/SetAngle.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h> 
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <vector>
#include <array>
#include <math.h>

#include "wicom_udp/Takeoff.h"
#include "wicom_udp/Navigate.h"
#include "wicom_udp/NavigateGlobal.h"

#pragma pack(1)
using std::string;

// ADD position in Offboard Mode
#define UAVLINK_MSG_ID_POSITION_CONTROL 8
#define UAVLINK_MSG_ID_POSITION_FEEDBACK 9
#define UAVLINK_CMD_POSITION_CONTROL_MODE 27

//
#define UAVLINK_MSG_ID_COMMAND 5
#define UAVLINK_MSG_ID_COMMAND_LEN 18

// Setup sampling data for AI training
#define UAVLINK_CMD_TRAIN_TOGGLE 29   // param1: 1 enable, 0 disable
#define UAVLINK_CMD_VX_OVERRIDE 30    // param1: 1 enable, 0 disable; param2: value (m/s)

// RCIn - POSCTL, ALTCTL
// #define UAVLINK_MSG_ID_RC_CHANNELS 11
#define UAVLINK_CMD_TAKEOFF 22
#define UAVLINK_CMD_ARM_DISARM 23
#define UAVLINK_CMD_LAND 24
#define UAVLINK_CMD_FLYTO 25
#define UAVLINK_CMD_SET_MODE 26
#define UAVLINK_MSG_ID_STATE 1
#define UAVLINK_MSG_ID_STATE_LEN 4

#define UAVLINK_MSG_ID_MANUAL_CONTROL 6
#define UAVLINK_MSG_ID_MANUAL_CONTROL_LEN 16

#define UAVLINK_MSG_ID_VELOCITY_CONTROL 12
#define UAVLINK_MSG_ID_VELOCITY_CONTROL_LEN 17

#define UAVLINK_MSG_ID_GLOBAL_POSITION_INT 2
#define UAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 24

#define UAVLINK_MSG_ID_LOCAL_POSITION_INT 3
#define UAVLINK_MSG_ID_LOCAL_POSITION_INT_LEN 24

#define UAVLINK_MSG_ID_POSITION_CONTROL_LEN 17
#define UAVLINK_MSG_ID_POSITION_FEEDBACK_LEN 13

#define UAVLINK_MSG_ID_DRONE_STATUS 10
#define UAVLINK_MSG_ID_DRONE_STATUS_LEN (sizeof(uavlink_drone_status_t))

#define UAVLINK_MSG_ID_SERVO_CONTROL 13
#define UAVLINK_MSG_ID_SERVO_CONTROL_LEN (sizeof(uavlink_servo_channels_t))

#define MAX_VOLTAGE 16
#define MIN_VOLTAGE 14
#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout))

#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

const string mode_define[] = {
    "MANUAL",        // 0
    "ALTCTL",        // 1
    "POSCTL",        // 2
    "OFFBOARD",      // 3
    "AUTO.LAND",     // 4
    "AUTO.MISSION",  // 5
    "AUTO.LOITER",   // 6
    "AUTO.RTL",      // 7
    "ACRO",          // 8
    "RATTITUDE"      // 9
};

/* ******************************************** 
*************** VELOCITY CONTROL ***************
*********************************************** */
typedef struct __uavlink_velocity_control_t
{
    float vx;       // linear.x (m/s)
    float vy;       // linear.y (m/s)
    float vz;       // linear.z (m/s)
    float yaw_rate; // angular.z (rad/s)
    uint8_t frame;  // 0: ENU local (map), 1: body
} uavlink_velocity_control_t;

// RC channels mapping
// RC channels message (8 channels, uint16_t each)
typedef struct __uavlink_rc_channels_t
{
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
} uavlink_rc_channels_t;
#define UAVLINK_MSG_ID_RC_CHANNELS 11
#define UAVLINK_MSG_ID_RC_CHANNELS_LEN (sizeof(uavlink_rc_channels_t))
// End RC channels mapping

// Function servo control
// Servo channels 5 values (float, degrees)
typedef struct __uavlink_servo_channels_t
{
    float ch0;
    float ch1;
    float ch2;
    float ch3;
    float ch4;
} uavlink_servo_channels_t;


void readingSocketThread();
void writeSocketMessage(char *, int);
int createSocket(int);
void handleState(const mavros_msgs::State &);
void handleGlobalPosition(const sensor_msgs::NavSatFix &);
void stateTimedOut(const ros::TimerEvent &);
void handleBatteryState(const sensor_msgs::BatteryState &);
// Function handle send msg
void handle_Write_State(char buff[]);

// Working byte library
inline int8_t ReadINT8(char *ByteArray, int32_t Offset)
{
	int8_t result;
	memcpy(&result, ByteArray + Offset, sizeof(int8_t));
	return result;
};
inline int16_t ReadINT16(char *ByteArray, int32_t Offset)
{
	int16_t result;
	memcpy(&result, ByteArray + Offset, sizeof(int16_t));
	return result;
};

inline int32_t ReadINT32(char *ByteArray, int32_t Offset)
{
	int32_t result;
	memcpy(&result, ByteArray + Offset, sizeof(int32_t));
	return result;
};
// Define message uavlink
typedef struct __uavlink_message_t
{
	uint8_t msgid;
	uint8_t len;
	uint8_t payload64[255];
} uavlink_message_t;

typedef struct __uavlink_state_t
{
	int8_t connected;
	int8_t armed;
	int8_t mode;
	int8_t battery_remaining;
} uavlink_state_t;

static inline uint16_t uavlink_state_encode(uavlink_message_t *msg, const uavlink_state_t *uavlink_state)
{
	uavlink_state_t packet;
	packet.connected = uavlink_state->connected;
	packet.armed = uavlink_state->armed;
	packet.mode = uavlink_state->mode;
	packet.battery_remaining = uavlink_state->battery_remaining;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_STATE_LEN);
	msg->msgid = UAVLINK_MSG_ID_STATE;
	msg->len = UAVLINK_MSG_ID_STATE_LEN;
	return 1;
}

static inline void uavlink_state_decode(const uavlink_message_t *msg, uavlink_state_t *state)
{

	uint8_t len = msg->len < UAVLINK_MSG_ID_STATE_LEN ? msg->len : UAVLINK_MSG_ID_STATE_LEN;
	memset(state, 0, UAVLINK_MSG_ID_STATE_LEN);
	memcpy(state, _MAV_PAYLOAD(msg), len);
}

typedef struct __uavlink_msg_manual_control_t
{
	int x;
	int y;
	int z;
	int r;
} uavlink_msg_manual_control;

static inline void uavlink_manual_control_decode(const uavlink_message_t *msg, uavlink_msg_manual_control *uavlink_manual_control)
{
	memset(uavlink_manual_control, 0, UAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
	memcpy(uavlink_manual_control, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_MANUAL_CONTROL_LEN);
}

/* ******************************************** 
***************OFFBOARD MODE*********************
*********************************************** */

typedef struct __uavlink_position_control_t
{
    float x;        // Position X (m)
    float y;        // Position Y (m) 
    float z;        // Position Z (m)
    float yaw;      // Yaw angle (radians)
    uint8_t frame;  // Coordinate frame (0: local, 1: global)
} uavlink_position_control_t;


typedef struct __uavlink_position_feedback_t
{
    uint8_t success;
    float error_x;
    float error_y;
    float error_z;
} uavlink_position_feedback_t;


// Position Control encode
static inline uint16_t uavlink_position_control_encode(uavlink_message_t *msg, const uavlink_position_control_t *position_control)
{
    uavlink_position_control_t packet;
    packet.x = position_control->x;
    packet.y = position_control->y;
    packet.z = position_control->z;
    packet.yaw = position_control->yaw;
    packet.frame = position_control->frame;
    
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_POSITION_CONTROL_LEN);
    msg->msgid = UAVLINK_MSG_ID_POSITION_CONTROL;
    msg->len = UAVLINK_MSG_ID_POSITION_CONTROL_LEN;
    return 1;
}

// Position Control decode
static inline void uavlink_position_control_decode(const uavlink_message_t *msg, uavlink_position_control_t *position_control)
{
    memset(position_control, 0, UAVLINK_MSG_ID_POSITION_CONTROL_LEN);
    memcpy(position_control, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_POSITION_CONTROL_LEN);
}

// Position Feedback encode
static inline uint16_t uavlink_position_feedback_encode(uavlink_message_t *msg, const uavlink_position_feedback_t *position_feedback)
{
    uavlink_position_feedback_t packet;
    packet.success = position_feedback->success;
    packet.error_x = position_feedback->error_x;
    packet.error_y = position_feedback->error_y;
    packet.error_z = position_feedback->error_z;
    
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_POSITION_FEEDBACK_LEN);
    msg->msgid = UAVLINK_MSG_ID_POSITION_FEEDBACK;
    msg->len = UAVLINK_MSG_ID_POSITION_FEEDBACK_LEN;
    return 1;
}

/***************** Drone State *******************/
typedef struct __uavlink_drone_status_t
{
    float altitude;      // current altitude (m)
    int8_t battery;      // current battery percentage (%)
    double latitude;     // current latitude
    double longitude;    // current longitude
    float pos_x;         // local X position (m)
    float pos_y;         // local Y position (m)
    float pos_z;         // local Z position (m)
	float vx;			 // local X velocity (m/s)
	float vy;			 // local Y velocity (m/s)
	float vz;			 // local Z velocity (m/s)
	float roll;			 // roll angle (radians)
	float pitch;		 // pitch angle (radians)
	float yaw;			 // yaw angle (radians)
	float dist_short;    // VL53 short range (m)
    float dist_long;     // VL53 long range  (m)
} uavlink_drone_status_t;

static inline uint16_t uavlink_drone_status_encode(uavlink_message_t *msg, const uavlink_drone_status_t *status)
{
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), status, UAVLINK_MSG_ID_DRONE_STATUS_LEN);
    msg->msgid = UAVLINK_MSG_ID_DRONE_STATUS;
    msg->len = UAVLINK_MSG_ID_DRONE_STATUS_LEN;
    return 1;
}

/* **************WayPoint******************* */ 

typedef struct __uavlink_command_t
{
	uint16_t command;
	float param1;
	float param2;
	float param3;
	float param4;
} uavlink_command_t;

static inline uint16_t uavlink_command_encode(uavlink_message_t *msg, const uavlink_command_t *uavlink_command)
{
	uavlink_command_t packet;
	packet.command = uavlink_command->command;
	packet.param1 = uavlink_command->param1;
	packet.param2 = uavlink_command->param2;
	packet.param4 = uavlink_command->param4;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, UAVLINK_MSG_ID_COMMAND_LEN);
	msg->msgid = UAVLINK_MSG_ID_COMMAND;
	msg->len = UAVLINK_MSG_ID_COMMAND_LEN;
	return 1;
}

static inline void uavlink_command_decode(const uavlink_message_t *msg, uavlink_command_t *uavlink_command)
{
	int index = 0;
	memset(uavlink_command, 0, UAVLINK_MSG_ID_COMMAND_LEN);
	// memcpy(&uavlink_command, _MAV_PAYLOAD(msg),UAVLINK_MSG_ID_COMMAND_LEN);
	memcpy(&uavlink_command->command, _MAV_PAYLOAD(msg), 2);
	memcpy(&uavlink_command->param1, _MAV_PAYLOAD(msg) + (index += 2), 4);
	memcpy(&uavlink_command->param2, _MAV_PAYLOAD(msg) + (index += 4), 4);
	memcpy(&uavlink_command->param3, _MAV_PAYLOAD(msg) + (index += 4), 4);
	memcpy(&uavlink_command->param4, _MAV_PAYLOAD(msg) + (index += 4), 4);
}


// Function velocity control message - #AI training
static inline void uavlink_velocity_control_decode(const uavlink_message_t *msg, uavlink_velocity_control_t *vc)
{
    memset(vc, 0, UAVLINK_MSG_ID_VELOCITY_CONTROL_LEN);
    memcpy(vc, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_VELOCITY_CONTROL_LEN);
}

// Message helper define
uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
	while (length > 1 && payload[length - 1] == 0)
	{
		length--;
	}
	return length;
}

int8_t mode_to_int(string mode)
{
	for (int8_t i = 0; i < (end(mode_define) - begin(mode_define)); i++)
	{
		if (mode == mode_define[i])
			return i;
	}
	return -1;
}

int8_t battery_remaining_calculate(float voltage)
{
	return (int8_t)((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100);
}

uint16_t uavlink_msg_to_send_buffer(uint8_t *buf, const uavlink_message_t *msg)
{
	buf[0] = msg->msgid;
	buf[1] = msg->len;
	memcpy(&buf[2], _MAV_PAYLOAD(msg), msg->len);
	return msg->len + 1 + 1;
}

// Function handle receiver msg
void handle_msg_manual_control(uavlink_message_t message);
void handle_command(uavlink_message_t message);

// Function handle command
void handle_cmd_arm_disarm(bool flag);
void handle_cmd_set_mode(int mode);
void handle_cmd_takeoff(float altitude);
void handle_cmd_land();

// Function handle position in Offboard
void handle_msg_position_control(uavlink_message_t message);
void handle_cmd_position_control_mode(bool enable);
void send_position_feedback(bool success, float error_x, float error_y, float error_z);

// Function handle RCIn control
void handle_msg_rc_channels(uavlink_message_t message);

// Function send drone status
void send_drone_status();
void handleVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
void handleImu(const sensor_msgs::Imu::ConstPtr& msg);
void handleRangeShort(const sensor_msgs::Range::ConstPtr& msg);
void handleRangeLong(const sensor_msgs::Range::ConstPtr& msg);

// Function handle velocity control
void handle_msg_velocity_control(uavlink_message_t message);

// Function handle servo control
void handle_msg_servo_control(const uavlink_message_t& msg);

#include "wicom_udp/UdpServer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Range.h>
#include <cmath>
// Robotic Arm library
#include <wicom_roboarm/SetAngle.h>

/* ---- Global variable ---- */
// Socket server
int sockfd;
sockaddr_in android_addr;
socklen_t android_addr_size = sizeof(android_addr);

bool check_busy;
bool check_take_off;

// Timing
ros::Timer state_timeout_timer; // Check timeout connecting
ros::Duration arming_timeout;
ros::Duration state_timeout;

// ROS Service
ros::ServiceClient takeoff_srv, land_srv;

// ROBOTIC ARM
ros::ServiceClient set_angle_cli_;  // client call /pca9685_servo/set_angle
void initServoBridge(ros::NodeHandle& nh);

// Training data record 
ros::ServiceClient dataset_toggle_srv; // for /dataset_logger/toggle

// ROS Message
sensor_msgs::NavSatFix global_msg;			   // message from topic "/mavros/global_position/global"
mavros_msgs::State state;					   // State robot
mavros_msgs::ManualControl manual_control_msg; // Manual control msg
sensor_msgs::BatteryState battery_msg;		   // message from /mavros/battery
mavros_msgs::OverrideRCIn rc_msg; 			   // RC message

// Distance Sensor vl53
float g_vl53_short_m = NAN;  // short range (channel VL53L0X)
float g_vl53_long_m  = NAN;  // long range  (channel VL53L1X)

// param
int port;

// Service clients
ros::ServiceClient arming, set_mode;

// Subcriber
ros::Subscriber state_sub;

// Publisher
ros::Publisher manual_control_pub;
ros::Publisher rc_override_pub; 	// add publisher RC override
bool check_receiver = false;

// ROS Publisher for position in offboard
ros::Publisher position_control_pub;
geometry_msgs::PoseStamped position_cmd_msg;

// velocity control publishers offboard mode - get data for AI training
ros::Publisher vel_cmd_pub_stamped;    // /mavros/setpoint_velocity/cmd_vel (TwistStamped)
ros::Publisher vel_cmd_pub_unstamped;  // /mavros/setpoint_velocity/cmd_vel_unstamped (Twist)

// Position control state
bool position_control_active = false;
ros::Time last_position_cmd_time;
ros::Duration position_cmd_timeout = ros::Duration(2.0);

// Drone status
double local_x = 0.0;   // local x
double local_y = 0.0;   // local y
double local_z = 0.0;   // altitude

geometry_msgs::TwistStamped velocity_msg; 	// velocity
sensor_msgs::Imu imu_msg; 					// orientation

// Setup Vx override for training
static bool   vx_override_enable = false;
static double vx_override_value  = 0.05; // m/s

// Altitude hold
static bool   alt_hold_enable = true;
static double alt_target_z    = 0.5;  // m (ENU up)
static double kp_z            = 1.0;
static double kd_z            = 0.6;
static double max_z_vel       = 0.5;  // m/s
static bool   invert_z_sign   = false;

static inline double clampd(double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// PD hold altitude: local_z, vz /mavros/local_position/velocity_local
static inline double compute_vz_hold() {
    const double z_now   = local_z;                       // m
    const double vz_meas = velocity_msg.twist.linear.z;   // m/s (ENU)
    const double err  = alt_target_z - z_now;             // ENU up
    double vz_cmd = kp_z * err - kd_z * vz_meas;          // PD
    if (invert_z_sign) vz_cmd = -vz_cmd;
    return clampd(vz_cmd, -max_z_vel, max_z_vel);
}

// --------------------------------ROBOTIC ARM------------------------------- //
void initServoBridge(ros::NodeHandle& nh) 
{
	if (!set_angle_cli_) 
	{
	    set_angle_cli_ = nh.serviceClient<wicom_roboarm::SetAngle>("/pca9685_servo/set_angle");
	}
  ROS_INFO("[UdpServer] Servo bridge ready: service [/pca9685_servo/set_angle]");
}

void handle_cmd_set_mode(int mode)
{
    if (mode >= 0 && mode < (int)(sizeof(mode_define)/sizeof(mode_define[0])))
    {
        if (state.mode != mode_define[mode])
        {
            mavros_msgs::SetMode sm;
            sm.request.custom_mode = mode_define[mode];

            if (!set_mode.call(sm))
            {
                ROS_ERROR("Error calling set_mode service");
            }
            else
            {
                ROS_INFO("Set mode to: %s", mode_define[mode].c_str());
            }
        }
    }
    else
    {
        ROS_ERROR("Invalid mode index: %d", mode);
    }
}

void handle_cmd_arm_disarm(bool flag)
{
	if (!TIMEOUT(state, state_timeout) && !state.armed && flag == true) // Arming
	{
		ros::Time start = ros::Time::now();
		ROS_INFO("arming");
		mavros_msgs::CommandBool srv;
		srv.request.value = true;
		if (!arming.call(srv))
		{
			throw std::runtime_error("Error calling arming service");
		}

		// wait until armed
		while (ros::ok())
		{
			if (state.armed)
			{
				break;
			}
			else if (ros::Time::now() - start > arming_timeout)
			{
				string report = "Arming timed out";
				ROS_INFO("ARMING TIMEOUT... TRY AGAIN!!");
				break;
			}
		}
	}
	else if (!TIMEOUT(state, state_timeout) && state.armed && flag == false) // Disarming
	{
		ROS_INFO("DISARM"); // TODO: handle disarm motor
	}
}

void handle_cmd_takeoff(float altitude)
{
	wicom_udp::Takeoff takeoff;
	takeoff.request.z = altitude;
	ROS_INFO("[ALTITUDE] alt=%.2f", altitude);

	if (takeoff_srv.call(takeoff))
	{
		ROS_INFO("CALLED TAKEOFF SRV!");
		check_take_off = true;
	}
	else
	{
		ROS_ERROR("Failed to call service takeoff");
		return;
	}
	return;
}

void handle_cmd_land()
{
	std_srvs::Trigger land;
	if (land_srv.call(land))
	{
		ROS_INFO("CALLED LAND SRV!");
		check_take_off = false;
	}
	else
	{
		ROS_ERROR("Failed to call service land");
		return;
	}
	return;
}

void handle_command(uavlink_message_t message)
{
	// debug: show raw payload bytes and length
    ROS_DEBUG("handle_command: msgid=%u len=%u", message.msgid, message.len);
    std::string hex;
    for (int i = 0; i < message.len; ++i)
    {
        char tmp[8];
        snprintf(tmp, sizeof(tmp), "%02X ", (unsigned char)message.payload64[i]);
        hex += tmp;
    }
    ROS_DEBUG("command payload: %s", hex.c_str());


	uavlink_command_t command_msg;
	uavlink_command_decode(&message, &command_msg);
	ROS_INFO("cmd: %d", command_msg.command);

	switch (command_msg.command)
	{
		case UAVLINK_CMD_SET_MODE:
			handle_cmd_set_mode((int)command_msg.param1);
			break;

		case UAVLINK_CMD_ARM_DISARM:
			handle_cmd_arm_disarm((bool)command_msg.param1);
			break;

		case UAVLINK_CMD_TAKEOFF:
			handle_cmd_takeoff((float)command_msg.param1);
			break;

		case UAVLINK_CMD_LAND:
			handle_cmd_land();
			break;
			//position control in offboard
		case UAVLINK_CMD_POSITION_CONTROL_MODE:
			handle_cmd_position_control_mode((bool)command_msg.param1);
			break;

		case UAVLINK_CMD_TRAIN_TOGGLE: {
			bool enable = (bool)command_msg.param1;     // 1=On, 0=Off
			if (!dataset_toggle_srv.exists()) {
				dataset_toggle_srv.waitForExistence(ros::Duration(2.0));
			}
			std_srvs::SetBool srv; srv.request.data = enable;
			if (dataset_toggle_srv.call(srv)) {
				ROS_INFO("[UdpServer] Training logging %s (%s)",
						enable ? "ENABLED" : "DISABLED",
						srv.response.message.c_str());
			} else {
				ROS_ERROR("[UdpServer] Failed to call /dataset_logger/toggle");
			}
			break;
		}

		case UAVLINK_CMD_VX_OVERRIDE: {
			bool en = static_cast<bool>(command_msg.param1);  // 1=On, 0=Off
			vx_override_enable = en;
			ROS_INFO("[UdpServer] vx override %s (default vx=%.3f m/s)",
					en ? "ENABLED" : "DISABLED", vx_override_value);
			break;
		}

		default:
			break;
	}
}

// Helper: get current yaw (rad) from IMU; fallback = 0 if quaternion is invalid - #AI
static inline double get_current_yaw()
{
    const auto& o = imu_msg.orientation;
    double qx = o.x, qy = o.y, qz = o.z, qw = o.w;
    double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm < 0.9 || norm > 1.1 ||
        std::isnan(qx) || std::isnan(qy) || std::isnan(qz) || std::isnan(qw))
    {
        return 0.0;
    }
    tf::Quaternion q(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw; // ENU yaw
}

void handle_msg_manual_control(uavlink_message_t message)
{
	uavlink_msg_manual_control manual_msg;
	uavlink_manual_control_decode(&message, &manual_msg);

	//Convert to fit MAVROS manual control
	manual_control_msg.x = manual_msg.x;
	manual_control_msg.y = manual_msg.y;
	manual_control_msg.z = manual_msg.z;
	manual_control_msg.r = manual_msg.r;

	manual_control_pub.publish(manual_control_msg);
}

/* ******************Offboard MODE********************** */
// Function position control message
void handle_msg_position_control(uavlink_message_t message)
{
    uavlink_position_control_t position_msg;
    uavlink_position_control_decode(&message, &position_msg);

	//Update Timer
	last_position_cmd_time = ros::Time::now();
    
    // ROS_INFO("Position control: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, frame=%d",
    //          position_msg.x, position_msg.y, position_msg.z, position_msg.yaw, position_msg.frame);
    
    if (!position_control_active) {
        // ROS_WARN("Position control not active. Enable position control mode first.");
        send_position_feedback(false, 0, 0, 0);
        return;
    }
    
    // Prepare message for ROS
    position_cmd_msg.header.stamp = ros::Time::now();
    position_cmd_msg.header.frame_id = "map";
    
    if (position_msg.frame == 0) { // Local frame
        position_cmd_msg.pose.position.x = position_msg.x;
        position_cmd_msg.pose.position.y = position_msg.y;
        position_cmd_msg.pose.position.z = position_msg.z;
    } else {
        // Global frame (GPS)
        // For Clover drone, we'll use local frame primarily
        ROS_WARN("Global frame position control not implemented. Using local frame.");
        position_cmd_msg.pose.position.x = position_msg.x;
        position_cmd_msg.pose.position.y = position_msg.y;
        position_cmd_msg.pose.position.z = position_msg.z;
    }

	double yaw_cmd = std::isfinite(position_msg.yaw) ? position_msg.yaw : get_current_yaw();
    
	// Use tf2 to create quaternion from yaw (keep roll/pitch = 0)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_cmd); // roll, pitch, yaw
    position_cmd_msg.pose.orientation = tf2::toMsg(q);
    
    // Publish message
    position_control_pub.publish(position_cmd_msg);
    last_position_cmd_time = ros::Time::now();
    
    // send feedback success
    send_position_feedback(true, 0, 0, 0);
}
// Function on/off position control Offboard
void handle_cmd_position_control_mode(bool enable)
{
    if (enable) {
        // Set mode to OFFBOARD to receive position commands
        handle_cmd_set_mode(3); // 3 = OFFBOARD
        position_control_active = true;
        ROS_INFO("Offboard control mode enabled");
    } else {
        // Return manual or posctl
        // handle_cmd_set_mode(0);
        position_control_active = false;
        ROS_INFO("Disabled position control mode");
    }
}
// Function feedback to position
void send_position_feedback(bool success, float error_x, float error_y, float error_z)
{
    uavlink_position_feedback_t feedback;
    feedback.success = success ? 1 : 0;
    feedback.error_x = error_x;
    feedback.error_y = error_y;
    feedback.error_z = error_z;
    
    uavlink_message_t msg;
    uavlink_position_feedback_encode(&msg, &feedback);
    
    char buf[300];
    uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
    writeSocketMessage(buf, len);
}

// Function velocity control message
void handle_msg_velocity_control(uavlink_message_t message)
{
    uavlink_velocity_control_t vc;
    uavlink_velocity_control_decode(&message, &vc);

	// ROS_INFO("[VELOCITY] vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f frame=%d",
    //          vc.vx, vc.vy, vc.vz, vc.yaw_rate, vc.frame);

    geometry_msgs::TwistStamped ts;
    ts.header.stamp = ros::Time::now();

    // ENU local frame ("map")
    // body frame (frame==1), transform body -> ENU using current yaw
    double vx_enu = vc.vx;
    double vy_enu = vc.vy;
    double vz_enu = vc.vz;

    if (vc.frame == 1) {
        double yaw = get_current_yaw();
        double c = std::cos(yaw), s = std::sin(yaw);
        // Body(X,Y) -> ENU(X,Y)
        double x_e = c * vc.vx - s * vc.vy;
        double y_e = s * vc.vx + c * vc.vy;
        vx_enu = x_e;
        vy_enu = y_e;
        // z no change (body Z // ENU Z, with small roll/pitch)
    }

    ts.header.frame_id = "map"; // publish to ENU local frame

	// Apply Vx override if enabled (for training)
	if (vx_override_enable) 
	{
		vx_enu = vx_override_value;
	}
    ts.twist.linear.x  = vx_enu;
    ts.twist.linear.y  = vy_enu;

	// hold altitude if alt_hold_enable=true
    // ts.twist.linear.z  = vz_enu;
	ts.twist.linear.z  = alt_hold_enable ? compute_vz_hold() : vz_enu;

    ts.twist.angular.x = 0.0;
    ts.twist.angular.y = 0.0;
    ts.twist.angular.z = vc.yaw_rate; // yaw rate (rad/s)

    if (vel_cmd_pub_stamped) vel_cmd_pub_stamped.publish(ts);

    geometry_msgs::Twist t = ts.twist;
    if (vel_cmd_pub_unstamped) vel_cmd_pub_unstamped.publish(t);
}

/* ************************* Function Servo Control ***********************************/

static inline void uavlink_servo_channels_decode(const uavlink_message_t *msg, uavlink_servo_channels_t *sc)
{
    // memset(sc, 0, UAVLINK_MSG_ID_SERVO_CONTROL_LEN);
    // memcpy(sc, _MAV_PAYLOAD(msg), UAVLINK_MSG_ID_SERVO_CONTROL_LEN);

	if (!msg || !sc) return;
    uint8_t len = msg->len < UAVLINK_MSG_ID_SERVO_CONTROL_LEN ? msg->len : UAVLINK_MSG_ID_SERVO_CONTROL_LEN;
    memset(sc, 0, UAVLINK_MSG_ID_SERVO_CONTROL_LEN);
    memcpy(sc, _MAV_PAYLOAD(msg), len);
}

void handle_msg_servo_channels(uavlink_message_t message)
{
    uavlink_servo_channels_t sc;
    uavlink_servo_channels_decode(&message, &sc);

    if (!set_angle_cli_.exists()) {
        set_angle_cli_.waitForExistence(ros::Duration(0.5));
    }

	// Send values to servo controller
    const float vals[5] = { sc.ch0, sc.ch1, sc.ch2, sc.ch3, sc.ch4 };
    for (uint8_t ch = 0; ch < 5; ++ch)
    {
        wicom_roboarm::SetAngle srv;
        srv.request.channel   = ch;
        srv.request.angle_deg = vals[ch];

		// ROS_INFO("Setting servo ch=%u to %.1f deg", ch, vals[ch]);

        if (set_angle_cli_.call(srv)) {
            if (srv.response.success) {
                // ROS_INFO("Servo ch=%u -> %.1f deg OK: %s", ch, vals[ch], srv.response.message.c_str());
            } else {
                // ROS_WARN("Servo ch=%u -> %.1f deg FAILED: %s", ch, vals[ch], srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("Failed calling /pca9685_servo/set_angle (ch=%u, deg=%.1f)", ch, vals[ch]);
        }
    }
}

/* ************************* Function RC Control ***********************************/
// decode implementation for RC channels (placed in .cpp so uavlink types/macros are available)
static inline void uavlink_rc_channels_decode(const uavlink_message_t *msg, uavlink_rc_channels_t *rc)
{
    if (!msg || !rc) return;
    uint8_t len = msg->len < UAVLINK_MSG_ID_RC_CHANNELS_LEN ? msg->len : UAVLINK_MSG_ID_RC_CHANNELS_LEN;
    memset(rc, 0, UAVLINK_MSG_ID_RC_CHANNELS_LEN);
    memcpy(rc, _MAV_PAYLOAD(msg), len);
}
// Handler cho RC channels (uavlink -> mavros OverrideRCIn)
void handle_msg_rc_channels(uavlink_message_t message)
{
    uavlink_rc_channels_t rc;
    uavlink_rc_channels_decode(&message, &rc);

    // mavros_msgs::OverrideRCIn uses fixed-size boost::array<unsigned short, 18>
    rc_msg.channels[0] = rc.chan1;
    rc_msg.channels[1] = rc.chan2;
    rc_msg.channels[2] = rc.chan3;
    rc_msg.channels[3] = rc.chan4;
    rc_msg.channels[4] = rc.chan5;
    rc_msg.channels[5] = rc.chan6;
    rc_msg.channels[6] = rc.chan7;
    rc_msg.channels[7] = rc.chan8;
    // zero remaining channels
    for (size_t i = 8; i < rc_msg.channels.size(); ++i)
    {
        rc_msg.channels[i] = 0;
    }

    // publish to MAVROS
    rc_override_pub.publish(rc_msg);

    // ROS_INFO("[RC] Override published ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u ch7=%u ch8=%u",
    //          (unsigned)rc_msg.channels[0], (unsigned)rc_msg.channels[1],
    //          (unsigned)rc_msg.channels[2], (unsigned)rc_msg.channels[3],
    //          (unsigned)rc_msg.channels[4], (unsigned)rc_msg.channels[5],
    //          (unsigned)rc_msg.channels[6], (unsigned)rc_msg.channels[7]);
}

/* ************************* Function Drone Status ***********************************/
// Function send drone status
void send_drone_status()
{
	uavlink_drone_status_t status;
    status.altitude = local_z;
    status.battery = battery_remaining_calculate(battery_msg.voltage);
    status.latitude = global_msg.latitude;
    status.longitude = global_msg.longitude;

    status.pos_x = local_x;
    status.pos_y = local_y;
    status.pos_z = local_z;

	status.vx = velocity_msg.twist.linear.x;
	status.vy = velocity_msg.twist.linear.y;
	status.vz = velocity_msg.twist.linear.z;

	// Get roll, pitch, yaw from imu_msg
    tf::Quaternion q(
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z,
        imu_msg.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    status.roll = roll;
    status.pitch = pitch;
    status.yaw = yaw;

    status.dist_short = std::isfinite(g_vl53_short_m) ? g_vl53_short_m : -1.0f;
    status.dist_long  = std::isfinite(g_vl53_long_m)  ? g_vl53_long_m  : -1.0f;

    uavlink_message_t msg;
    uavlink_drone_status_encode(&msg, &status);

    char buf[128];
    uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
    writeSocketMessage(buf, len);

    // Debug log
    // ROS_INFO("[DEBUG] Drone status sent: Alt=%.2f, Bat=%d%%, Lat=%.7f, Lon=%.7f, X=%.2f, Y=%.2f, Z=%.2f, Vx=%.2f, Vy=%.2f, Vz=%.2f, Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
    //     status.altitude, status.battery, status.latitude, status.longitude,
    //     status.pos_x, status.pos_y, status.pos_z,
    //     status.vx, status.vy, status.vz,
    //     status.roll, status.pitch, status.yaw);
}

//
void handleLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	local_x = msg->pose.position.x;  // store x value
	local_y = msg->pose.position.y;  // store y value
    local_z = msg->pose.position.z;  // store z value
}

void drone_status_timer_cb(const ros::TimerEvent&)
{
    send_drone_status();
}

// Velocity
void handleVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    velocity_msg = *msg;
}

// Orientation
void handleImu(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_msg = *msg;
}

// Distance
void handleRangeShort(const sensor_msgs::Range::ConstPtr& msg)
{
    // msg->range m (float)
    g_vl53_short_m = msg->range;
}

void handleRangeLong(const sensor_msgs::Range::ConstPtr& msg)
{
    g_vl53_long_m = msg->range;
}
/*************************************************************************************************/

// Handle state from UAV
void handleState(const mavros_msgs::State &s)
{
	state = s;
	uavlink_state_t send_state;
	send_state.armed = s.armed;
	send_state.connected = s.connected;
	send_state.mode = mode_to_int(s.mode);
	send_state.battery_remaining = battery_remaining_calculate(battery_msg.voltage);

	uavlink_message_t msg;
	uavlink_state_encode(&msg, &send_state);

	char buf[300];
	uint16_t len = uavlink_msg_to_send_buffer((uint8_t *)buf, &msg);
	writeSocketMessage(buf, len);
}

// Handle global Posotion from UAV
void handleGlobalPosition(const sensor_msgs::NavSatFix &n)
{
	global_msg = n;
}

// Handle battery state from UAV
void handle_Battery_State(const sensor_msgs::BatteryState &bat)
{
	battery_msg = bat;
}

void init()
{
	// Thread for UDP soket read
	std::thread readThread(&readingSocketThread);
	readThread.detach();
}

int createSocket(int port)
{
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
	sin.sin_port = htons(port);

	if (bind(sockfd, (sockaddr *)&sin, sizeof(sin)) < 0)
	{
		ROS_FATAL("socket bind error: %s", strerror(errno));
		close(sockfd);
		ros::shutdown();
	}

	return sockfd;
}

void readingSocketThread()
{
	char buff[1024];

	// Socket create
	sockfd = createSocket(port);
	memset(&android_addr, 0, sizeof(android_addr));
	ROS_INFO("UDP UdpSocket initialized on port %d", port);

	while (true)
	{
		// read next UDP packet
		int bsize = recvfrom(sockfd, (char *)buff, 1024, 0, (sockaddr *)&android_addr, &android_addr_size);

		buff[bsize] = '\0';
		if (bsize < 0)
		{
			ROS_ERROR("recvfrom() error: %s", strerror(errno));
		}
		else
		{
			if (!check_receiver)
				check_receiver = true;
			uavlink_message_t message;
			memcpy(&message, buff, bsize);
			switch (message.msgid)
			{
			case UAVLINK_MSG_ID_MANUAL_CONTROL:
				handle_msg_manual_control(message);
				break;

			case UAVLINK_MSG_ID_POSITION_CONTROL:  // Position in Offboard Mode
        		handle_msg_position_control(message);
        		break;

			case UAVLINK_MSG_ID_SERVO_CONTROL:
				handle_msg_servo_channels(message);
				break;

			case UAVLINK_MSG_ID_RC_CHANNELS: // message RC
                handle_msg_rc_channels(message);
                break;

			case UAVLINK_MSG_ID_COMMAND:
				handle_command(message);
				break;

			case UAVLINK_MSG_ID_VELOCITY_CONTROL:
				handle_msg_velocity_control(message);
				break;

			default:
				ROS_WARN("Unknown message ID: %d", message.msgid);
				break;
			}
		}
	}
}

void writeSocketMessage(char buff[], int length)
{
	if (check_receiver) // Need received first
	{
		int len = sendto(sockfd, (const char *)buff, length, 0, (const struct sockaddr *)&android_addr, android_addr_size);
	}
}

// safety check for position active in Offboard
void check_position_cmd_timeout(const ros::TimerEvent& e)
{
    if (position_control_active && (ros::Time::now() - last_position_cmd_time > position_cmd_timeout)) {
        ROS_WARN("Position command timeout. Disabling position control.");
        handle_cmd_position_control_mode(false);
        
        // sned feedback to timeout
        send_position_feedback(false, 0, 0, 0);
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "UdpSocket");
	ros::NodeHandle nh, nh_priv("~");
	//servo control
	initServoBridge(nh);

	// param
	nh_priv.param("port", port, 12345);

	// altitude hold param
	nh_priv.param("alt_hold/enable",   alt_hold_enable, true);
	nh_priv.param("alt_hold/target_z", alt_target_z,    0.5);
	nh_priv.param("alt_hold/kp",       kp_z,            1.0);
	nh_priv.param("alt_hold/kd",       kd_z,            0.6);
	nh_priv.param("alt_hold/max_vz",   max_z_vel,       0.5);
	nh_priv.param("alt_hold/invert_z", invert_z_sign,   false);
	// ROS_INFO("[UdpServer] AltHold: en=%s z=%.2f kp=%.2f kd=%.2f max_vz=%.2f invz=%s",
	// 		alt_hold_enable ? "true":"false", alt_target_z, kp_z, kd_z, max_z_vel, invert_z_sign ? "true":"false");

	// vx override params (private ns: ~vx_override/...)
	nh_priv.param("vx_override/enable", vx_override_enable, false);
	nh_priv.param("vx_override/value",  vx_override_value,  0.05);
	ROS_INFO("[UdpServer] vx_override: en=%s vx=%.3f",
			vx_override_enable ? "true":"false", vx_override_value);

	// dataset logger toggle service
	dataset_toggle_srv = nh.serviceClient<std_srvs::SetBool>("/dataset_logger/toggle");

	// Initial publisher
	manual_control_pub = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);
	rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1); // register publisher RC override

	// position pub in offboard
	position_control_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
	// velocity pub in offboard
	vel_cmd_pub_stamped   = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
	vel_cmd_pub_unstamped = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

	// State
	auto state_sub = nh.subscribe("mavros/state", 1, &handleState);
	// Global position
	auto global_position_sub = nh.subscribe("/mavros/global_position/global", 1, &handleGlobalPosition);
	// Battery state
	auto battery_sub = nh.subscribe("/mavros/battery", 1, &handle_Battery_State);
	// Altitude
	ros::Subscriber local_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, handleLocalPose);
	// Velocity
	auto velocity_sub = nh.subscribe("/mavros/local_position/velocity_local", 1, handleVelocity);
	// Orientation
    auto imu_sub = nh.subscribe("/mavros/imu/data", 1, handleImu);
	// Distance sensor
    auto vl53_short_sub = nh.subscribe("/vl53/short_range", 1, handleRangeShort);
    auto vl53_long_sub  = nh.subscribe("/vl53/long_range",  1, handleRangeLong);

	// Service client
	set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	takeoff_srv = nh.serviceClient<wicom_udp::Takeoff>("wicom_udp/takeoff");
	land_srv = nh.serviceClient<std_srvs::Trigger>("wicom_udp/land");

	// Timer
	state_timeout = ros::Duration(nh_priv.param("state_timeout", 3.0));
	arming_timeout = ros::Duration(nh_priv.param("arming_timeout", 4.0));
	// POSITION TIMER In offboard
	ros::Timer position_timeout_timer = nh.createTimer(ros::Duration(0.1), check_position_cmd_timeout);
	// Timer send drone status
	ros::Timer drone_status_timer = nh.createTimer(ros::Duration(0.5), drone_status_timer_cb);
	
	init();
	ros::spin();
}

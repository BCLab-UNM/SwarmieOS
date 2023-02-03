#define BOOST_BIND_NO_PLACEHOLDERS

#include <cmath>
#include "rclcpp/rclcpp.hpp"

#include <boost/thread.hpp>

//ROS libraries
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include <dynamic_reconfigure/server.h>
//#include <dynamic_reconfigure/client.h>

//ROS messages
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/u_int8.hpp>
//#include <std_srvs/srv/empty.hpp>

// Project messages
#include "swarmie_msgs/msg/pid_state.hpp"
//#include "swarmie_msgs/msg/pid_config.hpp"
#include "swarmie_msgs/msg/swarmie_imu.hpp"

//Package include
#include <usbSerial.h>

#include "pid.h"
using std::placeholders::_1;

using namespace std;


class abridge : public rclcpp::Node {
public:
    abridge(): Node("abridge"){
        string devicePath = "/dev/ttyACM0"; //Hardcoded for now works fine as there is no GPS, @TODO use the param
        //rclcpp::param::param("~device", devicePath, string("/dev/ttyUSB0"));
        usb.openUSBPort(devicePath, baud);

        fingerAnglePublish = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("fingerAngle/prev_cmd", 10);
        wristAnglePublish = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("wristAngle/prev_cmd", 10);
        imuRawPublish = this->create_publisher<swarmie_msgs::msg::SwarmieIMU>("imu/raw", 10);
        odomPublish = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        sonarLeftPublish = this->create_publisher<sensor_msgs::msg::Range>("sonarLeft", 10);
        sonarCenterPublish = this->create_publisher<sensor_msgs::msg::Range>("sonarCenter", 10);
        sonarRightPublish = this->create_publisher<sensor_msgs::msg::Range>("sonarRight", 10);
        infoLogPublisher = this->create_publisher<std_msgs::msg::String>("/infoLog", 1); //@TODO fix qos to be latching if needed
        debugPIDPublisher = this->create_publisher<swarmie_msgs::msg::PidState>("bridge/debugPID", 1); //@TODO fix qos to be latching if needed
        heartbeatPublisher = this->create_publisher<std_msgs::msg::String>("bridge/heartbeat", 1);

        driveControlSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("driveControl", 10, std::bind(&abridge::driveCommandHandler, this, _1));
        fingerAngleSubscriber = this->create_subscription<std_msgs::msg::Float32>("fingerAngle/cmd", 1, std::bind(&abridge::fingerAngleHandler, this, _1));
        wristAngleSubscriber = this->create_subscription<std_msgs::msg::Float32>("wristAngle/cmd", 1, std::bind(&abridge::wristAngleHandler, this, _1));
        modeSubscriber = this->create_subscription<std_msgs::msg::UInt8>("mode", 1, std::bind(&abridge::modeHandler, this, _1));

        serialActivityTimerTimer = this->create_wall_timer(deltaTime,
                                                                std::bind(&abridge::serialActivityTimer,
                                                                          this));
        publishHeartBeatTimerEventHandlerTimer = this->create_wall_timer(heartbeat_publish_interval,
                                                                                          std::bind(
                                                                                                  &abridge::publishHeartBeatTimerEventHandler,
                                                                                                  this));
        this->declare_parameter("odom_frame", "odom");
        /*
        rclcpp::param::param<std_msgs::msg::string>("odom_frame", odom.header.frame_id, "odom");
        rclcpp::param::param<std_msgs::msg::string>("base_link_frame", odom.child_frame_id, "base_link");
        rclcpp::param::param<std_msgs::msg::string>("sonar_left_frame", sonarLeft.header.frame_id, "us_left_link");
        rclcpp::param::param<std_msgs::msg::string>("sonar_center_frame", sonarCenter.header.frame_id, "us_center_link");
        rclcpp::param::param<std_msgs::msg::string>("sonar_right_frame", sonarRight.header.frame_id, "us_right_link");
        */
        imuRaw.header.frame_id = odom.child_frame_id;
    }

private:
    geometry_msgs::msg::QuaternionStamped fingerAngle;
    geometry_msgs::msg::QuaternionStamped wristAngle;
    swarmie_msgs::msg::SwarmieIMU imuRaw;
    nav_msgs::msg::Odometry odom;
    double odomTheta = 0;
    sensor_msgs::msg::Range sonarLeft;
    sensor_msgs::msg::Range sonarCenter;
    sensor_msgs::msg::Range sonarRight;
    USBSerial usb;
    const int baud = 115200;
    char dataCmd[3] = "d\n";
    char moveCmd[16];
    char host[128];
    chrono::duration<int_least64_t, milli> deltaTime = 100ms; //abridge's update interval
    chrono::duration<int_least64_t, milli> heartbeat_publish_interval = 2s;
    rclcpp::TimerBase::SharedPtr serialActivityTimerTimer;
    rclcpp::TimerBase::SharedPtr publishHeartBeatTimerEventHandlerTimer;
    int currentMode = 0;
    geometry_msgs::msg::Twist speedCommand;
    void driveCommandHandler(const geometry_msgs::msg::Twist::SharedPtr message);
    void fingerAngleHandler(const std_msgs::msg::Float32::SharedPtr angle);
    void wristAngleHandler(const std_msgs::msg::Float32::SharedPtr angle);
    void serialActivityTimer();
    void modeHandler(const std_msgs::msg::UInt8::SharedPtr message);
    void publishRosTopics();
    void parseData(string str);
    //void initialconfig();

    double leftTicksToMeters(double leftTicks) const;
    double rightTicksToMeters(double rightTicks) const;
    double metersToTicks(double meters) const;
    double leftMetersToTicks(double meters) const;
    double rightMetersToTicks(double meters) const;
    double diffToTheta(double right, double left) const;
    double thetaToDiff(double theta) const;

    //Callback handlers
    void publishHeartBeatTimerEventHandler();
    //void reconfigure(bridge::pidConfig &cfg, uint32_t level);

    // Allowing messages to be sent to the arduino too fast causes a disconnect
    // This is the minimum time between messages to the arduino in microseconds.
    // Only used with the gripper commands to fix a manual control bug.
    unsigned int min_usb_send_delay = 100;

    const double wheelBase = 0.278; //distance between left and right wheels (in M)
    const double leftWheelCircumference = 0.3651; // avg for 3 rovers (in M)
    const double rightWheelCircumference = 0.3662; // avg for 3 rovers (in M)
    const int cpr = 8400; //"cycles per revolution" -- number of encoder increments per one wheel revolution

    // running counts of encoder ticks
    double leftTicks = 0;
    double rightTicks = 0;
    // wheel velocities in ticks/sec
    double leftTickVel = 0;
    double rightTickVel = 0;
    double odomTS = 0;

    // Immobilize robot until the first PID configuration.
    PID left_pid = PID(0, 0, 0, 0, 120, -120, 0, -1);
    PID right_pid = PID(0, 0, 0, 0, 120, -120, 0, -1);

    //Publishers
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr fingerAnglePublish;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr wristAnglePublish;
    rclcpp::Publisher<swarmie_msgs::msg::SwarmieIMU>::SharedPtr imuRawPublish;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublish;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonarLeftPublish;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonarCenterPublish;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonarRightPublish;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr infoLogPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPublisher;
    rclcpp::Publisher<swarmie_msgs::msg::PidState>::SharedPtr debugPIDPublisher;

    //rclcpp::Publisher debugPIDPublisher;
    swarmie_msgs::msg::PidState pid_state;

    //Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr driveControlSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  fingerAngleSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  wristAngleSubscriber;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  modeSubscriber;

    //Timers
    //rclcpp::Timer publishTimer;
    //rclcpp::Timer publish_heartbeat_timer;

    // Feed-forward constants
    double ff_l;
    double ff_r;

    //void publishRosTopics();
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<abridge>());
    rclcpp::shutdown();
    return 0;
} //end main

void abridge::driveCommandHandler(const geometry_msgs::msg::Twist::SharedPtr message) {
  speedCommand.linear.x = message->linear.x;
  speedCommand.angular.z = message->angular.z;
  speedCommand.angular.y = message->angular.y;
}

// The finger and wrist handlers receive gripper angle commands in floating point
// radians, write them to a string and send that to the arduino
// for processing.
void abridge::fingerAngleHandler(const std_msgs::msg::Float32::SharedPtr angle) {

  // To throttle the message rate so we don't lose connection to the arduino
  usleep(min_usb_send_delay);
  
  char cmd[16]={'\0'};

  // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01) {
    // 'f' indicates this is a finger command to the arduino
    sprintf(cmd, "f,0\n");
  } else {
    sprintf(cmd, "f,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}

void abridge::wristAngleHandler(const std_msgs::msg::Float32::SharedPtr angle) {
  // To throttle the message rate so we don't lose connection to the arduino
  usleep(min_usb_send_delay);
  
    char cmd[16]={'\0'};

    // Avoid dealing with negative exponents which confuse the conversion to string by checking if the angle is small
  if (angle->data < 0.01) {
    // 'w' indicates this is a wrist command to the arduino
    sprintf(cmd, "w,0\n");
  } else {
    sprintf(cmd, "w,%.4g\n", angle->data);
  }
  usb.sendData(cmd);
  memset(&cmd, '\0', sizeof (cmd));
}


double abridge::leftTicksToMeters(double leftTicks_arg) const {
	return (leftWheelCircumference * leftTicks_arg) / cpr;
}

double abridge::rightTicksToMeters(double rightTicks_arg) const {
	return (rightWheelCircumference * rightTicks_arg) / cpr;
}

double abridge::metersToTicks(double meters) const {
    return (meters * cpr) / ((leftWheelCircumference + rightWheelCircumference) / 2);
}

double abridge::leftMetersToTicks(double meters) const {
    return (meters * cpr) / leftWheelCircumference;
}

double abridge::rightMetersToTicks(double meters) const {
    return (meters * cpr) / rightWheelCircumference;
}

double abridge::diffToTheta(double right, double left) const {
	return (right - left) / wheelBase;
}

double abridge::thetaToDiff(double theta) const {
	return theta * wheelBase;
}

void abridge::serialActivityTimer() {

	int cmd_mode = round(speedCommand.angular.y);
	if (cmd_mode == DRIVE_MODE_STOP) {
		left_pid.reset();
		right_pid.reset();

		sprintf(moveCmd, "s\n");
		usb.sendData(moveCmd);
		memset(&moveCmd, '\0', sizeof (moveCmd));
	}
	else {
		// Calculate tick-wise velocities.
		double linear_sp = metersToTicks(speedCommand.linear.x);
		double angular_sp = metersToTicks(thetaToDiff(speedCommand.angular.z));

        double left_sp = leftMetersToTicks(speedCommand.linear.x) - angular_sp;
        double right_sp = rightMetersToTicks(speedCommand.linear.x) + angular_sp;

		int l = round(left_pid.step(left_sp, leftTickVel, odomTS));
		int r = round(right_pid.step(right_sp, rightTickVel, odomTS));

		// Feed forward
		l += ff_l * left_sp;
		r += ff_r * right_sp;

		// Debugging: Report PID performance for tuning.
		// Output of the PID is in Linear:
		pid_state.header.stamp = this->get_clock()->now();
		pid_state.left_wheel.output = l; // sp_linear * ff
		pid_state.right_wheel.output = r;
		pid_state.left_wheel.error = left_sp - leftTickVel; // sp - feedback
		pid_state.right_wheel.error = right_sp - rightTickVel; // sp - feedback
		pid_state.left_wheel.p_term = left_pid.getP();
		pid_state.right_wheel.p_term = right_pid.getP();
		pid_state.left_wheel.i_term = left_pid.getI();
		pid_state.right_wheel.i_term = right_pid.getI();
		pid_state.left_wheel.d_term = left_pid.getD();
		pid_state.right_wheel.d_term = right_pid.getD();
		pid_state.linear_setpoint = linear_sp;
		pid_state.left_wheel.setpoint = left_sp;
		pid_state.right_wheel.setpoint = right_sp;

		// Feedback function is in Angular:
		pid_state.angular_setpoint = angular_sp;
		pid_state.left_wheel.feedback = leftTickVel;
		pid_state.right_wheel.feedback = rightTickVel;
		debugPIDPublisher->publish(pid_state);

		sprintf(moveCmd, "v,%d,%d\n", l, r); //format data for arduino into c string
		usb.sendData(moveCmd);                      //send movement command to arduino over usb
		memset(&moveCmd, '\0', sizeof (moveCmd));   //clear the movement command string
	}

	usb.sendData(dataCmd);
	try {
		parseData(usb.readData());
		publishRosTopics();
	} catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Exception while parsing Arduino data. Probably IMU.");
	}
}

void abridge::publishRosTopics() {
	/*
    fingerAnglePublish->publish(fingerAngle);
    wristAnglePublish->publish(wristAngle);
    imuRawPublish->publish(imuRaw);
    odomPublish->publish(odom);
    sonarLeftPublish->publish(sonarLeft);
    sonarCenterPublish->publish(sonarCenter);
    sonarRightPublish->publish(sonarRight);
    */
}

void abridge::parseData(string str) {
    istringstream oss(str);
    string sentence;
    static double lastOdomTS = 0;

    while (getline(oss, sentence, '\n')) {
		istringstream wss(sentence);
		string word;

		vector<string> dataSet;
		while (getline(wss, word, ',')) {
			dataSet.push_back(word);
		}

		if (dataSet.size() >= 3 && dataSet.at(1) == "1") {

			if (dataSet.at(0) == "GRF") {
				fingerAngle.header.stamp = this->get_clock()->now();
                tf2::Quaternion quat_tf;
                quat_tf.setRPY(strtod(dataSet.at(2).c_str(), nullptr), 0.0, 0.0);
                fingerAngle.quaternion =  tf2::toMsg(quat_tf);
            }
			else if (dataSet.at(0) == "GRW") {
				wristAngle.header.stamp = this->get_clock()->now();
                tf2::Quaternion quat_tf;
                quat_tf.setRPY(strtod(dataSet.at(2).c_str(), nullptr), 0.0, 0.0);
                wristAngle.quaternion =  tf2::toMsg(quat_tf);
			}
			else if (dataSet.at(0) == "IMU") {
				imuRaw.header.stamp = this->get_clock()->now();
				imuRaw.accelerometer.x = strtod(dataSet.at(2).c_str(), nullptr);
				imuRaw.accelerometer.y  = strtod(dataSet.at(3).c_str(), nullptr);
				imuRaw.accelerometer.z = strtod(dataSet.at(4).c_str(), nullptr);
				imuRaw.magnetometer.x = strtod(dataSet.at(5).c_str(), nullptr);
				imuRaw.magnetometer.y = strtod(dataSet.at(6).c_str(), nullptr);
				imuRaw.magnetometer.z = strtod(dataSet.at(7).c_str(), nullptr);
				imuRaw.angular_velocity.x = strtod(dataSet.at(8).c_str(), nullptr);
				imuRaw.angular_velocity.y = strtod(dataSet.at(9).c_str(), nullptr);
				imuRaw.angular_velocity.z = strtod(dataSet.at(10).c_str(), nullptr);

			    imuRawPublish->publish(imuRaw);
			}
			else if (dataSet.at(0) == "ODOM") {
				leftTicks = strtol(dataSet.at(2).c_str(), nullptr, 10);
				rightTicks = strtol(dataSet.at(3).c_str(), nullptr, 10);
				odomTS = strtod(dataSet.at(4).c_str(), nullptr) / 1000; // Seconds

                double rightWheelDistance = rightTicksToMeters(rightTicks);

                double leftWheelDistance = leftTicksToMeters(leftTicks);

			    //Calculate relative angle that robot has turned
				double dtheta = diffToTheta(rightWheelDistance, leftWheelDistance);

			    //Accumulate angles to calculate absolute heading
			    odomTheta += dtheta;

			    //Decompose linear distance into its component values
			    double meanWheelDistance = (rightWheelDistance + leftWheelDistance) / 2;
                //Twist is in base_link frame, use relative heading
			    double twistX = meanWheelDistance * cos(dtheta);
			    double twistY = meanWheelDistance * sin(dtheta);
                //Pose is in the odom frame, use absolute heading
                double poseX = meanWheelDistance * cos(odomTheta);
                double poseY = meanWheelDistance * sin(odomTheta);


                // Calculate velocities if possible.
			    double vtheta = 0;
			    double vx = 0;
			    double vy = 0;
			    if (lastOdomTS > 0) {
			    	double deltaT = odomTS - lastOdomTS;
			    	vtheta = dtheta / deltaT;
			    	vx = twistX / deltaT;
			    	vy = twistY / deltaT;

			    	// Normalize ticks to ticks/s
			    	leftTickVel = leftTicks / deltaT;
			    	rightTickVel = rightTicks / deltaT;
			    }
		    	lastOdomTS = odomTS;

				odom.header.stamp = this->get_clock()->now();
				odom.pose.pose.position.x += poseX;
				odom.pose.pose.position.y += poseY;
				odom.pose.pose.position.z = 0;

                tf2::Quaternion quat_tf;
                quat_tf.setRPY(0,0, odomTheta);
				odom.pose.pose.orientation = tf2::toMsg(quat_tf);

				odom.twist.twist.linear.x = vx;
				odom.twist.twist.linear.y = vy;
				odom.twist.twist.angular.z = vtheta;

			    odomPublish->publish(odom);
			}
			else if (dataSet.at(0) == "USL") {
				sonarLeft.header.stamp = this->get_clock()->now();
				//From https://www.pololu.com/product/1605
				sonarLeft.max_range = 3; //in meters Test Value @TODO tune
				sonarLeft.field_of_view = 0.698132; // in radians @TODO tune
				sonarLeft.range = strtod(dataSet.at(2).c_str(), nullptr) / 100.0;
			    sonarLeftPublish->publish(sonarLeft);
			}
			else if (dataSet.at(0) == "USC") {
				sonarCenter.header.stamp = this->get_clock()->now();
				sonarCenter.max_range = 3; //in meters Test Value @TODO tune
				sonarCenter.field_of_view = 0.698132; // in radians @TODO tune
				sonarCenter.range = strtod(dataSet.at(2).c_str(), nullptr) / 100.0;
			    sonarCenterPublish->publish(sonarCenter);
			}
			else if (dataSet.at(0) == "USR") {
				sonarRight.header.stamp = this->get_clock()->now();
				sonarRight.max_range = 3; //in meters Test Value @TODO tune
				sonarRight.field_of_view = 0.698132; // in radians @TODO tune
				sonarRight.range = strtod(dataSet.at(2).c_str(), nullptr) / 100.0;
			    sonarRightPublish->publish(sonarRight);
			}

		}
	}
}

void abridge::modeHandler(const std_msgs::msg::UInt8::SharedPtr message) {
	currentMode = message->data;
}

void abridge::publishHeartBeatTimerEventHandler() {
    std_msgs::msg::String msg;
    msg.data = "";
    heartbeatPublisher->publish(msg);
}

/*
void initialconfig() {
    // Set PID parameters from the launch configuration
    //bridge::pidConfig initial_config;
	rclcpp::NodeHandle nh("~");

	nh.getParam("scale", initial_config.scale);
	nh.getParam("Kp", initial_config.Kp);
	nh.getParam("Ki", initial_config.Ki);
	nh.getParam("Kd", initial_config.Kd);
	nh.getParam("db", initial_config.db);
	nh.getParam("st", initial_config.st);
	nh.getParam("wu", initial_config.wu);
	nh.getParam("ff_l", initial_config.ff_l);
	nh.getParam("ff_r", initial_config.ff_r);

	// Announce the configuration to the server
	//dynamic_reconfigure::Client<bridge::pidConfig> dyn_client("abridge");
	//dyn_client.setConfiguration(initial_config);

	cout << "Initial configuration sent." << endl;
}
*/
#include <iostream>
#include <cmath>
#include <ros/ros.h>

#include <boost/thread.hpp>

//ROS libraries
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>

//ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>

// Project messages
#include <bridge/PidState.h>
#include <swarmie_msgs/SwarmieIMU.h>

//Package include
#include <usbSerial.h>

#include "pid.h"
#include <bridge/pidConfig.h>

using namespace std;

//aBridge functions
void driveCommandHandler(const geometry_msgs::Twist::ConstPtr& message);
void serialActivityTimer(const ros::TimerEvent& e);
void parseData(const ros::TimerEvent& e);
void publishRosTopics();
void initialconfig();

//Globals
swarmie_msgs::SwarmieIMU imuRaw;
nav_msgs::Odometry odom;
double odomTheta = 0;
USBSerial usb;
const int baud = 460800;
char moveCmd[16];
char host[128];
const float deltaTime = 0.005; //abridge's update interval
const float indeltaTime = 0.01; //abridge's update interval
int currentMode = 0;
geometry_msgs::Twist speedCommand;

const int MAX_BUFFER_SIZE = 1024; // maximum size of the buffer
char buffer[MAX_BUFFER_SIZE + 1]; // buffer to hold received data
int bufferIndex = 0; // current index into the buffer

// Allowing messages to be sent to the arduino too fast causes a disconnect
// This is the minimum time between messages to the arduino in microseconds.
// Only used with the gripper commands to fix a manual control bug.
unsigned int min_usb_send_delay = 10;

float heartbeat_publish_interval = 2;

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
PID left_pid(0, 0, 0, 0, 120, -120, 0, -1);
PID right_pid(0, 0, 0, 0, 120, -120, 0, -1);

//Publishers
ros::Publisher imuRawPublish;
ros::Publisher odomPublish;
ros::Publisher infoLogPublisher;
ros::Publisher heartbeatPublisher;

ros::Publisher debugPIDPublisher;
bridge::PidState pid_state;

//Subscribers
ros::Subscriber driveControlSubscriber;
ros::Subscriber modeSubscriber;

//Timers
ros::Timer publishTimer;
ros::Timer dataTimer;
ros::Timer publish_heartbeat_timer;

// Feed-forward constants
double ff_l;
double ff_r;

//Callback handlers
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void reconfigure(bridge::pidConfig &cfg, uint32_t level);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);

int main(int argc, char **argv) {

    ros::init(argc, argv, "abridge");

    string devicePath;
    ros::param::param("~device", devicePath, string("/dev/ttyUSB0"));
    usb.openUSBPort(devicePath, baud);
    void modeHandler(const std_msgs::UInt8::ConstPtr& message);

    sleep(5);

    ros::NodeHandle aNH;
    imuRawPublish = aNH.advertise<swarmie_msgs::SwarmieIMU>("imu/raw", 10);
    odomPublish = aNH.advertise<nav_msgs::Odometry>("odom", 10);
    infoLogPublisher = aNH.advertise<std_msgs::String>("/infoLog", 1, true);
    debugPIDPublisher = aNH.advertise<bridge::PidState>("bridge/debugPID", 1, false);
    heartbeatPublisher = aNH.advertise<std_msgs::String>("bridge/heartbeat", 1, true);

    driveControlSubscriber = aNH.subscribe("driveControl", 10, driveCommandHandler);
    modeSubscriber = aNH.subscribe("mode", 1, modeHandler);

    dataTimer = aNH.createTimer(ros::Duration(indeltaTime), parseData);

    publishTimer = aNH.createTimer(ros::Duration(deltaTime), serialActivityTimer);
    publish_heartbeat_timer = aNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    ros::param::param<std::string>("odom_frame", odom.header.frame_id, "odom");
    ros::param::param<std::string>("base_link_frame", odom.child_frame_id, "base_link");
    imuRaw.header.frame_id = odom.child_frame_id;

    // configure dynamic reconfiguration
    dynamic_reconfigure::Server<bridge::pidConfig> config_server;
    dynamic_reconfigure::Server<bridge::pidConfig>::CallbackType f;
    f = boost::bind(&reconfigure, _1, _2);
    config_server.setCallback(f);

    boost::thread t(initialconfig);
    ros::spin();

    return EXIT_SUCCESS;
}

void reconfigure(bridge::pidConfig &cfg, uint32_t level) {
    double p, i, d, db, st, wu;
    p = cfg.groups.pid.Kp * cfg.groups.pid.scale;
    i = cfg.groups.pid.Ki * cfg.groups.pid.scale;
    d = cfg.groups.pid.Kd * cfg.groups.pid.scale;
    db = cfg.groups.pid.db;
    st = cfg.groups.pid.st;
    wu = cfg.groups.pid.wu;
    ff_l = cfg.groups.pid.ff_l;
    ff_r = cfg.groups.pid.ff_r;

    left_pid.reconfig(p, i, d, db, st, wu);
    right_pid.reconfig(p, i, d, db, st, wu);

    std_msgs::String msg;
    msg.data = "PID reconfigure is done.";
    infoLogPublisher.publish(msg);
}

void driveCommandHandler(const geometry_msgs::Twist::ConstPtr& message) {
    speedCommand.linear.x = message->linear.x;
    speedCommand.angular.z = message->angular.z;
    speedCommand.angular.y = message->angular.y;
}

double leftTicksToMeters(double leftTicks) {
    return (leftWheelCircumference * leftTicks) / cpr;
}

double rightTicksToMeters(double rightTicks) {
    return (rightWheelCircumference * rightTicks) / cpr;
}

double metersToTicks(double meters) {
    return (meters * cpr) / ((leftWheelCircumference + rightWheelCircumference) / 2);
}

double leftMetersToTicks(double meters) {
    return (meters * cpr) / leftWheelCircumference;
}

double rightMetersToTicks(double meters) {
    return (meters * cpr) / rightWheelCircumference;
}

double diffToTheta(double right, double left) {
    return (right - left) / wheelBase;
}

double thetaToDiff(double theta) {
    return theta * wheelBase;
}

void serialActivityTimer(const ros::TimerEvent& e) {

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
        pid_state.header.stamp = ros::Time::now();
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
        debugPIDPublisher.publish(pid_state);

        sprintf(moveCmd, "v %d %d\n", l, r); //format data for arduino into c string
        usb.sendData(moveCmd);                      //send movement command to arduino over usb
        memset(&moveCmd, '\0', sizeof (moveCmd));   //clear the movement command string
    }
}

void parseData(const ros::TimerEvent& e){
    static double lastOdomTS = 0;
    static std::string buffer;
    std::string data = usb.readData();
    if (!data.empty()) {buffer += data;}

    while (buffer.find('\n') != std::string::npos) {
        std::string line = buffer.substr(0, buffer.find('\n'));
        buffer = buffer.substr(buffer.find('\n') + 1);

        std::istringstream iss(line);
        std::string token;
        vector<string> dataSet;
        while (std::getline(iss, token, ',')) {
            dataSet.push_back(token);
        }
        if (dataSet.size() >= 3 && dataSet.at(1) == "1") {
            if (dataSet.at(0) == "IMU") {
                imuRaw.header.stamp = ros::Time::now();
                imuRaw.accelerometer.x = atof(dataSet.at(2).c_str());
                imuRaw.accelerometer.y  = atof(dataSet.at(3).c_str());
                imuRaw.accelerometer.z = atof(dataSet.at(4).c_str());
                imuRaw.magnetometer.x = atof(dataSet.at(5).c_str());
                imuRaw.magnetometer.y = atof(dataSet.at(6).c_str());
                imuRaw.magnetometer.z = atof(dataSet.at(7).c_str());
                imuRaw.angular_velocity.x = atof(dataSet.at(8).c_str());
                imuRaw.angular_velocity.y = atof(dataSet.at(9).c_str());
                imuRaw.angular_velocity.z = atof(dataSet.at(10).c_str());

                imuRawPublish.publish(imuRaw);
            }
            else if (dataSet.at(0) == "ODOM") {
                leftTicks = atoi(dataSet.at(2).c_str());
                rightTicks = atoi(dataSet.at(3).c_str());
                odomTS = atof(dataSet.at(4).c_str()) / 1000; // Seconds

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

                odom.header.stamp = ros::Time::now();
                odom.pose.pose.position.x += poseX;
                odom.pose.pose.position.y += poseY;
                odom.pose.pose.position.z = 0;
                odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odomTheta);

                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vtheta;

                odomPublish.publish(odom);
            }
        }
    }
} //end parseData function

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}

void initialconfig() {
    // Set PID parameters from the launch configuration
    bridge::pidConfig initial_config;
	ros::NodeHandle nh("~");

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
	dynamic_reconfigure::Client<bridge::pidConfig> dyn_client("abridge");
	dyn_client.setConfiguration(initial_config);

	cout << "Initial configuration sent." << endl;
}

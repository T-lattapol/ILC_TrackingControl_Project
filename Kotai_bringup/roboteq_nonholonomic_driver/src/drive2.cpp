#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))
#define _CMDVEL_DEBUG
//#define _CMDVEL_FORCE_RUN
#define _ODOM_DEBUG
#define _ODOM_SENSORS
//#define _ODOM_COVAR_SERVER
#define NORMALIZE(_z) atan2(sin(_z), cos(_z))
#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#ifdef _ODOM_SENSORS
#include <std_msgs/Float32.h>
#include <roboteq_nonholonomic_msgs/Duplex.h>
#endif
#ifdef _ODOM_COVAR_SERVER
#include "roboteq_nonholonomic_msgs/OdometryCovariances.h"
#include "roboteq_nonholonomic_msgs/RequestOdometryCovariances.h"
#endif

uint32_t millis()
{
	ros::WallTime walltime = ros::WallTime::now();
//	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	return (uint32_t)(walltime.toNSec()/1000000);
}

void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}

float convert_trans_rot_vel_to_steering_angle(float velocity,float omega,float wheelbase)
{
  if(omega == 0 || velocity == 0){

    // return 0 ;
  }
  float radius = velocity / omega;
  // return atan(wheelbase / radius);
  return omega;
}

class MainNode
{

public:
  MainNode();

public:
   void cmdvel_callback( const geometry_msgs::Twist& twist_msg);
   void cmdvel_setup();
  // void cmdvel_loop();
  // void cmdvel_run();
  // void odom_setup();
  // void odom_stream();
  // void odom_loop();
  //void odom_hs_run();
  // void odom_ms_run();
  // void odom_ls_run();
  // void odom_publish();
#ifdef _ODOM_COVAR_SERVER
  void odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res);
#endif

  int run();

protected:
  ros::NodeHandle nh;

  serial::Serial controller;

  uint32_t starttime;
  uint32_t hstimer;
  uint32_t mstimer;
  uint32_t lstimer;

  ros::Subscriber cmdvel_sub;
  geometry_msgs::TransformStamped tf_msg;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  ros::Publisher odom_pub;
#ifdef _ODOM_SENSORS
  std_msgs::Float32 voltage_msg;
  ros::Publisher voltage_pub;
  roboteq_nonholonomic_msgs::Duplex current_msg;
  ros::Publisher current_pub;
  std_msgs::Float32 energy_msg;
  ros::Publisher energy_pub;
  std_msgs::Float32 temperature_msg;
  ros::Publisher temperature_pub;
#endif
  int odom_idx;
  char odom_buf[24];
  char odom_encoder_toss;
  int32_t odom_encoder_left;
  int32_t odom_encoder_right;
  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;
  uint32_t odom_last_time;

#ifdef _ODOM_SENSORS
  float voltage;
  float current_right;
  float current_left;
  float energy;
  float temperature;
  uint32_t current_last_time;
#endif

  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string port;
  int baud;
  bool open_loop;
  double wheel_circumference;
  double wheel_base_width;
  int encoder_velocity_ppr;
  int encoder_steering_ppr;
  int encoder_velocity_cpr;
  int encoder_steering_cpr;
  double max_amps;
  int max_rpm;

};

MainNode::MainNode() : 
  starttime(0),
  hstimer(0),
  mstimer(0),
  odom_idx(0),
  odom_encoder_toss(5),
  odom_encoder_left(0),
  odom_encoder_right(0),
  odom_x(0.0),
  odom_y(0.0),
  odom_yaw(0.0),
  odom_last_x(0.0),
  odom_last_y(0.0),
  odom_last_yaw(0.0),
  odom_last_time(0),
#ifdef _ODOM_SENSORS
  voltage(0.0),
  current_right(0.0),
  current_left(0.0),
  energy(0.0),
  temperature(0.0),
  current_last_time(0),
#endif
  pub_odom_tf(true),
  open_loop(false),
  baud(115200),
  wheel_circumference(0),
  wheel_base_width(0),
  encoder_velocity_ppr(0),
  encoder_steering_ppr(0),
  encoder_velocity_cpr(0),
  encoder_steering_cpr(0),
  max_amps(0.0),
  max_rpm(0)
{

  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("cmdvel_topic", cmdvel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
  nhLocal.param<std::string>("odom_topic", odom_topic, "odom");
  ROS_INFO_STREAM("odom_topic: " << odom_topic);
  nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
  ROS_INFO_STREAM("port: " << port);
  nhLocal.param("baud", baud, 115200);
  ROS_INFO_STREAM("baud: " << baud);
  nhLocal.param("open_loop", open_loop, false);
  ROS_INFO_STREAM("open_loop: " << open_loop);
  nhLocal.param("wheel_circumference", wheel_circumference, 0.3192);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nhLocal.param("wheel_base_width", wheel_base_width, 0.4318);
  ROS_INFO_STREAM("wheel_base_width: " << wheel_base_width);
  nhLocal.param("encoder_velocity_ppr", encoder_velocity_ppr, 360);
  ROS_INFO_STREAM("encoder_velocity_ppr: " << encoder_velocity_ppr);
  nhLocal.param("encoder_steering_ppr", encoder_steering_ppr, 10);
  ROS_INFO_STREAM("encoder_steering_ppr: " << encoder_steering_ppr);
  nhLocal.param("encoder_velocity_cpr", encoder_velocity_cpr, 360);
  ROS_INFO_STREAM("encoder_velocity_cpr: " << encoder_velocity_cpr);
  nhLocal.param("encoder_steering_cpr", encoder_steering_cpr, 200);
  ROS_INFO_STREAM("encoder_steering_cpr: " << encoder_steering_cpr);
  nhLocal.param("max_amps", max_amps, 60.0);
  ROS_INFO_STREAM("max_amps: " << max_amps);
  nhLocal.param("max_rpm", max_rpm, 100);
  ROS_INFO_STREAM("max_rpm: " << max_rpm);

}

void MainNode::cmdvel_callback( const geometry_msgs::Twist& twist_msg)
{

  // wheel speed (m/s)
  float velocity = twist_msg.linear.x;
  float steering = convert_trans_rot_vel_to_steering_angle(velocity,twist_msg.angular.z,wheel_base_width);
 #ifdef _CMDVEL_DEBUG
 ROS_DEBUG_STREAM("cmdvel velocity: " << velocity << " steering: " << steering);
 #endif

  std::stringstream right_cmd;
  std::stringstream left_cmd;

  if (open_loop)
  {
    // motor power (scale 0-1000)
    int32_t right_power = velocity / wheel_circumference * 60.0 / max_rpm * 1000.0;
    int32_t left_power = steering / wheel_circumference * 60.0 / max_rpm * 1000.0;
// #ifdef _CMDVEL_DEBUG
// ROS_DEBUG_STREAM("cmdvel power right: " << right_power << " left: " << left_power);
// #endif
    right_cmd << "!G 1 " << right_power << "\r";
   // left_cmd << "!G 2 " << left_power << "\r";
  }
  else
  {
    // motor speed (rpm)
    int32_t right_rpm = 1000*steering;
    int32_t left_rpm =  100*velocity;
 #ifdef _CMDVEL_DEBUG
 ROS_DEBUG_STREAM("cmdvel rpm right: " << right_rpm << " left: " << left_rpm);
 #endif
    right_cmd << "!S 1 " << left_rpm << "\r";
    left_cmd << "!G 2 " << right_rpm << "\r";

  }

#ifndef _CMDVEL_FORCE_RUN
  controller.write(right_cmd.str());
  controller.write(left_cmd.str());
  controller.flush();
#endif
}


void MainNode::cmdvel_setup()
{
  // stop motors
  controller.write("!G 1 0\r");
  controller.write("!G 2 0\r");
  controller.write("!S 1 0\r");
  controller.write("!S 2 0\r");
  controller.flush();

  // disable echo
  controller.write("^ECHOF 1\r");
  controller.flush();

  // enable watchdog timer (1000 ms)
  controller.write("^RWD 1000\r");

  // set motor operating mode (1 for closed-loop speed)
  if (open_loop)
  {
    // open-loop speed mode
    controller.write("^MMOD 1 0\r");
    controller.write("^MMOD 2 0\r");
  }
  else
  {
    // closed-loop speed mode
    controller.write("^MMOD 1 1\r");
    controller.write("^MMOD 2 4\r");
  }
  // set motor amps limit (A * 10)
  std::stringstream velocity_ampcmd;
  std::stringstream steering_ampcmd;
  velocity_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
  steering_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
  controller.write(velocity_ampcmd.str());
  controller.write(steering_ampcmd.str());

  // set max speed (rpm) for relative speed commands
  // std::stringstream velocity_rpmcmd;
  // std::stringstream steering_rpmcmd;
  // velocity_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
  // steering_rpmcmd << "^MXRPM 2 " << max_rpm << "\r";
  // controller.write(velocity_rpmcmd.str());
  // controller.write(steering_rpmcmd.str());

  // set max acceleration rate (2000 rpm/s * 10)
  // controller.write("^MAC 1 20000\r");
  // controller.write("^MAC 2 20000\r");

  // set max deceleration rate (2000 rpm/s * 10)
  // controller.write("^MDEC 1 20000\r");
  // controller.write("^MDEC 2 20000\r");

  // set PID parameters (gain * 10)
  controller.write("^KP 1 0\r");
  controller.write("^KP 2 50\r");
  controller.write("^KI 1 20\r");
  controller.write("^KI 2 0\r");
  controller.write("^KD 1 0\r");
  controller.write("^KD 2 0\r");

  // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
  controller.write("^EMOD 1 18\r");
  controller.write("^EMOD 2 34\r");

  // set encoder counts (ppr)
  std::stringstream encoder_velocity_enccmd;
  std::stringstream encoder_steering_enccmd;
  encoder_velocity_enccmd << "^EPPR 1 " << encoder_velocity_ppr << "\r";
  encoder_steering_enccmd << "^EPPR 2 " << encoder_steering_ppr << "\r";
  controller.write(encoder_velocity_enccmd.str());
  controller.write(encoder_steering_enccmd.str());

  controller.flush();

  ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
   cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &MainNode::cmdvel_callback, this);

}

int MainNode::run()
{
	ROS_INFO("Beginning setup...");
	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	controller.setPort(port);
	controller.setBaudrate(baud);
	controller.setTimeout(timeout);
	// TODO: support automatic re-opening of port after disconnection
	while ( ros::ok() )
	{
		ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "..." );
		try
		{
			controller.open();
			if ( controller.isOpen() )
			{
				ROS_INFO("Successfully opened serial port");
				break;
			}
		}
		catch (serial::IOException e)
		{
			ROS_WARN_STREAM("serial::IOException: " << e.what());
		}
		ROS_WARN("Failed to open serial port");
		sleep( 5 );
	}
	cmdvel_setup();
//	odom_setup();
  starttime = millis();
  hstimer = starttime;
  mstimer = starttime;
  lstimer = starttime;
//  ros::Rate loop_rate(10);
  ROS_INFO("Beginning looping...");
  while (ros::ok())
  {

    //cmdvel_loop();
    // odom_loop();

    uint32_t nowtime = millis();
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << DELTAT(nowtime,lstimer) << " / " << (nowtime-lstimer));
//uint32_t delta = DELTAT(nowtime,lstimer);
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << delta << " / " << (nowtime-lstimer));

//    // Handle 50 Hz publishing
//    if (DELTAT(nowtime,hstimer) >= 20)
    // Handle 30 Hz publishing
    if (DELTAT(nowtime,hstimer) >= 33)
    {
      hstimer = nowtime;
//      odom_hs_run();
    }

    // Handle 10 Hz publishing
    if (DELTAT(nowtime,mstimer) >= 100)
    {
      mstimer = nowtime;
      // cmdvel_run();
      // odom_ms_run();
    }

    // Handle 1 Hz publishing
    if (DELTAT(nowtime,lstimer) >= 1000)
    {
      lstimer = nowtime;
      // odom_ls_run();
    }

    ros::spinOnce();

//    loop_rate.sleep();
  }
	
  if ( controller.isOpen() )
    controller.close();

  ROS_INFO("Exiting");
	
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_node");
  MainNode node;
  signal(SIGINT, mySigintHandler);
  return node.run();
}

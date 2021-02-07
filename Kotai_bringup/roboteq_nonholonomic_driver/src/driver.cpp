#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include <string>
#include <signal.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <roboteq_nonholonomic_msgs/Ackermann.h>

//#define _SENSORS

#define DELTAT(_nowtime, _thentime) ((_thentime > _nowtime) ? ((0xffffffff - _thentime) + _nowtime) : (_nowtime - _thentime))

float scale(float A, float A1, float A2, float Min, float Max)
{
  long double percentage = (A - A1) / (A1 - A2);
  return (percentage) * (Min - Max) + Min;
}
uint32_t millis()
{
  ros::WallTime walltime = ros::WallTime::now();
  //	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
  //	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
  return (uint32_t)(walltime.toNSec() / 1000000);
}
double constrainAngle(double x)
{
  x = fmod(x, 360);
  if (x < 0)
    x += 360;
  return x;
}
void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}

class MainNode
{

public:
  MainNode();

public:
  int run();
  void cmdvel_callback(const roboteq_nonholonomic_msgs::Ackermann &twist_msg);
  void cmdvel_setup();

  void odom_setup();
  void odom_stream();
  void odom_loop();
  void odom_publish();

protected:
  serial::Serial controller;
  ros::NodeHandle nh;
  ros::Subscriber cmdvel_sub;
  ros::Publisher odom_pub;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  tf::TransformBroadcaster odom_broadcaster;
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string port;
  int baud;
  bool open_loop;
  int kp_velocity;
  int ki_velocity;
  int kd_velocity;
  int kp_streeing;
  int ki_streeing;
  int kd_streeing;
  double wheel_to_streeing_width;
  double wheel_circumference;
#ifdef _SENSORS
  float voltage;
  float current_right;
  float current_left;
  float energy;
  float temperature;
  uint32_t current_last_time;
#endif
  uint32_t odom_last_time;

  int odom_idx;
  char odom_buf[24];

  char odom_encoder_toss;
  int32_t odom_encoder_velocity;
  int32_t odom_encoder_steering;
  int32_t odom_encoder_steering_sum;

  float odom_encode_pos;
  float odom_steering;
  float odom_encode_last_pos;
  float odom_steering_last_pos;
  float heading;
  float velocity;
  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;
};
void MainNode::cmdvel_callback(const roboteq_nonholonomic_msgs::Ackermann &Ackermann_msg)
{
  float velocity = Ackermann_msg.speed;

  float steering = Ackermann_msg.steering_angle;
  std::stringstream velocity_cmd;
  std::stringstream steering_cmd;
  if (open_loop)
  {
    int32_t velocity_power = velocity * 100;
    int32_t steering_power = steering * 1000;
#ifdef _CMDVEL_DEBUG
// ROS_DEBUG_STREAM("cmdvel power right: " << right_power << " left: " << left_power);
#endif
    velocity_cmd << "!G 1 " << velocity_power << "\r";
    steering_cmd << "!G 2 " << steering_power << "\r";
  }
  else
  {
    int32_t velocity_power = velocity * 100;
    int32_t steering_power = steering * 1000;
    //ROS_INFO("odom_encoder_velocity %d", velocity_power);
    // #ifdef _CMDVEL_DEBUG<< right_rpm << " left: " << left_rpm);
    // #endif
    velocity_cmd << "!S 1 " << velocity_power << "\r";
    steering_cmd << "!G 2 " << steering_power << "\r";
     
  }
#ifndef _CMDVEL_FORCE_RUN

  controller.write(velocity_cmd.str());
  controller.write(steering_cmd.str());
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
  controller.write("^RWD 500\r");
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
  // set PID parameters (gain * 10)
  std::stringstream velocity_pcmd;
  std::stringstream velocity_icmd;
  std::stringstream velocity_dcmd;
  std::stringstream steering_pcmd;
  std::stringstream steering_icmd;
  std::stringstream steering_dcmd;
  velocity_pcmd << "^KP 1 " << kp_velocity * 10 << "\r";
  velocity_icmd << "^KI 1 " << ki_velocity * 10 << "\r";
  velocity_dcmd << "^KD 1 " << kd_velocity * 10 << "\r";
  steering_pcmd << "^KP 2 " << kp_streeing * 10 << "\r";
  steering_icmd << "^KI 2 " << ki_streeing * 10 << "\r";
  steering_dcmd << "^KD 2 " << kd_streeing * 10 << "\r";
  controller.write(velocity_pcmd.str());
  controller.write(velocity_icmd.str());
  controller.write(velocity_dcmd.str());
  controller.write(steering_pcmd.str());
  controller.write(steering_icmd.str());
  controller.write(steering_dcmd.str());
  // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
  controller.write("^EMOD 1 18\r");
  controller.write("^EMOD 2 34\r");
  controller.flush();
  ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
  cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &MainNode::cmdvel_callback, this);
}

void MainNode::odom_setup()
{

  if (pub_odom_tf)
  {
    ROS_INFO("Broadcasting odom tf");
  }

  ROS_INFO_STREAM("Publishing to topic " << odom_topic);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);

#ifdef _ODOM_COVAR_SERVER
  ROS_INFO("Advertising service on roboteq/odom_covar_srv");
  odom_covar_server = nh.advertiseService("roboteq/odom_covar_srv", &MainNode::odom_covar_callback, this);
#endif

#ifdef _SENSORS
  ROS_INFO("Publishing to topic roboteq/voltage");
  voltage_pub = nh.advertise<std_msgs::Float32>("roboteq/voltage", 1000);
  ROS_INFO("Publishing to topic roboteq/current");
  current_pub = nh.advertise<roboteq_diff_msgs::Duplex>("roboteq/current", 1000);
  ROS_INFO("Publishing to topic roboteq/energy");
  energy_pub = nh.advertise<std_msgs::Float32>("roboteq/energy", 1000);
  ROS_INFO("Publishing to topic roboteq/temperature");
  temperature_pub = nh.advertise<std_msgs::Float32>("roboteq/temperature", 1000);
#endif

  tf_msg.header.seq = 0;
  tf_msg.header.frame_id = odom_frame;
  tf_msg.child_frame_id = base_frame;
  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;
  odom_msg.pose.covariance.assign(0);
  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.001;
  odom_msg.pose.covariance[14] = 1000000;
  odom_msg.pose.covariance[21] = 1000000;
  odom_msg.pose.covariance[28] = 1000000;
  odom_msg.pose.covariance[35] = 1000;
  odom_msg.twist.covariance.assign(0);
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[7] = 0.001;
  odom_msg.twist.covariance[14] = 1000000;
  odom_msg.twist.covariance[21] = 1000000;
  odom_msg.twist.covariance[28] = 1000000;
  odom_msg.twist.covariance[35] = 1000;

#ifdef _SENSORS
//  voltage_msg.header.seq = 0;
//  voltage_msg.header.frame_id = 0;
//  current_msg.header.seq = 0;
//  current_msg.header.frame_id = 0;
//  energy_msg.header.seq = 0;
//  energy_msg.header.frame_id = 0;
//  temperature_msg.header.seq = 0;
//  temperature_msg.header.frame_id = 0;
#endif
  odom_stream();

  odom_last_time = millis();
#ifdef _SENSORS
  current_last_time = millis();
#endif
}

void MainNode::odom_stream()
{
#ifdef _SENSORS
  controller.write("# C_?CR_?BA_?V_# 11\r");
#else

  controller.write("# C_?C_?CR_# 33\r");
#endif
  controller.flush();
}

void MainNode::odom_loop()
{

  uint32_t nowtime = millis();

  // if we haven't received encoder counts in some time then restart streaming
  if (DELTAT(nowtime, odom_last_time) >= 1000)
  {
    odom_stream();
    odom_last_time = nowtime;
  }

  //   // read sensor data stream from motor controller
  if (controller.available())
  {
    char ch = 0;
    if (controller.read((uint8_t *)&ch, 1) == 0)
      return;
    if (ch == '\r')
    {

      odom_buf[odom_idx] = 0;
      // CR= is encoder counts
      if (odom_buf[0] == 'C' && odom_buf[1] == 'R' && odom_buf[2] == '=')
      {
        int delim;
        for (delim = 3; delim < odom_idx; delim++)
        {
          if (odom_encoder_toss > 0)
          {
            --odom_encoder_toss;
            break;
          }
          if (odom_buf[delim] == ':')
          {
            odom_buf[delim] = 0;
            odom_encoder_velocity = (int32_t)strtol(odom_buf + 3, NULL, 10);
            odom_encoder_steering = odom_encoder_steering + (int32_t)strtol(odom_buf + delim + 1, NULL, 10);

#ifdef _ODOM_DEBUG
            ROS_DEBUG_STREAM("encoder right: " << odom_encoder_right << " left: " << odom_encoder_left);
#endif
            odom_publish();
            break;
          }
        }
      }

      // else if (odom_buf[0] == 'C' && odom_buf[1] == '=')
      // {
      //   int count = 0;
      //   int start = 2;
      //   for (int delim = 2; delim <= odom_idx; delim++)
      //   {
      //     if (odom_buf[delim] == ':' )
      //     {
      //       odom_buf[delim] = 0;
      //       if (count == 1)
      //       {
      //         voltage = (float)strtol(odom_buf + start, NULL, 10) / 10.0;
      //         break;
      //       }
      //       start = delim + 1;
      //       count++;
      //     }
      //   }
      // }
#ifdef _ODOM_SENSORS
      // V= is voltages
      else if (odom_buf[0] == 'V' && odom_buf[1] == '=')
      {
        int count = 0;
        int start = 2;
        for (int delim = 2; delim <= odom_idx; delim++)
        {
          if (odom_buf[delim] == ':' || odom_buf[delim] == 0)
          {
            odom_buf[delim] = 0;
            if (count == 1)
            {
              voltage = (float)strtol(odom_buf + start, NULL, 10) / 10.0;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM("voltage: " << voltage);
#endif
              break;
            }
            start = delim + 1;
            count++;
          }
        }
      }
      // BA= is motor currents
      else if (odom_buf[0] == 'B' && odom_buf[1] == 'A' && odom_buf[2] == '=')
      {
        int delim;
        for (delim = 3; delim < odom_idx; delim++)
        {
          if (odom_buf[delim] == ':')
          {
            odom_buf[delim] = 0;
            current_right = (float)strtol(odom_buf + 3, NULL, 10) / 10.0;
            current_left = (float)strtol(odom_buf + delim + 1, NULL, 10) / 10.0;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM("current right: " << current_right << " left: " << current_left);
#endif

            // determine delta time in seconds
            float dt = (float)DELTAT(nowtime, current_last_time) / 1000.0;
            current_last_time = nowtime;
            energy += (current_right + current_left) * dt / 3600.0;
            break;
          }
        }
      }
#endif
      odom_idx = 0;
    }
    else if (odom_idx < (sizeof(odom_buf) - 1))
    {
      odom_buf[odom_idx++] = ch;
    }
  }
}

void MainNode::odom_publish()
{

  // determine delta time in seconds
  uint32_t nowtime = millis();
  float dt = (float)DELTAT(nowtime, odom_last_time) / 1000.0;
  odom_last_time = nowtime;

  if (dt < 0.001)
  {
    return;
  }
  ///////////////drive return 
   velocity = ((float)odom_encoder_velocity / 1440.0 * 0.957072);
  odom_steering = scale(odom_encoder_steering, -314, 367, -35.0, 35.0);
  ////////////////////////Find Velocity Encode///////////////////////////////////
  //velocity = float((odom_encode_pos - odom_encode_last_pos) / dt);
  heading += (velocity / 0.50) * tan(odom_steering * 3.1414 / 180.0);
  /////////////////pos odom xy /////////////////////////////////////////
  odom_x += (velocity * cos(constrainAngle(heading) * 3.1414 / 180.0)) ; // m
  //odom_x += (velocity) ; // m
  odom_y += (velocity * sin(constrainAngle(heading) * 3.1414 / 180.0)) * dt; // m
  /////////////////////////////////////Vx Vy //////////////////////////////
  float vx = (odom_x - odom_last_x) / dt;
  float vy = (odom_y - odom_last_y) / dt;
  float vyaw = (constrainAngle(heading) - odom_last_yaw) / dt;
  odom_last_x = odom_x;
  odom_last_y = odom_y;
  odom_last_yaw = constrainAngle(heading);
  odom_encode_last_pos = odom_encode_pos;

  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(constrainAngle(heading) * 3.1414 / 180.0);

  if (pub_odom_tf)
  {
    tf_msg.header.seq++;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.transform.translation.x = odom_x;
    tf_msg.transform.translation.y = odom_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = quat;
    odom_broadcaster.sendTransform(tf_msg);
  }

  odom_msg.header.seq++;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_pub.publish(odom_msg);

}

MainNode::MainNode() : pub_odom_tf(true),
                       open_loop(false),
                       baud(115200),
                       kp_velocity(0),
                       ki_velocity(5),
                       kd_velocity(0),
                       kp_streeing(5),
                       ki_streeing(0),
                       kd_streeing(0),
                       wheel_to_streeing_width(0),
                       wheel_circumference(0),
#ifdef _SENSORS
                       voltage(0.0),
                       current_right(0.0),
                       current_left(0.0),
                       energy(0.0),
                       temperature(0.0),
                       current_last_time(0),
#endif
                       odom_idx(0),
                       odom_encoder_toss(5),
                       odom_encoder_velocity(0),
                       odom_encoder_steering(0),
                       odom_encode_pos(0.0),
                       odom_steering(0.0),
                       odom_encode_last_pos(0.0),
                       odom_steering_last_pos(0.0),
                       odom_x(0.0),
                       odom_y(0.0),
                       odom_yaw(0.0),
                       odom_last_x(0.0),
                       odom_last_y(0.0),
                       odom_last_yaw(0.0),
                       odom_last_time(0),
                       heading(0.0),
                       velocity(0.0)

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
  nhLocal.param("kp_velocity", kp_velocity, 0);
  ROS_INFO_STREAM("kp_velocity: " << kp_velocity);
  nhLocal.param("ki_velocity", ki_velocity, 10);
  ROS_INFO_STREAM("ki_velocity: " << ki_velocity);
  nhLocal.param("kd_velocity", kd_velocity, 0);
  ROS_INFO_STREAM("kd_velocity: " << kd_velocity);
  nhLocal.param("kp_streeing", kp_streeing, 5);
  ROS_INFO_STREAM("kp_streeing: " << kp_streeing);
  nhLocal.param("ki_streeing", ki_streeing, 0);
  ROS_INFO_STREAM("ki_streeing: " << ki_streeing);
  nhLocal.param("kd_streeing", kd_streeing, 0);
  ROS_INFO_STREAM("kd_streeing: " << kd_streeing);
  nhLocal.param("wheel_to_streeing_width", wheel_to_streeing_width, 0.50);
  ROS_INFO_STREAM("wheel_to_streeing_width: " << wheel_to_streeing_width);
  nhLocal.param("wheel_circumference", wheel_circumference, 0.50);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
}
int MainNode::run()
{
  ROS_INFO("Beginning setup...");
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  controller.setPort(port);
  controller.setBaudrate(baud);
  controller.setTimeout(timeout);
  // TODO: support automatic re-opening of port after disconnection
  while (ros::ok())
  {
    ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "...");
    try
    {
      controller.open();
      if (controller.isOpen())
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
    sleep(5);
  }

  //////setup controller///
  cmdvel_setup();
  odom_setup();
  ROS_INFO("Beginning looping...");
  while (ros::ok())
  {
    //////program////
    odom_loop();
    ros::spinOnce();
  }

  if (controller.isOpen())
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
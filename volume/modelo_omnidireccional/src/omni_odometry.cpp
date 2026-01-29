#include "omni_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace robmovil;

#define WHEEL_RADIUS 0.05
#define LX 0.175
#define LY 0.175
#define ENCODER_TICKS 500.0


OmniOdometry::OmniOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10), std::bind(&OmniOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_fl_wheel_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_fr_wheel_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rl_wheel_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rr_wheel_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));

  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>("/robot/encoders", rclcpp::QoS(10), std::bind(&OmniOdometry::on_encoder_ticks, this, std::placeholders::_1));
  
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OmniOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  /** Usamos eq. 19 para la cinematica inversa*/
  const double vx = twist->linear.x;
  const double vy = twist->linear.y;
  const double wz = twist->angular.z;

  double fl_wheel = (vx - vy - (LX + LY)*wz)/WHEEL_RADIUS;
  double fr_wheel = (vx + vy + (LX + LY)*wz)/WHEEL_RADIUS;
  double rl_wheel = (vx + vy - (LX + LY)*wz)/WHEEL_RADIUS;
  double rr_wheel = (vx - vy + (LX + LY)*wz)/WHEEL_RADIUS;

  {
    std_msgs::msg::Float64 msg;
    msg.data = fl_wheel;

    vel_pub_fl_wheel_->publish(msg);
  }

  {
    std_msgs::msg::Float64 msg;
    msg.data = fr_wheel;

    vel_pub_fr_wheel_->publish(msg);
  }

  {
    std_msgs::msg::Float64 msg;
    msg.data = rl_wheel;

    vel_pub_rl_wheel_->publish(msg);
  }

  {
    std_msgs::msg::Float64 msg;
    msg.data = rr_wheel;

    vel_pub_rr_wheel_->publish(msg);
  }
}

void OmniOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    for(int i = 0; i < 4; i++){
      last_ticks_[i] = encoder->ticks[i];
    }
    last_ticks_time = encoder->header.stamp;
    return;
  }

  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  /* Obtenemos las velocidades de cada rueda en rad/s */
  double radsPorTick = (M_PI*2)/ENCODER_TICKS;

  double w1 = ((encoder->ticks[0] - last_ticks_[0])*radsPorTick)/delta_t; // fl
  double w2 = ((encoder->ticks[1] - last_ticks_[1])*radsPorTick)/delta_t; // fr
  double w3 = ((encoder->ticks[2] - last_ticks_[2])*radsPorTick)/delta_t; // rl
  double w4 = ((encoder->ticks[3] - last_ticks_[3])*radsPorTick)/delta_t; // rr

  /* Usamos eq. 21 para calcular cinematica directa */
  double vx = (w1 + w2 + w3 + w4)*(WHEEL_RADIUS/4);
  double vy = (-w1 + w2 + w3 + -w4)*(WHEEL_RADIUS/4);
  double wz = (-w1 + w2 + -w3 + w4)*(WHEEL_RADIUS/(4*(LX + LY)));

  /* Usamos mismas ecuaciones que en el diferencial pero ahora vy != 0 */
  double delta_x = (vx * cos(theta_) - vy * sin(theta_))*delta_t;
  double delta_y = (vx * sin(theta_) + vy * cos(theta_))*delta_t;
  double delta_theta = wz*delta_t;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;
  
  // Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "map";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x  = vx;
  msg.twist.twist.linear.y  = vy;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = wz;

  pub_odometry_->publish( msg );

  // Actualizamos las variables de estado

  for(int i = 0; i < 4; i++){
    last_ticks_[i] = encoder->ticks[i];
  }
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);


}

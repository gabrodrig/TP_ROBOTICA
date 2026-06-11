#include "keys_to_twist.h"

using namespace robmovil;

KeysToTwist::KeysToTwist() : Node("nh")
{
  // KeysToTwist se suscribe a los tópicos que reportan cuando se apreta o suelta una tecla. (en los cuales publica keyboard)
  key_up_sub_ = this->create_subscription<keyboard_msgs::msg::Key>("/keyup", 10, std::bind(&KeysToTwist::on_key_up, this, std::placeholders::_1));
  key_down_sub_ = this->create_subscription<keyboard_msgs::msg::Key>("/keydown", 10, std::bind(&KeysToTwist::on_key_down, this, std::placeholders::_1));

  // publicamos en /robot/cmd_vel las velocidades correspondientes segun lo que suceda con el keyboard
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

  up_ = down_ = left_ = right_ = q_ = e_ = false;
}

void KeysToTwist::on_key_up(const keyboard_msgs::msg::Key::SharedPtr key_event)
{
  geometry_msgs::msg::Twist twist;

  switch( key_event->code )
  {
    case keyboard_msgs::msg::Key::KEY_W:
      up_ = false;
      break;

    case keyboard_msgs::msg::Key::KEY_S:
      down_ = false;
      break;

    case keyboard_msgs::msg::Key::KEY_A:
      left_ = false;
      break;

    case keyboard_msgs::msg::Key::KEY_D:
      right_ = false;
      break;

    case keyboard_msgs::msg::Key::KEY_Q:
      q_ = false;
      break;

    case keyboard_msgs::msg::Key::KEY_E:
      e_ = false;
      break;

    default:
      return;
  }
  const double linear_x_v = 1.0;
  const double linear_y_v = 1.0;
  const double angular_v = 4.0;

  if(up_ && !down_){
    twist.linear.x = linear_x_v;
  }
  if((up_ && down_) || (!up_ && !down_)){
    twist.linear.x = 0;
  }
  if(!up_ && down_){
    twist.linear.x = -linear_x_v;
  }
  if(q_ && !e_){
    twist.angular.z = angular_v;
  }
  if((q_ && e_) || (!q_ && !e_)){
    twist.angular.z = 0;
  }
  if(!q_ && e_){
    twist.angular.z = -angular_v;
  }
  if(left_ && !right_){
    twist.linear.y = linear_y_v;
  }
  if(!left_ && right_){
    twist.linear.y = -linear_y_v;
  }
  if((left_ && right_) || (!left_ && !right_)){
    twist.linear.y = 0;
  }
  twist_pub_->publish( twist );
}

void KeysToTwist::on_key_down(const keyboard_msgs::msg::Key::SharedPtr key_event)
{
  geometry_msgs::msg::Twist twist;

  switch( key_event->code )
  {
    case keyboard_msgs::msg::Key::KEY_W:
      up_ = true;
      break;

    case keyboard_msgs::msg::Key::KEY_S:
      down_ = true;
      break;

    case keyboard_msgs::msg::Key::KEY_A:
      left_ = true;
      break;

    case keyboard_msgs::msg::Key::KEY_D:
      right_ = true;
      break;

    case keyboard_msgs::msg::Key::KEY_Q:
      q_ = true;
      break;

    case keyboard_msgs::msg::Key::KEY_E:
      e_ = true;
      break;

    default:
      return;
  }

  const double linear_x_v = 1.0;
  const double linear_y_v = 1.0;
  const double angular_v = 4.0;

  if(up_ && !down_){
    twist.linear.x = linear_x_v;
  }
  if((up_ && down_) || (!up_ && !down_)){
    twist.linear.x = 0;
  }
  if(!up_ && down_){
    twist.linear.x = -linear_x_v;
  }
  if(q_ && !e_){
    twist.angular.z = angular_v;
  }
  if((q_ && e_) || (!q_ && !e_)){
    twist.angular.z = 0;
  }
  if(!q_ && e_){
    twist.angular.z = -angular_v;
  }
  if(left_ && !right_){
    twist.linear.y = linear_y_v;
  }
  if(!left_ && right_){
    twist.linear.y = -linear_y_v;
  }
  if((left_ && right_) || (!left_ && !right_)){
    twist.linear.y = 0;
  }

  twist_pub_->publish( twist );
}

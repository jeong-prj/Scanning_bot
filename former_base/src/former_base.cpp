/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "former_base/former_base.h"

FormerBase::FormerBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh)
  , nh_priv_(nh_priv)
  , port_("/dev/ttyUSB0")
  , baud_(115200)
  , robot_("former")
  , joint_()
  , wheel_radius_(0.085)
  , max_speed_(1.5)
  , max_rpm_(200)
  , control_frequency_(50.0)
  , previous_state_(false)
{
  nh_priv_.getParam("port", port_);
  nh_priv_.getParam("baud", baud_);
  nh_priv_.getParam("robot", robot_);
  nh_priv_.getParam("joint", joint_);
  nh_priv_.getParam("control_frequency", control_frequency_);

  controller_ = make_shared<RoasController>(robot_, joint_, wheel_radius_, max_speed_, max_rpm_, port_, baud_,
                                            feedback_, estop_state_);
  hw_ = make_shared<RoasHardware>(joint_, robot_);
  cm_ = make_shared<controller_manager::ControllerManager>(hw_.get(), nh_);
  diagnostics_ = make_shared<RoasDiagnostics>(robot_, feedback_);
}

bool FormerBase::init()
{
  srv_led_ = nh_.serviceClient<roas_base::SetLed>("/led/control");

  cmd_.command.assign(joint_.size(), 0.0);

  rp_feedback_.init(nh_, robot_ + "/feedback", 1);
  rp_feedback_.msg_.robot = robot_;

  for (size_t i = 0; i < ceil(joint_.size() / 2.0); i++)
  {
    rp_feedback_.msg_.motor_state.push_back(former_msgs::MotorState());
    feedback_.motor_state.push_back(MotorState());
  }

  rp_estop_.init(nh_, "/emergency_stop", 1);
  rp_estop_.msg_.data = false;

  if (!controller_->init())
    return false;

  if (!controller_->connect())
    return false;

  controller_->restartScript();

  cmd_led_.request.color = roas_base::SetLed::Request::MINT;
  cmd_led_.request.mode = roas_base::SetLed::Request::NORMAL;
  srv_led_.call(cmd_led_);

  return true;
}

void FormerBase::publishFeedback()
{
  if (rp_feedback_.trylock())
  {
    rp_feedback_.msg_.header.stamp = ros::Time::now();
    rp_feedback_.msg_.robot_state.emergency_stop = feedback_.robot_state.emergency_stop;
    rp_feedback_.msg_.robot_state.battery_voltage = feedback_.robot_state.battery_voltage;
    rp_feedback_.msg_.robot_state.charging_voltage = feedback_.robot_state.charging_voltage;
    rp_feedback_.msg_.robot_state.user_12v_current = feedback_.robot_state.user_12v_current;
    rp_feedback_.msg_.robot_state.user_24v_current = feedback_.robot_state.user_24v_current;

    for (size_t i = 0; i < ceil(joint_.size() / 2.0); i++)
    {
      rp_feedback_.msg_.motor_state[i].position = feedback_.motor_state[i].position;
      rp_feedback_.msg_.motor_state[i].velocity = feedback_.motor_state[i].velocity;
      rp_feedback_.msg_.motor_state[i].current = feedback_.motor_state[i].current;
      rp_feedback_.msg_.motor_state[i].temperature = feedback_.motor_state[i].temperature;
      rp_feedback_.msg_.motor_state[i].fault_flags = feedback_.motor_state[i].fault_flags;
    }
    rp_feedback_.unlockAndPublish();
  }
}

void FormerBase::publishEstopState()
{
  if (estop_state_.data != previous_state_)
  {
    if (rp_estop_.trylock())
    {
      rp_estop_.msg_ = estop_state_;
      rp_estop_.unlockAndPublish();

      if (estop_state_.data)
      {
        cmd_led_.request.color = roas_base::SetLed::Request::RED;
        cmd_led_.request.mode = roas_base::SetLed::Request::BLINK;
        srv_led_.call(cmd_led_);
      }
      else
      {
        cmd_led_.request.color = roas_base::SetLed::Request::MINT;
        cmd_led_.request.mode = roas_base::SetLed::Request::NORMAL;
        srv_led_.call(cmd_led_);
      }
    }
    previous_state_ = rp_estop_.msg_.data;
  }
}

void FormerBase::publishLoop()
{
  while (ros::ok())
  {
    publishFeedback();
    publishEstopState();
    diagnostics_->updateDiagnosticsMessage();

    ros::Rate(control_frequency_).sleep();
  }
}

void FormerBase::controlLoop()
{
  chrono::steady_clock::time_point last_time = chrono::steady_clock::now();

  while (ros::ok())
  {
    chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
    chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw_->receiveFeedback(feedback_);
    cm_->update(ros::Time::now(), elapsed);
    hw_->writeCommandToHardware(cmd_);
    controller_->sendHearbeat();
    controller_->sendCommand(cmd_);

    ros::Rate(control_frequency_).sleep();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "former_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  auto former = make_shared<FormerBase>(nh, nh_priv);

  if (former->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create thread for controller manager loop
    thread control([&former]() -> void { former->controlLoop(); });

    // Create thread for publishing the feedback data
    thread publish([&former]() -> void { former->publishLoop(); });

    while (ros::ok())
      former->controller_->read();

    spinner.stop();
  }

  return EXIT_FAILURE;
}
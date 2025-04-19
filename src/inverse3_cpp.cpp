#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "hd_cpp/HardwareAPI.h"

using namespace std::chrono_literals;

class Inverse3Node : public rclcpp::Node
{
public:
  Inverse3Node()
      : Node("inverse3_controller")
  {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("inv3_pose", 10);
    btn_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("inv3_btn", 10);

    setup_device();
    control_thread_ = std::thread(&Inverse3Node::control_loop, this);
  }

  ~Inverse3Node()
  {
    running_ = false;
    if (control_thread_.joinable())
      control_thread_.join();
  }

private:
  void setup_device()
  {
    auto inv3_ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectInverse3s();
    auto vp_ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectHandles();

    if (inv3_ports.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No Inverse3 devices found.");
      rclcpp::shutdown();
      return;
    }

    if (vp_ports.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No Verse Grip devices found.");
      rclcpp::shutdown();
      return;
    }

    try
    {
      inv3_serial_ = std::make_shared<Haply::HardwareAPI::IO::SerialStream>(inv3_ports[0].c_str());
      vg_serial_ = std::make_shared<Haply::HardwareAPI::IO::SerialStream>(vp_ports[0].c_str());

      if (vg_serial_->OpenDevice() < 0)
      {
        throw std::runtime_error("Unable to open Verse Grip serial stream.");
      }

      versegrip_ = std::make_shared<Haply::HardwareAPI::Devices::Handle>(vg_serial_.get());
      versegrip_->SendDeviceWakeup();

      // Receive the response to actually process it
      versegrip_->Receive(); // <- VERY important: Receive the reply from device
      auto vg_response = versegrip_->GetVersegripStatus();
      RCLCPP_INFO(this->get_logger(), "Connected to Verse Grip device ID: %d", vg_response.device_id);

      if (inv3_serial_->OpenDevice() < 0)
      {
        throw std::runtime_error("Unable to open Inverse3 serial stream.");
      }

      inverse3_ = std::make_shared<Haply::HardwareAPI::Devices::Inverse3>(inv3_serial_.get());
      auto inv3_response = inverse3_->DeviceWakeup();
      RCLCPP_INFO(this->get_logger(), "Connected to Inverse3 device ID: %d", inv3_response.device_id);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Device setup failed: %s", e.what());
      rclcpp::shutdown();
    }

    inverse3_->SendSetGravityCompensation(true, 1.0f);    // Enable gravity compensation at 1.0 scale
    int result = inverse3_->ReceiveGravityCompensation(); // Important to receive confirmation

    if (result < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive gravity compensation confirmation.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Gravity compensation enabled successfully.");
    }
  }

  void control_loop()
  {

    while (rclcpp::ok() && running_)
    {

      auto inv3_res = inverse3_->GetEndEffectorPosition();
      auto vg_res = versegrip_->GetVersegripStatus();
      // Create PoseStamped message
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = "inv3_frame";

      pose_msg.pose.position.x = -inv3_res.position[0]; // coordinate adjustment
      pose_msg.pose.position.y = -inv3_res.position[1];
      pose_msg.pose.position.z = inv3_res.position[2];

      pose_publisher_->publish(pose_msg);

      // Create Joy message (Button publishing - not yet implemented, placeholder)
      sensor_msgs::msg::Joy btn_msg;
      btn_msg.buttons.resize(2, 0); // Ensure the buttons vector has size 2 and initialize to 0
      if (vg_res.buttons & 0x01)    // Check if the first button is pressed
      {
        btn_msg.buttons[0] = 1;
      }
      if (vg_res.buttons & 0x02) // Check if the second button is pressed
      {
        btn_msg.buttons[1] = 1;
      }
      btn_publisher_->publish(btn_msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1 ms loop
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr btn_publisher_;
  std::thread control_thread_;
  bool running_ = true;

  std::shared_ptr<Haply::HardwareAPI::Devices::Inverse3> inverse3_;
  std::shared_ptr<Haply::HardwareAPI::Devices::Handle> versegrip_;
  std::shared_ptr<Haply::HardwareAPI::IO::SerialStream> inv3_serial_;
  std::shared_ptr<Haply::HardwareAPI::IO::SerialStream> vg_serial_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Inverse3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

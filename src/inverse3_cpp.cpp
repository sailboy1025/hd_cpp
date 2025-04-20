#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

#include "hd_cpp/HardwareAPI.h"

namespace API = Haply::HardwareAPI;
using namespace std::chrono_literals;
class Inverse3Node : public rclcpp::Node
{
public:
  Inverse3Node(int argc, char *argv[])
      : Node("inverse3_controller")
  {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("inv3_pose", 10);
    btn_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("inv3_btn", 10);
    force_lock_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "force_lock", 10, std::bind(&Inverse3Node::force_lock_callback, this, std::placeholders::_1));


    // Parse arguments
    for (int i = 1; i < argc; ++i)
    {
      std::string arg = argv[i];
      if (arg == "-gc")
      {
        gravity_compensation_ = true;
        RCLCPP_INFO(this->get_logger(), "Gravity compensation ENABLED via -gc");
      }
      else if (arg == "-nogc")
      {
        gravity_compensation_ = false;
        RCLCPP_INFO(this->get_logger(), "Gravity compensation DISABLED via -nogc");
      }
    }
    setup_device();
    control_thread_ = std::thread(&Inverse3Node::control_loop, this);
  }

  ~Inverse3Node()
  {
    if (control_thread_.joinable())
      control_thread_.join();
  }

private:
  void setup_device()
  {
    auto inv3_ports = API::Devices::DeviceDetection::DetectInverse3s();
    auto vp_ports = API::Devices::DeviceDetection::DetectHandles();

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
      inv3_serial_ = std::make_shared<API::IO::SerialStream>(inv3_ports[0].c_str());
      vg_serial_ = std::make_shared<API::IO::SerialStream>(vp_ports[0].c_str());

      if (vg_serial_->OpenDevice() < 0)
      {
        throw std::runtime_error("Unable to open Verse Grip serial stream.");
      }

      versegrip_ = std::make_shared<API::Devices::Handle>(vg_serial_.get());
      versegrip_->SendDeviceWakeup();

      // Receive the response to actually process it
      versegrip_->Receive(); // <- VERY important: Receive the reply from device
      auto vg_response = versegrip_->GetVersegripStatus();
      RCLCPP_INFO(this->get_logger(), "Connected to Verse Grip device ID: %d", vg_response.device_id);

      if (inv3_serial_->OpenDevice() < 0)
      {
        throw std::runtime_error("Unable to open Inverse3 serial stream.");
      }

      inverse3_ = std::make_shared<API::Devices::Inverse3>(inv3_serial_.get());
      auto inv3_response = inverse3_->DeviceWakeup();
      RCLCPP_INFO(this->get_logger(), "Connected to Inverse3 device ID: %d", inv3_response.device_id);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Device setup failed: %s", e.what());
      rclcpp::shutdown();
    }

    set_gravity_compensation(gravity_compensation_);
  }

  void control_loop()
  {
    typedef std::chrono::high_resolution_clock clock;
    auto next = clock::now();
    auto delay = 200us; // Target 5kHz

    API::Devices::Inverse3::EndEffectorStateResponse inv3_res;

    while (rclcpp::ok())
    {
      API::Devices::Inverse3::EndEffectorForceRequest request;

      if (force_lock_)
      {
        force_lock_inv3(inv3_res, request);
      }

      // Send force and receive new state
      inv3_res = inverse3_->EndEffectorForce(request);

      auto vg_res = versegrip_->GetVersegripStatus();

      // Publish pose
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = "inv3_frame";
      pose_msg.pose.position.x = inv3_res.position[0];
      pose_msg.pose.position.y = inv3_res.position[1];
      pose_msg.pose.position.z = inv3_res.position[2];
      pose_publisher_->publish(pose_msg);

      // Publish buttons
      sensor_msgs::msg::Joy btn_msg;
      btn_msg.buttons.resize(2, 0);
      if (vg_res.buttons & 0x01)
        btn_msg.buttons[0] = 1;
      if (vg_res.buttons & 0x02)
        btn_msg.buttons[1] = 1;
      btn_publisher_->publish(btn_msg);

      // High-precision timing (busy wait to next cycle)
      next += delay;
      while (next > clock::now())
        ;
    }
  }

  void force_lock_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    force_lock_ = msg->data;
  }

  void force_lock_inv3(
      const API::Devices::Inverse3::EndEffectorStateResponse &current_position,
      API::Devices::Inverse3::EndEffectorForceRequest &request)
  {
    float lock_center[3] = {0.03f, -0.16f, 0.2f};
    float Kp = 50.0f;

    for (int i = 0; i < 3; ++i)
    {
      request.force[i] = Kp * (lock_center[i] - current_position.position[i]);
    }
  }

  void set_gravity_compensation(bool enable)
  {
    inverse3_->SendSetGravityCompensation(enable, 1.0f);
    int result = inverse3_->ReceiveGravityCompensation();

    if (result < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive gravity compensation confirmation.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Gravity compensation %s successfully.", enable ? "enabled" : "disabled");
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr btn_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr force_lock_subscriber_;
  bool force_lock_{false};
  bool gravity_compensation_{true};
  std::thread control_thread_;

  std::shared_ptr<API::Devices::Inverse3> inverse3_;
  std::shared_ptr<API::Devices::Handle> versegrip_;
  std::shared_ptr<API::IO::SerialStream> inv3_serial_;
  std::shared_ptr<API::IO::SerialStream> vg_serial_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Inverse3Node>(argc, argv);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

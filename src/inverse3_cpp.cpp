#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <termios.h>
#include <unistd.h>

#include "hd_cpp/HardwareAPI.h"

namespace API = Haply::HardwareAPI;
using namespace std::chrono_literals;
using json = nlohmann::json;

class Inverse3Node : public rclcpp::Node
{
public:
  Inverse3Node(int argc, char *argv[])
      : Node("inverse3_controller")
  {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    btn_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("button", 10);
    force_lock_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "HD_force_lock", 10, std::bind(&Inverse3Node::force_lock_callback, this, std::placeholders::_1));
    gc_toggle_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "HD_gc_toggle", 10,
        std::bind(&Inverse3Node::gc_toggle_callback, this, std::placeholders::_1));

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
    keyboard_thread_ = std::thread(&Inverse3Node::keyboard_input, this);
  }

  ~Inverse3Node()
  {
    if (control_thread_.joinable())
      control_thread_.join();
    if (keyboard_thread_.joinable())
      keyboard_thread_.join();
  }

private:
  void setup_device()
  {
    auto inv3_ports = API::Devices::DeviceDetection::DetectInverse3s();
    auto vp_ports = API::Devices::DeviceDetection::DetectHandles();

    if (inv3_ports.empty() || vp_ports.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Check usb ports.");
      rclcpp::shutdown();
      return;
    }

    try
    {
      inv3_serial_ = std::make_shared<API::IO::SerialStream>(inv3_ports[0].c_str());
      vg_serial_ = std::make_shared<API::IO::SerialStream>(vp_ports[0].c_str());

      if (vg_serial_->OpenDevice() < 0)
        throw std::runtime_error("Failed to open VerseGrip serial.");

      versegrip_ = std::make_shared<API::Devices::Handle>(vg_serial_.get(), 10.0f);
      versegrip_->SendDeviceWakeup();
      versegrip_->Receive();

      if (inv3_serial_->OpenDevice() < 0)
        throw std::runtime_error("Failed to open Inverse3 serial.");

      inverse3_ = std::make_shared<API::Devices::Inverse3>(inv3_serial_.get(), 10.0f);
      auto inv3_response = inverse3_->DeviceWakeup();
      RCLCPP_INFO(this->get_logger(), "Connected to Inverse3 ID: %d", inv3_response.device_id);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Setup error: %s", e.what());
      rclcpp::shutdown();
    }

    set_gravity_compensation(gravity_compensation_);
  }

  void control_loop()
  {
    typedef std::chrono::high_resolution_clock clock;
    auto next = clock::now();
    auto delay = std::chrono::milliseconds(1);

    API::Devices::Inverse3::EndEffectorStateResponse inv3_res;
    Eigen::Matrix3d R_to_world = load_cal_file();
    while (rclcpp::ok())
    {
      API::Devices::Inverse3::EndEffectorForceRequest request;

      if (force_lock_)
        force_lock_inv3(inv3_res, request);

      {
        std::lock_guard<std::mutex> lock(inv3_mutex_);
        try
        {
          inverse3_->SendEndEffectorForce(request.force);
          int result = inverse3_->ReceiveEndEffectorState(inv3_res.position, inv3_res.velocity);
          if (result < 0)
          {
            RCLCPP_WARN(this->get_logger(), "Inverse3 receive state failed.");
            continue;
          }
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(this->get_logger(), "Inverse3 error: %s", e.what());
          continue;
        }
      }

      auto vg_res = versegrip_->GetVersegripStatus();
      Eigen::Vector3d pos(inv3_res.position[0], inv3_res.position[1], inv3_res.position[2]);
      Eigen::Vector3d cal_pos = pos.transpose() * R_to_world.transpose();

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = "inv3_cal_frame";
      pose_msg.pose.position.x = cal_pos.x();
      pose_msg.pose.position.y = cal_pos.y();
      pose_msg.pose.position.z = cal_pos.z();
      pose_publisher_->publish(pose_msg);

      sensor_msgs::msg::Joy btn_msg;
      btn_msg.buttons.resize(2, 0);
      btn_msg.buttons[0] = (vg_res.buttons & 0x01) ? 1 : 0;
      btn_msg.buttons[1] = (vg_res.buttons & 0x02) ? 1 : 0;
      btn_publisher_->publish(btn_msg);

      next += delay;
      while (next > clock::now())
        ;
    }
  }

  void keyboard_input()
  {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (rclcpp::ok())
    {
      char input;
      if (read(STDIN_FILENO, &input, 1) > 0)
      {
        if (input == 'g')
        {
          gravity_compensation_ = !gravity_compensation_;
          RCLCPP_INFO(this->get_logger(), "%s gravity compensation", gravity_compensation_ ? "Enabling" : "Disabling");
          try
          {
            set_gravity_compensation(gravity_compensation_);
          }
          catch (const std::exception &e)
          {
            std::cerr << e.what() << '\n';
          }
        }
      }
      std::this_thread::sleep_for(100ms);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

  void force_lock_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    force_lock_ = msg->data;
  }
  void force_lock_inv3(const API::Devices::Inverse3::EndEffectorStateResponse &pos,
                       API::Devices::Inverse3::EndEffectorForceRequest &req)
  {
    std::lock_guard<std::mutex> lock(inv3_mutex_);

    float lock_center[3] = {0.03f, -0.16f, 0.2f};
    float max_Kp = 80.0f;       // Stiff gain
    float min_Kp = 5.0f;        // Soft gain
    float Kd = 0.002f;          // Damping
    float max_distance = 0.03f; // 5 cm threshold

    for (int i = 0; i < 3; ++i)
    {
      float distance = std::abs(lock_center[i] - pos.position[i]);
      float Kp = (distance <= max_distance) ? max_Kp : min_Kp;

      float error = lock_center[i] - pos.position[i];
      req.force[i] = Kp * error - Kd * pos.velocity[i];
    }
  }

  void set_gravity_compensation(bool enable)
  {
    std::lock_guard<std::mutex> lock(inv3_mutex_);
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

  void gc_toggle_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data == gravity_compensation_)
    {
      return;
    }

    gravity_compensation_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "%s gravity compensation (via topic)",
                gravity_compensation_ ? "Enabling" : "Disabling");

    try
    {
      set_gravity_compensation(gravity_compensation_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Gravity set error (topic): %s", e.what());
    }
  }

  Eigen::Matrix3d load_cal_file()
  {
    Eigen::Matrix3d matrix;
    std::string package_share = ament_index_cpp::get_package_share_directory("hd_cpp");
    std::string file_path = package_share + "/device_cal/inverse3_cali_param.json";
    std::ifstream file(file_path);
    if (!file.is_open())
      throw std::runtime_error("Failed to open calibration file: " + file_path);

    json j;
    file >> j;
    for (int i = 0; i < 3; ++i)
      for (int k = 0; k < 3; ++k)
        matrix(i, k) = j.at(i).at(k).get<double>();
    return matrix;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr btn_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr force_lock_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gc_toggle_subscriber_;
  bool force_lock_{false};
  bool gravity_compensation_{true};
  std::thread control_thread_;
  std::thread keyboard_thread_;
  std::mutex inv3_mutex_;

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

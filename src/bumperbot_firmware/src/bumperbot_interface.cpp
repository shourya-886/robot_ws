#include "bumperbot_firmware/bumperbot_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sstream>
#include <iomanip>
#include <cmath>

namespace bumperbot_firmware
{

BumperbotInterface::BumperbotInterface()
{
}

BumperbotInterface::~BumperbotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(
        rclcpp::get_logger("BumperbotInterface"),
        "Something went wrong while closing connection with port "
          << port_);
    }
  }
}

CallbackReturn BumperbotInterface::on_init(
  const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result =
    hardware_interface::SystemInterface::on_init(hardware_info);

  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("BumperbotInterface"),
      "No Serial Port provided! Aborting");

    return CallbackReturn::FAILURE;
  }

  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);

  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BumperbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &position_states_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BumperbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &velocity_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn BumperbotInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("BumperbotInterface"),
    "Starting robot hardware interfacing over UART ...");

  velocity_commands_ = {0.0, 0.0};
  position_states_ = {0.0, 0.0};
  velocity_states_ = {0.0, 0.0};

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    
    // Explicit hardware configuration for raw UART header pins
    arduino_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    arduino_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    arduino_.SetParity(LibSerial::Parity::PARITY_NONE);
    arduino_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(
      rclcpp::get_logger("BumperbotInterface"),
      "Something went wrong while interacting with UART port "
        << port_);

    return CallbackReturn::FAILURE;
  }

  // Create ROS node
  node_ = rclcpp::Node::make_shared(
    "bumperbot_hardware_interface");

  // Create publishers
  jetson_write_pub_ =
    node_->create_publisher<std_msgs::msg::String>(
      "jetson_write", 10);

  jetson_read_pub_ =
    node_->create_publisher<std_msgs::msg::String>(
      "jetson_read", 10);

  RCLCPP_INFO(
    rclcpp::get_logger("BumperbotInterface"),
    "Hardware UART interface started, ready to take commands");

  return CallbackReturn::SUCCESS;
}

CallbackReturn BumperbotInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("BumperbotInterface"),
    "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(
        rclcpp::get_logger("BumperbotInterface"),
        "Something went wrong while closing connection with port "
          << port_);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BumperbotInterface"),
    "Hardware stopped");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
BumperbotInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (arduino_.IsDataAvailable())
  {
    auto dt =
      (rclcpp::Clock().now() - last_run_).seconds();

    std::string message;

    arduino_.ReadLine(message);

    // Publish incoming serial data
    if (jetson_read_pub_)
    {
      std_msgs::msg::String msg;
      msg.data = message;
      jetson_read_pub_->publish(msg);
    }

    std::stringstream ss(message);
    std::string res;
    int multiplier = 1;

    while (std::getline(ss, res, ','))
    {
      if (res.size() < 3)
      {
        continue;
      }

      multiplier =
        (res.at(1) == 'p') ? 1 : -1;

      // Joint order in bumperbot_ros2_control.xacro is:
      //   index 0 = wheel_left_joint
      //   index 1 = wheel_right_joint
      // The UART protocol uses 'l' and 'r', so keep that mapping explicit.
      if (res.at(0) == 'l')
      {
        velocity_states_.at(0) =
          multiplier *
          std::stod(res.substr(2));

        position_states_.at(0) +=
          velocity_states_.at(0) * dt;
      }
      else if (res.at(0) == 'r')
      {
        velocity_states_.at(1) =
          multiplier *
          std::stod(res.substr(2));

        position_states_.at(1) +=
          velocity_states_.at(1) * dt;
      }
    }

    last_run_ = rclcpp::Clock().now();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
BumperbotInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  std::stringstream message_stream;

  // Joint order in bumperbot_ros2_control.xacro is:
  //   index 0 = wheel_left_joint
  //   index 1 = wheel_right_joint
  // The UART protocol expects the right-wheel command first and the
  // left-wheel command second, so map the ROS joint indices explicitly.
  char right_wheel_sign =
    velocity_commands_.at(1) >= 0 ? 'p' : 'n';

  char left_wheel_sign =
    velocity_commands_.at(0) >= 0 ? 'p' : 'n';

  std::string compensate_zeros_right = "";
  std::string compensate_zeros_left = "";

  if (std::abs(velocity_commands_.at(1)) < 10.0)
  {
    compensate_zeros_right = "0";
  }

  if (std::abs(velocity_commands_.at(0)) < 10.0)
  {
    compensate_zeros_left = "0";
  }

  message_stream
    << std::fixed
    << std::setprecision(2)
    << "r"
    << right_wheel_sign
    << compensate_zeros_right
    << std::abs(velocity_commands_.at(1))
    << ",l"
    << left_wheel_sign
    << compensate_zeros_left
    << std::abs(velocity_commands_.at(0))
    << ",\n";

  try
  {
    arduino_.Write(message_stream.str());

    // Publish outgoing serial data for tracking
    if (jetson_write_pub_)
    {
      std_msgs::msg::String msg;
      msg.data = message_stream.str();
      jetson_write_pub_->publish(msg);
    }
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("BumperbotInterface"),
      "Something went wrong while sending the message to the port "
        << port_);

    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace bumperbot_firmware

PLUGINLIB_EXPORT_CLASS(
  bumperbot_firmware::BumperbotInterface,
  hardware_interface::SystemInterface)
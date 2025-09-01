// HPC_OUTPUT (Odometry, Trajectory) -> converter -> BOARD_INPUT (VehicleKinematicState, Trajectory)
// HPC_INPUT (AckermannControlCommand) <- converter <- BOARD_OUTPUT (VehicleControlCommand)

#ifndef ACTUATION_MESSAGE_CONVERTER__ACTUATION_MESSAGE_CONVERTER_NODE_HPP_
#define ACTUATION_MESSAGE_CONVERTER__ACTUATION_MESSAGE_CONVERTER_NODE_HPP_

#include <dds/dds.h>

#include <actuation_message_converter/visibility_control.hpp>

#include "rt_motion_planning_hpc_msgs/msg/hpc_input_data.hpp"
#include "rt_motion_planning_hpc_msgs/msg/hpc_output_data.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace actuation_message_converter
{

// using autoware_auto_planning_msgs::msg::Trajectory;
// using autoware_auto_control_msgs::msg::AckermannControlCommand;
// using nav_msgs::msg::Odometry;
using rt_motion_planning_hpc_msgs::msg::HpcInputData;
using rt_motion_planning_hpc_msgs::msg::HpcOutputData;


/// \class MessageConverterNode
/// \brief Converts messages between ROSIDL and IDLC.
class ACTUATION_MESSAGE_CONVERTER_PUBLIC MessageConverterNode : public rclcpp::Node
{
public:
  /// \brief constructor
  /// \param options Node options for this node.
  explicit MessageConverterNode(const rclcpp::NodeOptions & options);

  /// \brief destructor
  ~MessageConverterNode();

  // Domain ID used for Safety Island communication.
  static constexpr dds_domainid_t DDS_DOMAIN_ACTUATION {2};

private:
  // const rclcpp::Subscription<Odometry>::SharedPtr m_state_sub;
  // const rclcpp::Subscription<Trajectory>::SharedPtr m_trajectory_sub;
  const rclcpp::Subscription<HpcOutputData>::SharedPtr m_hpc_output_data_sub;

  // const rclcpp::Publisher<AckermannControlCommand>::SharedPtr m_command_pub;
  const rclcpp::Publisher<HpcInputData>::SharedPtr m_hpc_input_data_pub;

  dds_listener_t * m_listener {};
  
  // dds_entity_t m_state_writer {};
  // dds_entity_t m_trajectory_writer {};
  dds_entity_t m_board_input_data_writer {};

  // dds_entity_t m_command_reader {};
  dds_entity_t m_board_output_data_reader {};

  // std::function<void(const AckermannControlCommand &)> m_publish {};
  std::function<void(const HpcInputData &)> m_publish {};

  /// \brief Callback handler converting Odometry rosidl messages to VehicleKinematicState idlc
  ///        messages and sending them with a CycloneDDS writer.
  /// \param rosidl_msg Message to be converted.
  // void on_state_callback(Odometry::SharedPtr rosidl_msg);
  void on_hpc_output_data_callback(HpcOutputData::SharedPtr rosidl_msg);

  /// \brief Callback handler converting Trajectory rosidl messages to idlc and sending them with a
  ///        CycloneDDS writer.
  /// \param rosidl_msg Message to be converted.
  // void on_trajectory_callback(Trajectory::SharedPtr rosidl_msg);
};
}  // namespace actuation_message_converter
}  // namespace autoware

#endif  // ACTUATION_MESSAGE_CONVERTER__ACTUATION_MESSAGE_CONVERTER_NODE_HPP_

// HPC_OUTPUT (Odometry, Trajectory) -> converter -> BOARD_INPUT (VehicleKinematicState, Trajectory)
// HPC_INPUT (AckermannControlCommand) <- converter <- BOARD_OUTPUT (VehicleControlCommand)

#include <actuation_message_converter/actuation_message_converter_node.hpp>
// #include <Trajectory.h>
// #include <VehicleControlCommand.h>
// #include <VehicleKinematicState.h>
#include "board_input_data.h"
#include "board_output_data.h"

#include <algorithm>
#include <functional>
#include <vector>

namespace autoware
{
namespace actuation_message_converter
{

static void on_board_output_data_callback(dds_entity_t rd, void * arg)
{
  if (!arg) {
    return;
  }
  // auto publish = *reinterpret_cast<std::function<void(const AckermannControlCommand &)> *>(arg);
  auto publish = *reinterpret_cast<std::function<void(const HpcInputData &)> *>(arg);

  // autoware_auto_vehicle_msgs_msg_VehicleControlCommand idlc_msg {};
  board_output_data_msg idlc_msg {};

  void * idlc_msg_p = &idlc_msg;
  dds_sample_info_t info;

  // The return value contains the number of samples read.
  dds_return_t rc = dds_take(rd, &idlc_msg_p, &info, 1, 1);
  if (rc < 0) {
    dds_log(DDS_LC_WARNING, __FILE__, __LINE__, DDS_FUNCTION, "can't take msg\n");
  }

  // Check if we have read some data and if it is valid.
  if (rc > 0 && info.valid_data) {

    HpcInputData rosidl_msg {};

    rosidl_msg.x.assign(idlc_msg.x._buffer, idlc_msg.x._buffer + idlc_msg.x._length);
    rosidl_msg.y.assign(idlc_msg.y._buffer, idlc_msg.y._buffer + idlc_msg.y._length);

    rosidl_msg.feasibility = idlc_msg.feasibility;
    rosidl_msg.cost = idlc_msg.cost;

    publish(rosidl_msg);
  }
}

MessageConverterNode::MessageConverterNode(const rclcpp::NodeOptions & options)
: Node("actuation_message_converter", options),
  m_hpc_output_data_sub(create_subscription<HpcOutputData>(
      "/hpc_output_data", rclcpp::QoS{1},
      std::bind(&MessageConverterNode::on_hpc_output_data_callback, this, std::placeholders::_1))),
  m_hpc_input_data_pub(create_publisher<HpcInputData>(
      "/hpc_input_data", rclcpp::QoS{1}.transient_local())) 
  {
  // Create a Participant for all DDS usage.
  dds_entity_t participant = dds_create_participant(DDS_DOMAIN_ACTUATION, NULL, NULL);
  if (participant < 0) {
    fprintf(stderr, "dds_create_participant: %s\n", dds_strretcode(-participant));
    std::exit(EXIT_FAILURE);
  }

  // Create a Topic for VehicleControlCommand messages.
  dds_entity_t board_output_data_topic = dds_create_topic(
    participant,
    // &autoware_auto_vehicle_msgs_msg_VehicleControlCommand_desc,
    &board_output_data_msg_desc,
    // "ctrl_cmd",
    "board_output_data_msg", //topic name
    NULL,
    NULL);
  if (board_output_data_topic < 0) {
    fprintf(stderr, "dds_create_topic: %s\n", dds_strretcode(-board_output_data_topic));
    std::exit(EXIT_FAILURE);
  }

  // Create a listener.
  // m_publish = [&](const AckermannControlCommand & msg) {m_command_pub->publish(msg);};
  m_publish = [&](const HpcInputData & msg) {m_hpc_input_data_pub->publish(msg);};
  m_listener = dds_create_listener(reinterpret_cast<void *>(&m_publish));
  dds_lset_data_available(m_listener, on_board_output_data_callback);

  // Create a reliable QoS.
  dds_qos_t * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));

  // Create a Reader for board output data messages.
  m_board_output_data_reader = dds_create_reader(participant, board_output_data_topic, qos, m_listener);
  if (m_board_output_data_reader < 0) {
    dds_delete_qos(qos);
    fprintf(stderr, "dds_create_reader: %s\n", dds_strretcode(-m_board_output_data_reader));
    std::exit(EXIT_FAILURE);
  }
  dds_delete_qos(qos);

  // Create a Topic for VehicleKinematicState messages.
  dds_entity_t board_input_data_topic = dds_create_topic(
    participant,
    &board_input_data_msg_desc,
    "board_input_data_msg",
    NULL,
    NULL);
  if (board_input_data_topic < 0) {
    fprintf(stderr, "dds_create_topic: %s\n", dds_strretcode(-board_input_data_topic));
    std::exit(EXIT_FAILURE);
  }

  // Create a Writer for VehicleKinematicState messages.
  m_board_input_data_writer = dds_create_writer(participant, board_input_data_topic, NULL, NULL);
  if (m_board_input_data_writer < 0) {
    fprintf(stderr, "dds_create_writer: %s\n", dds_strretcode(-m_board_input_data_writer));
    std::exit(EXIT_FAILURE);
  }
}

MessageConverterNode::~MessageConverterNode()
{
  dds_delete_listener(m_listener);
}

static void copy(
  const HpcOutputData & src,
  board_input_data_msg & dst)
  // autoware_auto_vehicle_msgs_msg_VehicleKinematicState & dst)
{
  dst.s = src.s;                   
  dst.ss = src.ss;                   
  dst.sss = src.sss;               
  dst.d = src.d;            
  dst.dd = src.dd;        
  dst.ddd = src.ddd;
  dst.velocity = src.velocity;
  dst.timestep = src.timestep;
  dst.orientation = src.orientation;
  dst.desired_velocity = src.desired_velocity;
}

void MessageConverterNode::on_hpc_output_data_callback(const HpcOutputData::SharedPtr rosidl_msg)
{
  board_input_data_msg idlc_msg {};
  copy(*rosidl_msg, idlc_msg);
  dds_return_t rc = dds_write(m_board_input_data_writer, &idlc_msg);
  if (rc < 0) {
    RCLCPP_ERROR(get_logger(), "dds_write: %s", dds_strretcode(-rc));
  }
}

}  // namespace actuation_message_converter
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::actuation_message_converter::MessageConverterNode)

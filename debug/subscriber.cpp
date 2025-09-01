#include <dds/dds.h>
#include <iostream>
#include <cstdlib>
#include <cstring>

#include "msg/board_input_data.h"
#include "msg/board_output_data.h"  // new header for publishing

#include <Eigen/Dense>

// inline dds_sequence_double vectorToDdsSequence(const Eigen::VectorXd& vec)
// {
//     dds_sequence_double seq;
//     seq._maximum = static_cast<uint32_t>(vec.size());
//     seq._length  = static_cast<uint32_t>(vec.size());
//     seq._buffer  = static_cast<double*>(dds_alloc(seq._maximum * sizeof(double)));
//     seq._release = true;
//     std::memcpy(seq._buffer, vec.data(), seq._length * sizeof(double));
//     return seq;
// }

inline void fillDdsSequence(dds_sequence_double& dst, const Eigen::VectorXd& vec)
{
  const uint32_t need = static_cast<uint32_t>(vec.size());
  if (dst._maximum < need || dst._buffer == nullptr) {
    if (dst._buffer && dst._release) dds_free(dst._buffer);
    dst._buffer  = static_cast<double*>(dds_alloc(need * sizeof(double)));
    dst._maximum = need;
    dst._release = true;
  }
  dst._length = need;
  std::memcpy(dst._buffer, vec.data(), need * sizeof(double));
}

int main() {
    // Create participant
    dds_entity_t participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (participant < 0) {
        std::cerr << "Failed to create participant: " << dds_strretcode(-participant) << "\n";
        return EXIT_FAILURE;
    }

    // === Reader for board_input_data_msg ===
    dds_entity_t input_topic = dds_create_topic(
        participant,
        &board_input_data_msg_desc,
        "board_input_data_msg",
        NULL,
        NULL
    );
    if (input_topic < 0) {
        std::cerr << "Failed to create input topic: " << dds_strretcode(-input_topic) << "\n";
        return EXIT_FAILURE;
    }

    dds_entity_t reader = dds_create_reader(participant, input_topic, NULL, NULL);
    if (reader < 0) {
        std::cerr << "Failed to create reader: " << dds_strretcode(-reader) << "\n";
        return EXIT_FAILURE;
    }

    // === Writer for board_output_data_msg ===
    dds_entity_t output_topic = dds_create_topic(
        participant,
        &board_output_data_msg_desc,
        "board_output_data_msg",
        NULL,
        NULL
    );
    if (output_topic < 0) {
        std::cerr << "Failed to create output topic: " << dds_strretcode(-output_topic) << "\n";
        return EXIT_FAILURE;
    }

    dds_entity_t writer = dds_create_writer(participant, output_topic, NULL, NULL);
    if (writer < 0) {
        std::cerr << "Failed to create writer: " << dds_strretcode(-writer) << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Subscribed to 'board_input_data_msg' and publishing to 'board_output_data_msg'...\n";

    board_input_data_msg msg{};
    void* samples[1];
    dds_sample_info_t info;
    samples[0] = &msg;

    // ============== Prepare static board_output_data_msg =============================
    board_output_data_msg output_msg{};
    
    // Prepare example Eigen vectors
    Eigen::VectorXd x_vec(3);
    x_vec << 1.0, 2.0, 3.0;

    Eigen::VectorXd y_vec(3);
    y_vec << 4.0, 5.0, 6.0;

    fillDdsSequence(output_msg.x, x_vec);
    fillDdsSequence(output_msg.y, y_vec);
    output_msg.feasibility = true;
    output_msg.cost = 0.42;
    // ============== Prepare static board_output_data_msg =============================

    // ====================== Allocate and assign sequence values ===========================
    // size_t len = 3;
    // output_msg.x._length = len;
    // output_msg.x._maximum = len;
    // output_msg.x._release = true;
    // output_msg.x._buffer = static_cast<double *>(malloc(len * sizeof(double)));
    // output_msg.x._buffer[0] = 1.0;
    // output_msg.x._buffer[1] = 2.0;
    // output_msg.x._buffer[2] = 3.0;

    // output_msg.y._length = len;
    // output_msg.y._maximum = len;
    // output_msg.y._release = true;
    // output_msg.y._buffer = static_cast<double *>(malloc(len * sizeof(double)));
    // output_msg.y._buffer[0] = 4.0;
    // output_msg.y._buffer[1] = 5.0;
    // output_msg.y._buffer[2] = 6.0;

    // output_msg.feasibility = true;
    // output_msg.cost = 0.42;
    // ====================== Allocate and assign sequence values ===========================

    // === Main loop ===
    while (true) {
        int rc = dds_take(reader, samples, &info, 1, 1);
        if (rc < 0) {
            std::cerr << "dds_take error: " << dds_strretcode(-rc) << "\n";
            continue;
        }

        if (rc > 0 && info.valid_data) {
            std::cout << "Received board_input_data_msg:\n";
            std::cout << "  s: " << msg.s << "\n";
            std::cout << "  ss: " << msg.ss << "\n";
            std::cout << "  sss: " << msg.sss << "\n";
            std::cout << "  d: " << msg.d << "\n";
            std::cout << "  dd: " << msg.dd << "\n";
            std::cout << "  ddd: " << msg.ddd << "\n";
            std::cout << "  velocity: " << msg.velocity << "\n";
            std::cout << "  timestep: " << msg.timestep << "\n";
            std::cout << "  orientation: " << msg.orientation << "\n";
            std::cout << "  desired_velocity: " << msg.desired_velocity << "\n";
            std::cout << "----------------------------\n";

            // === Publish the static message
            dds_return_t wrc = dds_write(writer, &output_msg);
            if (wrc < 0) {
                std::cerr << "dds_write error: " << dds_strretcode(-wrc) << "\n";
            } else {
                std::cout << "Published static board_output_data_msg.\n";
            }
        }

        dds_sleepfor(DDS_MSECS(500)); // avoid CPU spinning
    }

    // === Cleanup
    free(output_msg.x._buffer);
    free(output_msg.y._buffer);
    dds_delete(participant);

    return 0;
}

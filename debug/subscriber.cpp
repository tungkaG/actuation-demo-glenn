#include <dds/dds.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <random>

#include "msg/board_input_data.h"
#include "msg/board_output_data.h"  // new header for publishing
#include "msg/cartesian_sample.h"
#include "msg/curvilinear_sample.h"

#include <Eigen/Dense>

// === Constants ===
constexpr uint32_t N = 3; // Number of trajectory samples

// === Sample data structures ===
struct CartesianSample {
  Eigen::VectorXd x, y, theta, velocity, acceleration, kappa, kappaDot;
};

struct CurviLinearSample {
  Eigen::VectorXd s, ss, sss, d, dd, ddd;
};

struct Trajectory {
  CartesianSample m_cartesianSample;
  CurviLinearSample m_curvilinearSample;
  bool m_feasible;
  double m_cost;
};

struct TrajectoryHandler {
  std::vector<Trajectory> m_trajectories;

  TrajectoryHandler() {
    std::mt19937 gen(42); // fixed seed
    std::uniform_real_distribution<double> dist(-10.0, 10.0);

    for (uint32_t i = 0; i < N; ++i) {
      Trajectory traj;

      // Fill CartesianSample with 5 points each
      traj.m_cartesianSample.x = Eigen::VectorXd::LinSpaced(5, i, i + 4);
      traj.m_cartesianSample.y = Eigen::VectorXd::LinSpaced(5, i + 10, i + 14);
      traj.m_cartesianSample.theta = Eigen::VectorXd::Constant(5, 0.1 * i);
      traj.m_cartesianSample.velocity = Eigen::VectorXd::Constant(5, 1.0 + i);
      traj.m_cartesianSample.acceleration = Eigen::VectorXd::Constant(5, 0.2 * i);
      traj.m_cartesianSample.kappa = Eigen::VectorXd::Constant(5, 0.05 * i);
      traj.m_cartesianSample.kappaDot = Eigen::VectorXd::Constant(5, 0.01 * i);

      // Fill CurviLinearSample with 5 points each
      traj.m_curvilinearSample.s = Eigen::VectorXd::LinSpaced(5, i, i + 4);
      traj.m_curvilinearSample.ss = Eigen::VectorXd::Constant(5, 0.2 * i);
      traj.m_curvilinearSample.sss = Eigen::VectorXd::Constant(5, 0.05 * i);
      traj.m_curvilinearSample.d = Eigen::VectorXd::Constant(5, -0.1 * i);
      traj.m_curvilinearSample.dd = Eigen::VectorXd::Constant(5, 0.01 * i);
      traj.m_curvilinearSample.ddd = Eigen::VectorXd::Constant(5, 0.005 * i);

      traj.m_feasible = (i % 2 == 0); // alternate true/false
      traj.m_cost = dist(gen);

      m_trajectories.push_back(traj);
    }
  }
};

TrajectoryHandler trajectory_handler;

// generic grow helper, tells you if it allocated a new buffer
static inline void ensure_seq_capacity(void** buf, uint32_t* maximum, bool* release,
                                       uint32_t need, size_t elem_size,
                                       bool* newly_allocated) {
  *newly_allocated = false;
  if (*maximum < need || *buf == nullptr) {
    if (*buf && *release) dds_free(*buf);
    *buf = dds_alloc(need * elem_size);
    *maximum = need;
    *release = true;
    *newly_allocated = true;
  }
}

static inline void fill_seq_double(dds_sequence_double* dst, const Eigen::VectorXd& vec) {
  const uint32_t n = static_cast<uint32_t>(vec.size());
  bool newbuf = false;
  ensure_seq_capacity((void**)&dst->_buffer, &dst->_maximum, &dst->_release,
                      n, sizeof(double), &newbuf);
  dst->_length = n;
  if (n) std::memcpy(dst->_buffer, vec.data(), n * sizeof(double));
}

static inline void fill_cartesian_sample(trajectory_data_cartesian_sample* out,
                                         const CartesianSample& in) {
  fill_seq_double(&out->x,            in.x);
  fill_seq_double(&out->y,            in.y);
  fill_seq_double(&out->theta,        in.theta);
  fill_seq_double(&out->velocity,     in.velocity);
  fill_seq_double(&out->acceleration, in.acceleration);
  fill_seq_double(&out->kappa,        in.kappa);
  fill_seq_double(&out->kappaDot,     in.kappaDot);
}

static inline void fill_curvilinear_sample(trajectory_data_curvilinear_sample* out,
                                         const CurviLinearSample& in) {
  fill_seq_double(&out->s,            in.s);
  fill_seq_double(&out->ss,           in.ss);
  fill_seq_double(&out->sss,          in.sss);
  fill_seq_double(&out->d,            in.d);
  fill_seq_double(&out->dd,           in.dd);
  fill_seq_double(&out->ddd,          in.ddd);
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
    static board_output_data_msg output_msg{};

    static bool called_once = false;
    if (!called_once) {
        // allocate arrays once
        output_msg.samples._buffer =
            static_cast<trajectory_data_cartesian_sample*>(dds_alloc(N * sizeof(trajectory_data_cartesian_sample)));
        std::memset(output_msg.samples._buffer, 0, N *  sizeof(trajectory_data_cartesian_sample));
        output_msg.samples._maximum = N;
        output_msg.samples._length  = N;
        output_msg.samples._release = true;

        output_msg.samples_curv._buffer =
            static_cast<trajectory_data_curvilinear_sample*>(dds_alloc(N * sizeof(trajectory_data_curvilinear_sample)));
        std::memset(output_msg.samples_curv._buffer, 0, N *  sizeof(trajectory_data_curvilinear_sample));
        output_msg.samples_curv._maximum = N;
        output_msg.samples_curv._length  = N;
        output_msg.samples_curv._release = true;

        output_msg.feasibility._buffer =
            static_cast<bool*>(dds_alloc(N * sizeof(bool)));
        output_msg.feasibility._maximum = N;
        output_msg.feasibility._length  = N;
        output_msg.feasibility._release = true;

        output_msg.cost._buffer =
            static_cast<double*>(dds_alloc(N * sizeof(double)));
        output_msg.cost._maximum = N;
        output_msg.cost._length  = N;
        output_msg.cost._release = true;

        called_once = true;
    }

    // overwrite each element
    for (uint32_t i = 0; i < N; ++i) {
        const auto& t = trajectory_handler.m_trajectories[i];

        trajectory_data_cartesian_sample* s_cart = &output_msg.samples._buffer[i];
        fill_cartesian_sample(s_cart, t.m_cartesianSample);

        trajectory_data_curvilinear_sample* s_curv = &output_msg.samples_curv._buffer[i];
        fill_curvilinear_sample(s_curv, t.m_curvilinearSample);

        output_msg.feasibility._buffer[i] = t.m_feasible;
        output_msg.cost._buffer[i]        = t.m_cost;
    }
    // ============== Prepare static board_output_data_msg =============================

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
    dds_delete(participant);

    return 0;
}

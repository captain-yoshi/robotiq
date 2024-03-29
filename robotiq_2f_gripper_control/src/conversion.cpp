#include <robotiq_2f_gripper_control/conversion.h>

namespace Robotiq2f140 {

// calculates the x-axis tcp frame offset off the gripper when fully closed
// frame convention : https://www.ros.org/reps/rep-0103.html
double compute_tcp_frame_offset(double gap_d) {
  double a = gap_d / 2.0 + 0.00735 - 0.0127;
  double c = 0.1;
  double b = sqrt(c * c - a * a);

  double offset = sqrt(c * c - 0.00535 * 0.00535) - b;

  return offset; // ros frame convention
}

/// Gripper gap between 2 fingers to position in joint space
/// (left_outer_knucle_joint)
double gap_to_joint_pos(double gap_d) {
  double theta_offset = acos(0.06691 / 0.1);
  double j_pos = acos((gap_d / 2.0 + 0.00735 - 0.0127) / 0.1) - theta_offset;

  return j_pos;
}

double joint_pos_to_gap(double j_pos) {
  double theta_offset = acos(0.06691 / 0.1);

  double gap_d = 2 * (0.1 * cos(j_pos + theta_offset) - (0.00735 - 0.0127));

  return gap_d;
}
} // namespace Robotiq2f140

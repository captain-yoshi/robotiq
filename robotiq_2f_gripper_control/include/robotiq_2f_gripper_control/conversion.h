#ifndef CONVERSION_H_
#define CONVERSION_H_

#include <cmath>

namespace Robotiq2f140 {

// calculates the x-axis tcp frame offset off the gripper when fully closed
// frame convention : https://www.ros.org/reps/rep-0103.html
double compute_tcp_frame_offset(double gap_d);

/// Gripper gap between 2 fingers to position in joint space
/// (left_outer_knucle_joint)
double gap_to_joint_pos(double gap_d);

double joint_pos_to_gap(double j_pos);

} // namespace Robotiq2f140

#endif // CONVERSION_H_

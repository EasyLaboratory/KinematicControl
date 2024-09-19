#include "data_frame.hpp"
#include "math.h"

namespace Planning {
class Control {
 public:
  Control() {}
  ~Control() {}

  double calDistance(const Postion& ego_postion, const Postion& target_postion);
  void normalizeAngle(double* yaw);
  void updateState(State* state);
  void setDisMin(double dis_min) { dis_min_ = dis_min; }
  void setDisMax(double dis_max) { dis_max_ = dis_max; }
  double dismin() { return dis_min_; }
  double dismax() { return dis_max_; }
  void KinematicControl();
  void updateState();

 private:
  double dt_ = 0.2;
  double dis_min_ = 0.0;
  double dis_max_ = 0.0;
  State ego_state_;
  State target_state_;
  double k1_ = 1.0;
  double k2_ = 1.0;
};
}  // namespace Planning

struct Postion {
  Postion(const double& x_, const double& y_, const double& z_) {
    x = x_;
    y = y_;
    z = z_;
  }
  Postion() {}
  double x;
  double y;
  double z;
};

struct Vel {
  Vel(const double& vx_, const double& vy_, const double& vz_) {
    vx = vx_;
    vy = vy_;
    vz = vz_;
  }
  Vel() {}
  double vx;
  double vy;
  double vz;
};

class State {
 public:
  State(const Postion& pos, const Vel& vel, const double& yaw,
        const double& omega)
      : pos_(pos), vel_(vel), yaw_(yaw), omega_(omega) {}
  State() {}
  ~State() {}

  Postion pos() const { return pos_; }
  Vel vel() const { return vel_; }
  double yaw() const { return yaw_; }
  double omega() const { return omega_; }

  void setPos(const Postion& pos) { pos_ = pos; }
  void setVel(const Vel& vel) { vel_ = vel; }
  void setYaw(double yaw) { yaw_ = yaw; }
  void setOmega(double omega) { omega_ = omega; }

 private:
  Postion pos_;
  Vel vel_;
  double yaw_ = 0;
  double omega_ = 0;
};
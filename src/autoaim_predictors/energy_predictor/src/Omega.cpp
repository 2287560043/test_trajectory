#include "Omega.hpp"

namespace helios_cv
{

/**
 * @brief 初始化
 */
Omega::Omega()
{
  refresh();
}

/**
 * @brief 找到波峰后的初始化
 */
void Omega::refresh_after_wave()
{
  fit_cnt_ = 0;
  change_cnt_ = 0;
  max_omega_ = 0;
  min_omega_ = 5;
  total_theta_ = 0;
}

/**
 * @brief 刷新数据
 */
void Omega::refresh()
{
  angle_.clear();
  filter_omega_.clear();
  time_series_.clear();
  t_list_.clear();
  clockwise_cnt_ = 0;
  energy_rotation_direction_ = 1;
  time_start_ = 0;
  time_gap_ = 0;
  a_ = 0.9125;
  w_ = 1.942;
  phi_ = CV_2PI / 2;
  b_ = 1.1775;
  st_ = 15;
  start_ = false;
  dt_ = 0.012;
  differ_step_ = 4;
  refresh_after_wave();
  // total_theta_ = 0;
}

/**
 * @brief 根据角速度的方差判断大小符
 */
int Omega::judge_mode(double& std_dev)
{
  if(not energy_mode_examed_)
    {

    if (energy_exam_cnt_ > energy_exam_num_)
    {
      std::vector<double> diffs;
      for (size_t i = filter_omega_.size() - energy_exam_num_; i < filter_omega_.size(); ++i) {
        diffs.push_back(filter_omega_[i] - filter_omega_[i-1]);
      }
      double sum = 0.0;
      for (double d : diffs) {sum += d;}
      double mean = sum / diffs.size();
      double variance = 0.0;
      for (double d : diffs) {
        variance += (d - mean) * (d - mean);
      }
      variance /= diffs.size();
      std_variance_.push_back(sqrt(variance));
      if(std_variance_.size() >= 30){
        std_dev = std::accumulate(std_variance_.begin(), std_variance_.end(), 0.0) / std_variance_.size();
        std_dev *= 100;   
        if (std_dev < 5) {
          //1.8-2.1
          std::cout << "Energy state: SMALL!" << " std_dev: " << std_dev << std::endl;
          return 1;
        }
        else {
          //7.6-9
          std::cout << "Energy state: BIG!" << " std_dev: " << std_dev << std::endl;
          return 2;
        }
      }
      return 0;
    }
  }
  else{
    std_variance_.clear();
    return 0;
  }
  return 0;
}

/**
 * @brief 设置角度， 满足条件的开始求解角速度
 * @param theta 两帧之间的角度
 */
void Omega::set_theta(double theta)
{
  angle_.push_back(theta);
  // RCLCPP_INFO(rclcpp::get_logger("roll"), "ro: %f", theta);
  if (angle_.size() > differ_step_)
  {
    start_ = true;
    last_current_theta_ = current_theta_;
    current_theta_ = calOmegaNstep(differ_step_, total_theta_);
    if (current_theta_ < 50.0)
    {
      JudgeFanRotation(current_theta_);
    }
  }

  change_time_series();
}

/**
 * @brief 根据传入的角度求解角速度
 * @param step 指定步长多少来平滑
 * @param total_theta 角度
 */
double Omega::calOmegaNstep(int step, double& total_theta)
{
  long unsigned int step_ = step + 1;
  if (angle_.size() < step_)
  {
    return 0;
  }

  double d_theta;
  double dt = t_list_.back() - t_list_[t_list_.size() - step_];

  double last_d_theta = angle_[angle_.size() - 1] - angle_[angle_.size() - 2];
  // RCLCPP_INFO(rclcpp::get_logger("lasttheta"), "l0: %f", last_d_theta);

  if (last_d_theta > 6.1)
  {
    last_d_theta -= CV_2PI;
  }
  else if (last_d_theta < -6.1)
  {
    last_d_theta += CV_2PI;
  }

  int d_fan = round2int(last_d_theta / 1.2566);

  if (abs(d_fan) >= 1)
  {
    // RCLCPP_INFO(rclcpp::get_logger("fan"), "change fan");
    for (long unsigned int i = 2; i <= step_; i++)
    {
      angle_[angle_.size() - i] += d_fan * 1.2566;
      if (angle_[angle_.size() - i] < -CV_2PI)
        angle_[angle_.size() - i] += CV_2PI;
      if (angle_[angle_.size() - i] > CV_2PI)
        angle_[angle_.size() - i] += -CV_2PI;
    }
  }

  d_theta = angle_[angle_.size() - 1] - angle_[angle_.size() - step_];
  total_theta += d_theta;

  if (d_theta > 6)
  {
    // RCLCPP_INFO(rclcpp::get_logger("error"), "s-1: %f", angle_[angle_.size() - 1]);
    // RCLCPP_INFO(rclcpp::get_logger("error"), "s-s: %f", angle_[angle_.size() - step_]);
    d_theta -= CV_2PI;
  }
  if (d_theta < -6)
  {
    // RCLCPP_INFO(rclcpp::get_logger("error"), "s-1: %f", angle_[angle_.size() - 1]);
    // RCLCPP_INFO(rclcpp::get_logger("error"), "s-s: %f", angle_[angle_.size() - step_]);
    d_theta += CV_2PI;
  }

  double tmp_omega = d_theta / dt;
  // RCLCPP_INFO(rclcpp::get_logger("norm dt"), "dt: %f", dt);
  // RCLCPP_INFO(rclcpp::get_logger("norm dtheta"), "d0: %f", d_theta);

  if (fabs(tmp_omega) > 2.1)
  {
    if (filter_omega_.size() != 0)
    {
      tmp_omega = filter_omega_.back() * energy_rotation_direction_;
    }
    else
    {
      tmp_omega = (tmp_omega > 0) ? 2.09 : -2.09;
    }
    // tmp_omega = 100.0;
    // RCLCPP_INFO(rclcpp::get_logger("error"), "s-1: %f", angle_[angle_.size() - 1]);
    // RCLCPP_INFO(rclcpp::get_logger("error"), "s-s: %f", angle_[angle_.size() - step_]);
    // RCLCPP_INFO(rclcpp::get_logger("dtheta"), "d0: %f", d_theta);
    // RCLCPP_INFO(rclcpp::get_logger("dt"), "dt: %f", dt);
  }

  return tmp_omega;
}

/**
 * @brief 四舍五入
 * @param f 输入的值
 * @return 四舍五入后的值
 */
int Omega::round2int(double f)
{
  int int_f = f;
  double d = f - int_f;
  if (d >= 0.5)
    return int_f;
  else if (d < 0 && d > -0.5)
    return int_f;
  else if (d > 0 && d < 0.5)
    return int_f;
  else
    return int_f - 1;
}

/**
 * @brief 顺逆时针
 */
void Omega::JudgeFanRotation(double omega)
{
  clockwise_cnt_ = (omega > 0) ? clockwise_cnt_ + 1 : clockwise_cnt_ - 1;
  energy_rotation_direction_ = (clockwise_cnt_ > 0) ? 1 : -1;
}

/**
 * @brief 设置时间
 * @param time_t 外部输入当前时间
 */
void Omega::set_time(double time_t)
{
  time_gap_ += (time_start_ == 0) ? 0.0 : time_t - time_start_;
  time_start_ = time_t;
  t_list_.push_back(time_gap_);
  dt_ = t_list_.back() - t_list_[t_list_.size() - 2];
}

/**
 * @brief 设置time_series
 */
void Omega::change_time_series()
{
  if (start_)
  {
    time_series_.push_back(t_list_[t_list_.size() - 1 - differ_step_ / 2]);
  }
}

/**
 * @brief 根据官方的公式求角速度
 */
double Omega::IdealOmega(double& t_)
{
  //return a_ * std::sin(w_ * t_ + phi_) + b_;
  return a_ * std::sin(w_ * t_ + phi_) + 2.09 - a_;
}

/**
 * @brief 根据公式积分求提前量
 */
double Omega::IdealRad(double t1, double t2)
{
  //return (-a_ / w_) * (std::cos(w_ * t2 + phi_) - std::cos(w_ * t1 + phi_)) + (b_) * (t2 - t1);
  return (-a_ / w_) * (std::cos(w_ * t2 + phi_) - std::cos(w_ * t1 + phi_)) + (2.09 - a_) * (t2 - t1);
}

void Omega::set_a_w_phi(double a, double w, double phi)
{
  a_ = a;
  w_ = w;
  phi_ = phi;
}

void Omega::set_a_w_phi_b(double a, double w, double phi, double b)
{
  a_ = a;
  w_ = w;
  phi_ = phi;
  b_ = b;
}

/**
 * @brief 获取预测角速度
 */
double Omega::get_omega()
{
  double wt_ = IdealOmega(time_series_.back());
  return wt_ * energy_rotation_direction_;
}

/**
 * @brief 获取预测角度
 * @param latency 根据姿态解算和弹道解算求出来的时间
 */
double Omega::get_rad(double latency)
{
  double predict_rad = IdealRad(t_list_.back(), t_list_.back() + latency);
  return predict_rad * energy_rotation_direction_;
}

/**
 * @brief 找到波峰或者波谷来确定相位
 * @return 是否找到
 */
bool Omega::FindWavePeak()
{
  //RCLCPP_INFO(rclcpp::get_logger("FindWavePeak"),"filter_omega_ size:%zu", filter_omega_.size());
  if (filter_omega_.size() > 15)
  {
    std::vector<double> cut_filter_omega(filter_omega_.end() - 8, filter_omega_.end());
    std::vector<double> cut_time_series(time_series_.end() - 8, time_series_.end());
    Eigen::MatrixXd rate = LeastSquare(cut_time_series, cut_filter_omega, 1);
    fit_cnt_ = (rate(0, 0) > 0) ? fit_cnt_ + 1 : fit_cnt_ - 1;
    if (fit_cnt_ > 5)
    {
      if (fabs(filter_omega_.back()) > max_omega_)
      {
        change_cnt_ = 0;
        max_omega_ = filter_omega_.back();
        phi_ = CV_2PI / 2 - w_ * time_series_.back();
        while (phi_ < -CV_PI || phi_ > CV_PI)
        {
          phi_ += 2 * CV_PI;
        }
      }
      else
      {
        change_cnt_++;
      }
    }
    else if (fit_cnt_ < -5)
    {
      if (fabs(filter_omega_.back()) < min_omega_)
      {
        change_cnt_ = 0;
        min_omega_ = filter_omega_.back();
        phi_ = -CV_PI / 2 - w_ * time_series_.back();
        while (phi_ < -CV_PI || phi_ > CV_PI)
        {
          phi_ += 2 * CV_PI;
        }
      }
      else
      {
        change_cnt_++;
      }
    }
    else
    {
      return false;
    }

    if (change_cnt_ > 5)
    {
      if (fit_cnt_ >= 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("FindWavePeak"),"get peak omega : %f, init phi :%f", max_omega_, phi_);
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("FindWavePeak"),"get valley omega : %f, init phi :%f", min_omega_, phi_);
      }
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

/**
 * @brief 最小二乘拟合
 * @param x 输入值
 * @param y 输入值
 * @param N 数量
 * @return 拟合的权重
 */
Eigen::MatrixXd Omega::LeastSquare(std::vector<double> x, std::vector<double> y, int N)
{
  Eigen::MatrixXd A(x.size(), N + 1);
  Eigen::MatrixXd B(y.size(), 1);
  Eigen::MatrixXd W;

  for (unsigned int i = 0; i < x.size(); i++)
  {
    for (int n = N, dex = 0; n >= 1; --n, ++dex)
    {
      A(i, N) = 1;
      B(i, 0) = y[i];
    }
  }
  W = (A.transpose() * A).inverse() * A.transpose() * B;
  return W;
}

/**
 * @brief 获取时间间隔
 */
double Omega::get_time_gap()
{
  return time_series_.back() - time_series_[st_];
}

/**
 * @brief 将置滤波后的角速度转为正值存储
 */
void Omega::set_filter(double d)
{
  energy_exam_cnt_ ++;
  filter_omega_.push_back(energy_rotation_direction_ * d);
}

/**
 * @brief 获取预测误差
 */
double Omega::get_err()
{
  return get_omega() - filter_omega_.back() * energy_rotation_direction_;
}

/**
 * @brief 改变st_的值
 */
void Omega::change_st()
{
  st_ = (filter_omega_.size() - 100) ? filter_omega_.size() - 100 : 0;
}

}  // namespace helios_cv

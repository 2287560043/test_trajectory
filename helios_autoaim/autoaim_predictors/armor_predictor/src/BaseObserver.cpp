#include "BaseObserver.hpp"

namespace helios_cv
{

int BaseObserver::find_priority_armor(autoaim_interfaces::msg::Armors& armors,
                                      const std::vector<std::string>& priority_sequence)
{
  // remove armors not in priority_sequence
  for (auto it = armors.armors.begin(); it != armors.armors.end();)
  {
    bool find_armor = false;
    for (const auto& number : priority_sequence)
    {
      if (it->number == number)
      {
        find_armor = true;
        break;
      }
    }
    if (!find_armor)
    {
      it = armors.armors.erase(it);
    }
    else
    {
      it++;
    }
  }
  // sort by priority_sequence
  std::sort(armors.armors.begin(), armors.armors.end(),
            [&](const autoaim_interfaces::msg::Armor& a, const autoaim_interfaces::msg::Armor& b) {
              int a_position = -1, b_position = -1;
              for (int i = 0; i < priority_sequence.size(); i++)
              {
                if (a.number == priority_sequence[i])
                {
                  a_position = i;
                }
                if (b.number == priority_sequence[i])
                {
                  b_position = i;
                }
              }
              return a_position < b_position;
            });
  // to avoid armors become empty
  if (armors.armors.empty())
  {
    return -1;
  }
  else
  {
    // caculate same priority armor's count
    int same_priority_count = 0;
    for (int i = 1; i < armors.armors.size(); i++)
    {
      if (armors.armors[i] == armors.armors[0])
      {
        same_priority_count++;
      }
    }
    return same_priority_count;
  }
}

int BaseObserver::find_priority_armor(autoaim_interfaces::msg::Armors& armors)
{
  // sort armor by priority
  std::sort(armors.armors.begin(), armors.armors.end(),
            [&](const autoaim_interfaces::msg::Armor& a, const autoaim_interfaces::msg::Armor& b) {
              return a.distance_to_image_center < b.distance_to_image_center;
            });
  // check if armors is empty
  if (armors.armors.empty())
  {
    return -1;
  }
  else
  {
    // caculate same priority armor's count
    int same_priority_count = 0;
    for (int i = 1; i < armors.armors.size(); i++)
    {
      if (armors.armors[i] == armors.armors[0])
      {
        same_priority_count++;
      }
    }
    return same_priority_count;
  }
}

std::map<int, int> BaseObserver::getMatch(Eigen::MatrixXd matrix, double score_max, int m)
{
  /// 初始化最终结果
  this->row_col.clear();
  for (int i = 0; i < matrix.rows(); i++)
  {
    // 值为-1表示不选择该行中的任何数
    this->row_col[i] = -1;
  }
  /// 初始化min
  for (int i = 0; i < matrix.rows() && i < m; i++)
  {
    this->min += matrix(i, i);
    this->row_col[i] = i;
  }
  /// 初始化行数列
  this->row.clear();
  for (int i = 0; i < matrix.rows(); i++)
  {
    // 有多少行，行数列就有多少个数
    this->row.emplace_back(i);
  }
  /// 初始化列数列，n块装甲板，因此列为n
  this->col.clear();
  for (int i = 0; i < m; i++)
  {
    this->col.emplace_back(i);
  }
  this->tmp_v.clear();
  this->result.clear();
  this->nAfour.clear();
  this->fourAfour.clear();
  /**
   * @brief 用A(n,m)求出可能选择的所有行的组合
   * 因为C(n,m) * A(m,m) = A(n,m)，所以先计算C(n,m)，再计算A(m,m)，最后将结果存到result中
   *
   */
  /// 计算C(n,m)
  if (matrix.rows() <= m)
  {
    this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, matrix.rows());
  }
  else
  {
    this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, m);
  }

  /// 使用上一步的结果计算A(m,m)，最终得到A(n,m)
  for (auto it = this->result.begin(); it != this->result.end(); it++)
  {
    do
    {
      this->nAfour.emplace_back(*it);
    } while (std::next_permutation(it->begin(), it->end()));  // stl自带全排列函数
  }
  /// 计算A(m,m)
  do
  {
    fourAfour.push_back(col);
  } while (std::next_permutation(col.begin(), col.end()));
  /**
   * @brief 用A(n,m)和A(m,m)求出可能选择的所有行列的组合的结果
   *
   */
  for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
  {
    for (auto it2 = this->fourAfour.begin(); it2 != this->fourAfour.end(); it2++)
    {
      // 对于每一种组合，都求一遍它们的值，与min比较，如果小于min，则更新min
      this->tmp = 0;
      for (int i = 0; i < it->size(); i++)
      {
        this->tmp += matrix((*it)[i], (*it2)[i]);
      }
      if (this->tmp < this->min)
      {
        this->min = this->tmp;
        // 重新初始化map
        this->row_col.clear();
        // 每选择矩阵中的一个数时，就将该位置记录下来存到map中
        for (int i = 0; i < it->size(); i++)
        {
          this->row_col[it->at(i)] = it2->at(i);
        }
      }
    }
  }
  /// 删除-1的键值对
  for (auto it = this->row_col.begin(); it != this->row_col.end();)
  {
    if (it->second == -1 || (it->second != -1 && matrix(it->first, it->second) > score_max))
    {
      it = this->row_col.erase(it);
    }
    else
    {
      ++it;
    }
  }

  return this->row_col;
}

void BaseObserver::getCombinationsNumbers(std::vector<int>& input, std::vector<int>& tmp_v,
                                          std::vector<std::vector<int>>& result, int start, int k)
{
  for (int i = start; i < input.size(); i++)
  {
    tmp_v.emplace_back(input[i]);
    if (tmp_v.size() == k)
    {
      result.emplace_back(tmp_v);
      tmp_v.pop_back();
      continue;
    }
    // 使用递归计算
    getCombinationsNumbers(input, tmp_v, result, i + 1, k);
    tmp_v.pop_back();
  }
}

Eigen::MatrixXd BaseObserver::getScoreMat(const autoaim_interfaces::msg::Armors::_armors_type& detect_armors,
                                          const autoaim_interfaces::msg::Armors::_armors_type& standard_armors)
{
  int m = detect_armors.size();
  int n = standard_armors.size();
  // 计算两组装甲板之间的坐标差和角度差两个负向指标
  Eigen::Matrix<double, Eigen::Dynamic, 2> negative_score;
  negative_score.resize(m * n, 2);
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < n; j++)
    {
      negative_score(i * n + j, 0) = getDistance(detect_armors[i].pose.position, standard_armors[j].pose.position);
      negative_score(i * n + j, 1) = std::abs(math::get_angle_diff(
          orientation2yaw(detect_armors[i].pose.orientation), orientation2yaw(standard_armors[j].pose.orientation)));
    }
  }

  // 数据标准化
  Eigen::Matrix<double, Eigen::Dynamic, 2> regular_score;
  regular_score.resize(m * n, 2);
  for (int i = 0; i < regular_score.rows(); i++)
  {
    regular_score(i, 0) = (negative_score.col(0).maxCoeff() - negative_score(i, 0)) /
                          (negative_score.col(0).maxCoeff() - negative_score.col(0).minCoeff());
    regular_score(i, 1) = (negative_score.col(1).maxCoeff() - negative_score(i, 1)) /
                          (negative_score.col(1).maxCoeff() - negative_score.col(1).minCoeff());
  }

  // 计算样本值占指标的比重
  Eigen::Matrix<double, Eigen::Dynamic, 2> score_weight;
  score_weight.resize(m * n, 2);
  Eigen::VectorXd col_sum = regular_score.colwise().sum();
  for (int i = 0; i < score_weight.rows(); i++)
  {
    score_weight(i, 0) = regular_score(i, 0) / col_sum(0);
    score_weight(i, 1) = regular_score(i, 1) / col_sum(1);
  }

  // 计算每项指标的熵值
  Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
  for (int i = 0; i < score_weight.rows(); i++)
  {
    if (score_weight(i, 0) != 0)
    {
      entropy(0) -= score_weight(i, 0) * std::log(score_weight(i, 0));
    }
    if (score_weight(i, 1) != 0)
    {
      entropy(1) -= score_weight(i, 1) * std::log(score_weight(i, 1));
    }
  }
  entropy /= std::log(score_weight.rows());

  // 计算权重
  Eigen::Vector2d weight = (Eigen::Vector2d::Ones() - entropy) / (2 - entropy.sum());
  // 计算匹配得分矩阵
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> score;
  score.resize(m, n);
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < n; j++)
    {
      if (i < detect_armors.size() && j < standard_armors.size())
      {
        score(i, j) = negative_score.row(i * standard_armors.size() + j) * weight;
      }
    }
  }
  return score;
}

double BaseObserver::orientation2yaw(const geometry_msgs::msg::Quaternion& orientation)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(orientation, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

double BaseObserver::getDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
{
  Eigen::Vector3d v1(p1.x, p1.y, p1.z);
  Eigen::Vector3d v2(p2.x, p2.y, p2.z);
  return (v1 - v2).norm();
}

}  // namespace helios_cv
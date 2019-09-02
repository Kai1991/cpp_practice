#include "base_filter.h"
class KalmanFilter : BaseFilter
{
private:
    Eigen::VectorXd prior_global_states_;//上一个全局状态
    Eigen::VectorXd gain_break_down_;
    Eigen::VectorXd value_break_down_;
    Eigen::MatrixXd kalman_gain_;//卡尔曼收益
public:
    KalmanFilter(/* args */);
    ~KalmanFilter();

    bool Init(const Eigen::VectorXd &global_states,
                      const Eigen::MatrixXd &global_uncertainty);
    //预测信息
    bool Predict(const Eigen::MatrixXd &transform_matrix,
                         const Eigen::MatrixXd &env_uncertainty_matrix);
    //纠正预测值和观测值
    bool Correct(const Eigen::VectorXd &cur_observation,
                         const Eigen::MatrixXd &cur_observation_uncertainty);

    Eigen::VectorXd getGlobal_states();
};

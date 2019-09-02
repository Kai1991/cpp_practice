#include <Eigen/Dense>
class BaseFilter
{
protected:
    //转移矩阵
    Eigen::MatrixXd transform_matrix_;
    //系统状态值
    Eigen::VectorXd global_states_;
    //系统的不确定性，协方差
    Eigen::MatrixXd global_uncertainty_;
    //环境的不确定性
    Eigen::MatrixXd env_uncertainty_;
    //当前观察值
    Eigen::VectorXd cur_observation_;
    //当前观察值的不确定性，协方差
    Eigen::MatrixXd cur_observation_uncertainty_;
    //控制矩阵
    Eigen::MatrixXd c_matrix_;

    int states_num_;//状态数量


public:
    BaseFilter();
    ~BaseFilter();
    //初始化 状态信息，状态的方差信息
    virtual bool Init(const Eigen::VectorXd &global_states,
                      const Eigen::MatrixXd &global_uncertainty) = 0;
    //预测信息
    virtual bool Predict(const Eigen::MatrixXd &transform_matrix,
                         const Eigen::MatrixXd &env_uncertainty_matrix) = 0;
    //纠正预测值和观测值
    virtual bool Correct(const Eigen::VectorXd &cur_observation,
                         const Eigen::MatrixXd &cur_observation_uncertainty)=0;
};

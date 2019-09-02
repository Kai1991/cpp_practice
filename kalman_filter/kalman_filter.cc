#include "kalman_filter.h"
//初始化
bool KalmanFilter::Init(const Eigen::VectorXd &initial_belief_states,
                      const Eigen::MatrixXd &initial_uncertainty){
    states_num_ = initial_belief_states.size();
    //系统状态初始化
    //系统不确定系初始化
    global_states_ = initial_belief_states;
    global_uncertainty_ = initial_uncertainty;
    prior_global_states_ = global_states_;

    transform_matrix_.setIdentity(states_num_, states_num_);//单位矩阵
    cur_observation_.setZero(states_num_, 1);
    cur_observation_uncertainty_.setIdentity(states_num_, states_num_);

    c_matrix_.setIdentity(states_num_, states_num_);
    env_uncertainty_.setZero(states_num_, states_num_);

    gain_break_down_.setZero(states_num_, 1);
    value_break_down_.setZero(states_num_, 1);

    kalman_gain_.setZero(states_num_, states_num_);
    return true;
};
//预测信息
bool KalmanFilter::Predict(const Eigen::MatrixXd &transform_matrix,const Eigen::MatrixXd &env_uncertainty_matrix){
    transform_matrix_ = transform_matrix;
    env_uncertainty_ = env_uncertainty_matrix;
    global_states_ = transform_matrix_ * global_states_;
    global_uncertainty_ = transform_matrix_ * global_uncertainty_ * transform_matrix_.transpose + env_uncertainty_;
    return true; 
};
//纠正预测值和观测值
bool KalmanFilter::Correct(const Eigen::VectorXd &cur_observation,const Eigen::MatrixXd &cur_observation_uncertainty){
    cur_observation_ = cur_observation;
    cur_observation_uncertainty_ = cur_observation_uncertainty;

    kalman_gain_ = global_uncertainty_ * c_matrix_.transpose() * 
    (c_matrix_ * global_uncertainty_ * c_matrix_.transpose() + cur_observation_uncertainty_).inverse();

    global_states_ = global_states_ + kalman_gain_ * (cur_observation_ - c_matrix_ * global_states_);

    Eigen::MatrixXd tmp_identity;
    tmp_identity.setIdentity(states_num_, states_num_);
    global_uncertainty_ = (tmp_identity - kalman_gain_ * c_matrix_) * global_uncertainty_ * 
                        (tmp_identity - kalman_gain_ * c_matrix_).transpose() + 
                        kalman_gain_ * cur_observation_uncertainty_ * kalman_gain_.transpose();
    return true;
};

Eigen::VectorXd KalmanFilter::getGlobal_states(){
    return global_states_;
};
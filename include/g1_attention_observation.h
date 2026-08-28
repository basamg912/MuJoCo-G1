#pragma once

#include <eigen3/Eigen/Dense>
#include <mujoco/mujoco.h>

#include <string>
#include <vector>

class G1AttentionObservation
{
public:
    G1AttentionObservation() = default;

    void setMujocoModel(const mjModel* m, mjData* d);
    void setVelocityCommand(double vx, double vy, double wz);
    void setComCommand(double dx, double dy, double dz);
    void loadStaticDescription(const std::string& joint_desc_path,
                               const std::string& feet_desc_path);
    void reset();

    Eigen::MatrixXd computeJointObs(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& qdot,
                                    const Eigen::VectorXd& q_home,
                                    const Eigen::VectorXd& last_action);
    Eigen::MatrixXd computeFeetObs();
    Eigen::VectorXd computeGlobalObs();

    const Eigen::MatrixXd& jointDesc() const { return _joint_desc; }
    const Eigen::MatrixXd& feetDesc() const { return _feet_desc; }

private:
    const mjModel* _mj_model = nullptr;
    mjData* _mj_data = nullptr;
    int _root_body_id = -1;
    std::vector<int> _foot_body_ids;

    Eigen::MatrixXd _joint_desc;
    Eigen::MatrixXd _feet_desc;
    Eigen::Vector3d _vel_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d _com_cmd = Eigen::Vector3d::Zero();
};

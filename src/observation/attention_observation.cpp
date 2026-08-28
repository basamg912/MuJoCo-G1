#include "attention_observation.h"
#include "mapping.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

Eigen::MatrixXd load_matrix_bin(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open: " + path);
    int32_t rows, cols;
    f.read(reinterpret_cast<char*>(&rows), 4);
    f.read(reinterpret_cast<char*>(&cols), 4);
    std::vector<float> buf(static_cast<size_t>(rows) * cols);
    f.read(reinterpret_cast<char*>(buf.data()), buf.size() * sizeof(float));
    Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        view(buf.data(), rows, cols);
    return view.cast<double>();
}

}  // namespace

AttentionObservation::AttentionObservation()
{
    _default_com << -0.066071861, -0.19773669, 0.0024721903;
}

void AttentionObservation::setMujocoModel(const mjModel* m, mjData* d)
{
    _mj_model = m;
    _mj_data  = d;

    int root_joint_id = mj_name2id(m, mjOBJ_JOINT, "root");
    _root_body_id = root_joint_id >= 0 ? m->jnt_bodyid[root_joint_id] : mj_name2id(m, mjOBJ_BODY, "pelvis");
    _torso_body_id = mj_name2id(m, mjOBJ_BODY, "WL3");
    if (_root_body_id < 0) std::cout << "[AttentionObs] Root body 'WL3' not found\n";
    else                   std::cout << "[AttentionObs] Root ID : " << _root_body_id << '\n';

    _foot_body_ids.clear();
    for (const char* n : {"LL7", "RL7"}) {
        int id = mj_name2id(m, mjOBJ_BODY, n);
        if (id < 0) std::cout << "[AttentionObs] " << n << " body not found\n";
        else        std::cout << "[AttentionObs] " << n << " ID : " << id << '\n';
        _foot_body_ids.push_back(id);
    }
}

void AttentionObservation::setVelocityCommand(double vx, double vy, double wz) {
    _vel_cmd << vx, vy, wz;
}

void AttentionObservation::setComCommand(double dx, double dy, double dz) {
    _com_cmd << dx, dy, dz;
}

void AttentionObservation::loadStaticDescription(const std::string& joint_desc_path,
                                                 const std::string& feet_desc_path)
{
    _joint_desc = load_matrix_bin(joint_desc_path);
    _feet_desc  = load_matrix_bin(feet_desc_path);
    std::cout << "[AttentionObs] joint_desc: " << _joint_desc.rows() << "x" << _joint_desc.cols() << '\n';
    std::cout << "[AttentionObs] feet_desc: "  << _feet_desc.rows()  << "x" << _feet_desc.cols()  << '\n';
}

void AttentionObservation::reset()
{
    // attention obs is stateless per-step; nothing to reset
}

Eigen::MatrixXd AttentionObservation::computeJointObs(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_home,
    const Eigen::VectorXd& last_action)
{
    const int J = static_cast<int>(_joint_desc.rows());
    Eigen::MatrixXd out(J, 3);
    for (int i = 0; i < J; ++i) {
        int mj = isaac_joint_to_mujoco[i];
        out(i, 0) = q(mj) - q_home(mj);
        out(i, 1) = qdot(mj);
        out(i, 2) = last_action(i);
    }
    return out;
}

Eigen::MatrixXd AttentionObservation::computeFeetObs()
{
    const int F = static_cast<int>(_foot_body_ids.size());
    Eigen::MatrixXd out(F, 2);

    const double* root_pos  = &_mj_data->xpos [3 * _torso_body_id];
    const double* root_quat = &_mj_data->xquat[4 * _torso_body_id];
    mjtNum root_quat_inv[4];
    mju_negQuat(root_quat_inv, root_quat);

    mjtNum root_vel6[6];   // [ang(3), lin(3)] world
    mj_objectVelocity(_mj_model, _mj_data, mjOBJ_BODY, _torso_body_id, root_vel6, 0);
    const mjtNum* root_omega_w = root_vel6;
    const mjtNum* root_linv_w  = root_vel6 + 3;

    for (int f = 0; f < F; ++f) {
        int fid = _foot_body_ids[f];
        const double* foot_pos = &_mj_data->xpos[3 * fid];

        mjtNum foot_vel6[6];
        mj_objectVelocity(_mj_model, _mj_data, mjOBJ_BODY, fid, foot_vel6, 0);
        const mjtNum* foot_linv_w = foot_vel6 + 3;


        mjtNum r_w[3] = {
            foot_pos[0] - root_pos[0],
            foot_pos[1] - root_pos[1],
            foot_pos[2] - root_pos[2],
        };
        mjtNum cross[3];
        mju_cross(cross, root_omega_w, r_w);
        mjtNum v_rel_w[3] = {
            foot_linv_w[0] - root_linv_w[0] - cross[0],
            foot_linv_w[1] - root_linv_w[1] - cross[1],
            foot_linv_w[2] - root_linv_w[2] - cross[2],
        };
        mjtNum r_root[3], v_rel_root[3];
        mju_rotVecQuat(r_root,     r_w,     root_quat_inv);
        mju_rotVecQuat(v_rel_root, v_rel_w, root_quat_inv);

        out(f, 0) = r_root[2];        // foot_z_in_root
        out(f, 1) = v_rel_root[2];    // foot_vz_in_root
    }
    return out;
}

Eigen::VectorXd AttentionObservation::computeGlobalObs()
{
    Eigen::VectorXd out(15);
    int idx = 0;

    mjtNum root_vel6[6];
    mj_objectVelocity(_mj_model, _mj_data, mjOBJ_BODY, _root_body_id, root_vel6, 1);
    // 1) base_lin_vel (body frame)
    for (int i=0; i<3; ++i){
        out(idx++) = root_vel6[3+i];
    }
    // 2) base_ang_vel (body frame)
    for (int i = 0; i < 3; ++i) {
        out(idx++) = root_vel6[i];
    }
    // 3) projected_gravity (body frame)
    const double* root_quat = &_mj_data->xquat[4*_root_body_id];
    mjtNum root_quat_inv[4];
    mju_negQuat(root_quat_inv, root_quat);
    mjtNum gravity_world[3] = {0.0, 0.0, -1.0};
    mjtNum gravity_body[3];
    mju_rotVecQuat(gravity_body, gravity_world, root_quat_inv);
    for (int i = 0; i < 3; ++i) out(idx++) = gravity_body[i];

    // 4) velocity_commands
    out(idx++) = _vel_cmd(0);
    out(idx++) = _vel_cmd(1);
    out(idx++) = _vel_cmd(2);

    // 5) dif_torso_com: body_ipos(local COM) vs nominal default_com
    const double* ipos = &_mj_model->body_ipos[3 * _torso_body_id];
    out(idx++) = ipos[0] - _default_com(0);
    out(idx++) = ipos[1] - _default_com(1);
    out(idx++) = ipos[2] - _default_com(2);

    return out;
}

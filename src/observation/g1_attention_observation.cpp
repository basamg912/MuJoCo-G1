#include "g1_attention_observation.h"

#include "g1_mapping.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

Eigen::MatrixXd load_matrix_bin(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::runtime_error("cannot open: " + path);
    }

    int32_t rows = 0;
    int32_t cols = 0;
    f.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    f.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    if (!f || rows <= 0 || cols <= 0) {
        throw std::runtime_error("invalid matrix header: " + path);
    }

    std::vector<float> buf(static_cast<size_t>(rows) * static_cast<size_t>(cols));
    f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size() * sizeof(float)));
    if (!f) {
        throw std::runtime_error("invalid matrix payload: " + path);
    }

    Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        view(buf.data(), rows, cols);
    return view.cast<double>();
}

int first_freejoint_body(const mjModel* m)
{
    for (int i = 0; i < m->njnt; ++i) {
        if (m->jnt_type[i] == mjJNT_FREE) {
            return m->jnt_bodyid[i];
        }
    }
    return -1;
}

}  // namespace

void G1AttentionObservation::setMujocoModel(const mjModel* m, mjData* d)
{
    _mj_model = m;
    _mj_data = d;

    int floating_joint = mj_name2id(m, mjOBJ_JOINT, "floating_base_joint");
    _root_body_id = floating_joint >= 0 ? m->jnt_bodyid[floating_joint] : first_freejoint_body(m);
    if (_root_body_id < 0) {
        _root_body_id = mj_name2id(m, mjOBJ_BODY, "pelvis");
    }
    if (_root_body_id < 0) {
        throw std::runtime_error("[G1AttentionObs] root body not found");
    }

    _foot_body_ids.clear();
    for (const char* name : {"left_ankle_roll_link", "right_ankle_roll_link"}) {
        int id = mj_name2id(m, mjOBJ_BODY, name);
        if (id < 0) {
            throw std::runtime_error(std::string("[G1AttentionObs] foot body not found: ") + name);
        }
        _foot_body_ids.push_back(id);
    }

    std::cout << "[G1AttentionObs] root body id: " << _root_body_id << '\n';
    std::cout << "[G1AttentionObs] foot body ids: "
              << _foot_body_ids[0] << ", " << _foot_body_ids[1] << '\n';
}

void G1AttentionObservation::setVelocityCommand(double vx, double vy, double wz)
{
    _vel_cmd << vx, vy, wz;
}

void G1AttentionObservation::setComCommand(double dx, double dy, double dz)
{
    _com_cmd << dx, dy, dz;
}

void G1AttentionObservation::loadStaticDescription(const std::string& joint_desc_path,
                                                   const std::string& feet_desc_path)
{
    _joint_desc = load_matrix_bin(joint_desc_path);
    _feet_desc = load_matrix_bin(feet_desc_path);

    if (_joint_desc.rows() != G1_N_JOINTS || _joint_desc.cols() != 16) {
        throw std::runtime_error("[G1AttentionObs] joint_desc shape mismatch");
    }
    if (_feet_desc.rows() != G1_N_FEET || _feet_desc.cols() != 3) {
        throw std::runtime_error("[G1AttentionObs] feet_desc shape mismatch");
    }

    std::cout << "[G1AttentionObs] joint_desc: "
              << _joint_desc.rows() << "x" << _joint_desc.cols() << '\n';
    std::cout << "[G1AttentionObs] feet_desc: "
              << _feet_desc.rows() << "x" << _feet_desc.cols() << '\n';
}

void G1AttentionObservation::reset()
{
}

Eigen::MatrixXd G1AttentionObservation::computeJointObs(const Eigen::VectorXd& q,
                                                        const Eigen::VectorXd& qdot,
                                                        const Eigen::VectorXd& q_home,
                                                        const Eigen::VectorXd& last_action)
{
    Eigen::MatrixXd out(G1_N_JOINTS, 3);
    for (int isaac_idx = 0; isaac_idx < G1_N_JOINTS; ++isaac_idx) {
        int mj_idx = g1_isaac_joint_to_mujoco[isaac_idx];
        out(isaac_idx, 0) = q(mj_idx) - q_home(mj_idx);
        out(isaac_idx, 1) = qdot(mj_idx);
        out(isaac_idx, 2) = last_action(isaac_idx);
    }
    return out;
}

Eigen::MatrixXd G1AttentionObservation::computeFeetObs()
{
    Eigen::MatrixXd out(G1_N_FEET, 2);

    const double* root_pos = &_mj_data->xpos[3 * _root_body_id];
    const double* root_quat = &_mj_data->xquat[4 * _root_body_id];
    mjtNum root_quat_inv[4];
    mju_negQuat(root_quat_inv, root_quat);

    mjtNum root_vel6[6];
    mj_objectVelocity(_mj_model, _mj_data, mjOBJ_BODY, _root_body_id, root_vel6, 0);
    const mjtNum* root_omega_w = root_vel6;
    const mjtNum* root_linv_w = root_vel6 + 3;

    for (int f = 0; f < G1_N_FEET; ++f) {
        int foot_id = _foot_body_ids[f];
        const double* foot_pos = &_mj_data->xpos[3 * foot_id];

        mjtNum foot_vel6[6];
        mj_objectVelocity(_mj_model, _mj_data, mjOBJ_BODY, foot_id, foot_vel6, 0);
        const mjtNum* foot_linv_w = foot_vel6 + 3;

        mjtNum rel_pos_w[3] = {
            foot_pos[0] - root_pos[0],
            foot_pos[1] - root_pos[1],
            foot_pos[2] - root_pos[2],
        };
        mjtNum omega_cross_r[3];
        mju_cross(omega_cross_r, root_omega_w, rel_pos_w);

        mjtNum rel_vel_w[3] = {
            foot_linv_w[0] - root_linv_w[0] - omega_cross_r[0],
            foot_linv_w[1] - root_linv_w[1] - omega_cross_r[1],
            foot_linv_w[2] - root_linv_w[2] - omega_cross_r[2],
        };

        mjtNum rel_pos_root[3];
        mjtNum rel_vel_root[3];
        mju_rotVecQuat(rel_pos_root, rel_pos_w, root_quat_inv);
        mju_rotVecQuat(rel_vel_root, rel_vel_w, root_quat_inv);

        out(f, 0) = rel_pos_root[2];
        out(f, 1) = rel_vel_root[2];
    }

    return out;
}

Eigen::VectorXd G1AttentionObservation::computeGlobalObs()
{
    Eigen::VectorXd out(15);
    int idx = 0;

    mjtNum root_vel6[6];
    mj_objectVelocity(_mj_model, _mj_data, mjOBJ_BODY, _root_body_id, root_vel6, 1);
    for (int i = 0; i < 3; ++i) {
        out(idx++) = root_vel6[3 + i];
    }
    for (int i = 0; i < 3; ++i) {
        out(idx++) = root_vel6[i];
    }

    const double* root_quat = &_mj_data->xquat[4 * _root_body_id];
    mjtNum root_quat_inv[4];
    mju_negQuat(root_quat_inv, root_quat);
    mjtNum gravity_world[3] = {0.0, 0.0, -1.0};
    mjtNum gravity_body[3];
    mju_rotVecQuat(gravity_body, gravity_world, root_quat_inv);
    for (int i = 0; i < 3; ++i) {
        out(idx++) = gravity_body[i];
    }

    out(idx++) = _vel_cmd(0);
    out(idx++) = _vel_cmd(1);
    out(idx++) = _vel_cmd(2);

    out(idx++) = _com_cmd(0);
    out(idx++) = _com_cmd(1);
    out(idx++) = _com_cmd(2);

    return out;
}

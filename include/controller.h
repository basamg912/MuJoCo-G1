#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "observation.h"
#include "policy.h"

#include <eigen3/Eigen/Dense>
#include <vector>

#include "robotmodel.h"
#include "custommath.h"

using namespace std;
using namespace Eigen;

#define NECS2SEC 1000000000

class CController
{

public:
    CController();
    virtual ~CController();

    Observation _obs;
    std::unique_ptr<Policy> _policy;
    Eigen::VectorXd _last_action;

    void read(double time, double* q, double* qdot);
    void control_mujoco();
    void step_pd();
    void write(double* ctrl);
    void set_default_pose(mjData* d);
    void setModel(const mjModel* m, mjData* d){
        Model.set_mujoco_model(m,d);
        _obs.setMujocoModel(m,d);
        _qpos_adr.clear();
        for (int i = 0; i < m->njnt; i++) {
            if (m->jnt_type[i] == mjJNT_FREE) continue;
            _qpos_adr.push_back(m->jnt_qposadr[i]);
        }
        // for(int i=0 ; i< m->njnt; i++){
        //     std::cout << "[INFO] Joint " << i << " name : " << mj_id2name(m, mjOBJ_BODY, i) <<'\n';
        // }
    }
    void loadPolicy(const std::string& onnx_path){
        _policy = std::make_unique<Policy>(onnx_path);
        _last_action.setZero(29);
    }
    void Initialize();
    void reset();

    VectorXd _q, _qdot;
    std::vector<int> _qpos_adr;
    
private:
    VectorXd _kp_diag, _kd_diag;
    void ModelUpdate();
    void JointControl();

    VectorXd _torque, _pre_q, _pre_qdot;
    int _k; // DOF

    bool _bool_init;
    double _t;
    double _dt;
    double _init_t;
    double _pre_t;

    CModel Model;

    VectorXd _q_home;
    VectorXd _q_des, _qdot_des;
    
};

#endif

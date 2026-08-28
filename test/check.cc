#include "mujoco/mjmodel.h"
#include <mujoco/mujoco.h>
#include <iostream>

int main(int argc, char** argv) {
    char filepath[100] = "/home/kist/work/workspace/mjc-sangyun/model/kapex/kapex_play.xml";
    char err[100] = "";
    mjModel* m = mj_loadXML(filepath, nullptr, err, sizeof(err));

    if (!m) {
        std::cerr << "Model Load Error: " << err << std::endl;
        return 1;
    }

    // ==========================================
    // 1. 모든 Joint 정보 출력
    // ==========================================
    std::cout << "=== [JOINT RANGE INFO] ===\n";
    for (int i = 0; i < m->njnt; i++) {
        const char* name = mj_id2name(m, mjOBJ_JOINT, i);
        std::string jnt_name = name ? name : "unnamed";

        double min_jnt = m->jnt_range[i * 2];
        double max_jnt = m->jnt_range[i * 2 + 1];

        std::cout << "[Joint ID " << i << " | " << jnt_name << "] "
                  << "Range: " << min_jnt << " ~ " << max_jnt;

        if (!m->jnt_limited[i]) std::cout << " (No Limit)";
        std::cout << "\n";
    }

    std::cout << "\n";

    // ==========================================
    // 2. 모든 Actuator(Ctrl) 정보 및 매핑된 Joint 출력
    // ==========================================
    std::cout << "=== [ACTUATOR CTRLRANGE INFO] ===\n";
    for (int i = 0; i < m->nu; i++) {
        const char* act_name_ptr = mj_id2name(m, mjOBJ_ACTUATOR, i);
        std::string act_name = act_name_ptr ? act_name_ptr : "unnamed_act";

        double min_ctrl = m->actuator_ctrlrange[i * 2];
        double max_ctrl = m->actuator_ctrlrange[i * 2 + 1];

        // 이 Actuator가 어떤 오브젝트(대개 Joint)를 target으로 하는지 확인
        int trn_type = m->actuator_trntype[i];
        int target_id = m->actuator_trnid[i * 2]; // 대상 오브젝트의 ID

        std::cout << "[Actuator ID " << i << " | " << act_name << "] ";
        std::cout << "Ctrlrange: " << min_ctrl << " ~ " << max_ctrl;

        // 만약 이 액추에이터가 Joint를 제어하는 거라면 해당 Joint 이름도 같이 출력
        if (trn_type == mjTRN_JOINT) {
            const char* jnt_name_ptr = mj_id2name(m, mjOBJ_JOINT, target_id);
            std::cout << " --> Target [Joint ID " << target_id << " | " << (jnt_name_ptr ? jnt_name_ptr : "unnamed") << "]";
        }

        if (!m->actuator_ctrllimited[i]) std::cout << " (No Ctrl Limit)";
        std::cout << "\n";
    }


    std::cout << "=== [JOINT ARMATURE INFO] ===\n";
    for (int i = 0; i < m->njnt; i++) {
        const char* jnt_name = mj_id2name(m, mjOBJ_JOINT, i);
        double joint_armature = m->dof_armature[i];
        int joint_type = m->jnt_type[i];
        int dof_start_idx = m->jnt_dofadr[i];
        std::cout << "[JOINT ID " << i << " | " << jnt_name << "] ";

        if (joint_type == mjJNT_FREE){
            std::cout << "Armature (FREE JOINT) : ";
            for (int d=0; d<6; d++){
                std::cout << m->dof_armature[dof_start_idx+d] << ' ';
            }
            std::cout << '\n';
        }
        else{
            double joint_armature = m->dof_armature[dof_start_idx];
            std::cout << "Armature : " << joint_armature <<'\n';
        }
    }

    std::cout << "=== LINK MASS ===\n";
    for (int i=0; i<m->nbody; i++){
        const char* linkname = mj_id2name(m, mjOBJ_BODY, i);
        std::cout << "[LINK ID " << i << " | " << linkname << " ]";
        std::cout << " Mass : " << m->body_mass[i] << '\n';
    }

    mj_deleteModel(m);
    return 0;
}

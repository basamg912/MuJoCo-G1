"""*******************************************************************************
* HumARConoid-KAPEX
*
* Advanced Humanoid Locomotion Strategy using Reinforcement Learning
*
*     https://github.com/S-CHOI-S/HumARConoid-KAPEX.git
*
* Advanced Robot Control Lab. (ARC)
* 	  @ Korea Institute of Science and Technology
*
*	  https://sites.google.com/view/kist-arc
*
*******************************************************************************"""

"* Authors: Sol Choi *"

import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from pace_sim2real.utils.assetloader import LoadUrdfFileCfg
from pace_sim2real.utils import PaceDCMotorCfg
# LLJ1 : roll
# LLJ2 : yaw
# LLJ3 : pitch
# LLJ4 : knee pitch
# J5 : ankle pitch
# J6 : ankle roll
# J7 : toe pitch

ARMATURE_RO80 = 0.084934656  # Hip Roll, Pitch & Ankle Pitch, Roll & Shoulder Pitch, Roll
ARMATURE_RO100 = 0.2808152064  # Hip Pitch, Knee
ARMATURE_RI60 = 0.001754431488  # Toe
ARMATURE_AK8064 = 0.2312192  # Waist RYP
ARMATURE_AK1093 = 0.0081162  # Shoulder Yaw, Elbow
ARMATURE_AK709 = 0.00288  # Wrist Yaw
ARMATURE_WRIST_RP = 0.00003232  # Wrist Roll & Pitch
ARMATURE_AK7010 = 0.00414  # Head Pitch
ARMATURE_AK606 = 0.0008766 # Head Yaw

NATURAL_FREQ = 5 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_RO80 = ARMATURE_RO80 * NATURAL_FREQ**2          # 83.8271454615
STIFFNESS_RO100 = ARMATURE_RO100 * NATURAL_FREQ**2        # 277.153499682
STIFFNESS_RI60 = ARMATURE_RI60 * NATURAL_FREQ**2          # 1.73155447344
STIFFNESS_AK8064 = ARMATURE_AK8064 * NATURAL_FREQ**2      # 228.204203381
STIFFNESS_AK1093 = ARMATURE_AK1093 * NATURAL_FREQ**2      # 8.01036832355
STIFFNESS_AK709 = ARMATURE_AK709 * NATURAL_FREQ**2        # 2.84244606735
STIFFNESS_WRIST_RP = ARMATURE_WRIST_RP * NATURAL_FREQ**2  # 0.0318985614225
STIFFNESS_AK7010 = ARMATURE_AK7010 * NATURAL_FREQ**2      # 4.08601622182
STIFFNESS_AK606 = ARMATURE_AK606 * NATURAL_FREQ**2        # 0.86516952175
DAMPING_RO80 = 2.0 * DAMPING_RATIO * ARMATURE_RO80 * NATURAL_FREQ          # 10.6732036527
DAMPING_RO100 = 2.0 * DAMPING_RATIO * ARMATURE_RO100 * NATURAL_FREQ        # 35.2882795767
DAMPING_RI60 = 2.0 * DAMPING_RATIO * ARMATURE_RI60 * NATURAL_FREQ          # 0.220468362951
DAMPING_AK8064 = 2.0 * DAMPING_RATIO * ARMATURE_AK8064 * NATURAL_FREQ      # 29.0558616027
DAMPING_AK1093 = 2.0 * DAMPING_RATIO * ARMATURE_AK1093 * NATURAL_FREQ      # 1.01991177177
DAMPING_AK709 = 2.0 * DAMPING_RATIO * ARMATURE_AK709 * NATURAL_FREQ        # 0.361911473683
DAMPING_WRIST_RP = 2.0 * DAMPING_RATIO * ARMATURE_WRIST_RP * NATURAL_FREQ  # 0.00406145098244
DAMPING_AK7010 = 2.0 * DAMPING_RATIO * ARMATURE_AK7010 * NATURAL_FREQ      # 0.52024774342
DAMPING_AK606 = 2.0 * DAMPING_RATIO * ARMATURE_AK606 * NATURAL_FREQ        # 0.110156804802


KAPEX0_CFG = ArticulationCfg(
    spawn=LoadUrdfFileCfg(
        asset_path="/home/kist/work/workspace/pace-humanoid/kapex0_description/KAPEX_wo_hand_head.urdf",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.3),
        joint_pos={
            # ".*": 0.0,

            ## LEGS (14)
            "LLJ3": -0.035,
            "RLJ3": 0.035,
            "LLJ4": 0.38,
            "RLJ4": -0.38,
            "LLJ5": -0.33,
            "RLJ5": 0.33,

            ## ARMS (14)
            "LAJ1": 0.2,
            "RAJ1": -0.2,
            "LAJ2": 0.2,
            "RAJ2": -0.2,
            "LAJ3": 0.18,
            "RAJ3": -0.18,
            "LAJ4": -1.,  # 팔꿈치 og LAJ4 = -0.35 , RAJ4 = 0.35 [-1.658~0.175]
            "RAJ4": 1.,

            ## HANDS (40)
            # ".*HJ_index2": 0.2,
            # ".*HJ_index3": 0.2,
            # ".*HJ_little1": 0.2,
            # ".*HJ_little2": 0.2,
            # ".*HJ_middle2": 0.2,
            # ".*HJ_middle3": 0.2,
            # ".*HJ_ring2": 0.2,
            # ".*HJ_ring3": 0.2,
            # "LHJ_thumb0": -0.45,
            # "RHJ_thumb0": 0.45,
            # ".*HJ_thumb3": 0.3,
            # ".*HJ_thumb4": 0.15,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs_hip": PaceDCMotorCfg(
            joint_names_expr=["(L|R)LJ[1-2]"],
            effort_limit=70,
            velocity_limit=12,
            stiffness=STIFFNESS_RO80,
            damping=DAMPING_RO80,
            armature=ARMATURE_RO80,
            saturation_effort=90,
            max_delay=10,
            encoder_bias=[0.0] * 4,
        ),
        "legs_knee": PaceDCMotorCfg(
            joint_names_expr=["(L|R)LJ[3-4]"],
            effort_limit=180,
            velocity_limit=12,
            stiffness=STIFFNESS_RO100,
            damping=20,
            armature=ARMATURE_RO100,
            saturation_effort=220,
            max_delay=10,
            encoder_bias=[0.0] * 4,
        ),
        "ankle": PaceDCMotorCfg(
            joint_names_expr=["(L|R)LJ[5-6]"],
            effort_limit=60,
            velocity_limit=10,
            stiffness=40,
            damping=2,
            armature=0.5 * ARMATURE_RO80,
            saturation_effort=150,
            max_delay=10,
            encoder_bias=[0.0] * 4,
        ),
        "toe": PaceDCMotorCfg(
            joint_names_expr=["(L|R)LJ7"],
            effort_limit=20,
            velocity_limit=5,
            stiffness=5,
            damping=0.05,
            armature=3.0 * ARMATURE_RI60,
            saturation_effort=30,
            max_delay=10,
            encoder_bias=[0.0] * 2,
        ),
        # ! 내장 physx PD 제어기를 이용해 position 제어, action 에서 target 지정 X 필요
        # ! init_state 에서 지정한 포지션을 유지
        "torso": ImplicitActuatorCfg(
            joint_names_expr=["WLJ[1-3]"],
            stiffness=200.0,
            damping=20.0,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*AJ[1-7]"],
            stiffness=50.0,
            damping=5.0,
        ),
    },
)
"""Configuration for the KAPEX_GEN2_WHOLEBODY_CFG robot."""

# """Joint names of KAPEX Gen2 Wholebody robot."""
# 0  'LLJ1'
# 1  'RLJ1'
# 2  'WLJ1'
# 3  'LLJ2'
# 4  'RLJ2'
# 5  'WLJ2'
# 6  'LLJ3'
# 7  'RLJ3'
# 8  'WLJ3'
# 9  'LLJ4'
# 10 'RLJ4'
# 11 'HLJ1'
# 12 'LAJ1'
# 13 'RAJ1'
# 14 'LLJ5'
# 15 'RLJ5'
# 16 'HLJ2'
# 17 'LAJ2'
# 18 'RAJ2'
# 19 'LLJ6'
# 20 'RLJ6'
# 21 'LAJ3'
# 22 'RAJ3'
# 23 'LLJ7'
# 24 'RLJ7'
# 25 'LAJ4'
# 26 'RAJ4'
# 27 'LAJ5'
# 28 'RAJ5'
# 29 'LAJ6'
# 30 'RAJ6'
# 31 'LAJ7'
# 32 'RAJ7'
# 33 'LHJ_index0'
# 34 'LHJ_little0'
# 35 'LHJ_middle0'
# 36 'LHJ_ring0'
# 37 'LHJ_thumb0'
# 38 'RHJ_index0'
# 39 'RHJ_little0'
# 40 'RHJ_middle0'
# 41 'RHJ_ring0'
# 42 'RHJ_thumb0'
# 43 'LHJ_index1'
# 44 'LHJ_little1'
# 45 'LHJ_middle1'
# 46 'LHJ_ring1'
# 47 'LHJ_thumb1'
# 48 'RHJ_index1'
# 49 'RHJ_little1'
# 50 'RHJ_middle1'
# 51 'RHJ_ring1'
# 52 'RHJ_thumb1'
# 53 'LHJ_index2'
# 54 'LHJ_little2'
# 55 'LHJ_middle2'
# 56 'LHJ_ring2'
# 57 'LHJ_thumb2'
# 58 'RHJ_index2'
# 59 'RHJ_little2'
# 60 'RHJ_middle2'
# 61 'RHJ_ring2'
# 62 'RHJ_thumb2'
# 63 'LHJ_index3'
# 64 'LHJ_middle3'
# 65 'LHJ_ring3'
# 66 'LHJ_thumb3'
# 67 'RHJ_index3'
# 68 'RHJ_middle3'
# 69 'RHJ_ring3'
# 70 'RHJ_thumb3'
# 71 'LHJ_thumb4'
# 72 'RHJ_thumb4'
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, scale=0.2, noise=UniNoise(n_min=-0.2, n_max=0.2)
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity, noise=UniNoise(n_min=-0.05, n_max=0.05)
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, params={"command_name": "base_velocity"}
        )
        joint_pos_rel = ObsTerm(
            func=mdp.joint_pos_rel, noise=UniNoise(n_min=-0.01, n_max=0.01)
        )
        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel, scale=0.05, noise=UniNoise(n_min=-1.5, n_max=1.5)
        )
        last_action = ObsTerm(func=mdp.last_action)
        dif_torso_com = ObsTerm(func=mdp.torso_com)
        def __post_init__(self):
            self.history_length = 5
            self.enable_corruption = True
            self.concatenate_terms = True

    # ! critic network 는 noise X
    @configclass
    class CriticCfg(ObsGroup):
        """Observations for critic group."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, params={"command_name": "base_velocity"}
        )
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, scale=0.05)
        last_action = ObsTerm(func=mdp.last_action)
        
        dif_torso_com = ObsTerm(func=mdp.torso_com)
        height_scan = ObsTerm(
            func = mdp.height_scan,
            params={"sensor_cfg" : SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )
        obs_feet_air_time = ObsTerm(
            func=mdp.obs_feet_air_time,
            params={
                "sensor_cfg" : SceneEntityCfg("contact_forces", body_names=["LL[67]","RL[67]"]),
            },
        )
        
        def __post_init__(self):
            self.history_length = 5

    critic: CriticCfg = CriticCfg()
    policy: PolicyCfg = PolicyCfg()
    
    
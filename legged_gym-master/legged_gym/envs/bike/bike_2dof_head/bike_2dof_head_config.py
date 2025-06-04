# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class Bike2DofHeadCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_actions = 2
        num_drives = 2
        num_dofs = 3
        num_observations = 3 + 3 + 3 + 3 + num_drives + num_drives + num_actions
        
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        measure_heights = False
        
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.55] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'joint_front_turn':0.0,
            'joint_f_wheel':0.0,
            'joint_rear_wheel':0.0,
        }
    
    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        
        control_type = 'P'
        use_turnV = False
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 2.0}     # [N*m*s/rad]
        # stiffness = {'joint': 5.}  # [N*m/rad]
        # damping = {'joint': 0}     # [N*m*s/rad]
        wheel_stiffness = 5.
        wheel_damping = 0.2
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        
        # control_type = 'V'
        turn_params = [5,60,0.,(-3.,3.),(-3,3),True]
        use_turnV_clip = False
        turnV_clip = 9.
        # stiffness = {'joint': 10.}  # [N*m/rad]
        # damping = {'joint': 0.02}     # [N*m*s/rad]
        # wheel_stiffness = 10.
        # wheel_damping = 0.015
        ## action scale: target angle = actionScale * action + defaultAngle
        # action_scale = 0.25
        
        # control_type = 'T'
        # stiffness = {'joint_front_turn': 0., "joint_f_wheel": 0., "joint_rear_wheel": 0.}  # [N*m/rad]
        # damping = {'joint_front_turn': 0.3, "joint_f_wheel": 0.2, "joint_rear_wheel": 0.2}     # [N*m*s/rad]
        # # damping = {'joint_front_turn': 0., "joint_f_wheel": 0., "joint_rear_wheel": 0.}     # [N*m*s/rad]
        # wheel_stiffness = 0.
        # wheel_damping = 0.
        # action_scale = 1.
        
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_scale_reduction = 0.5

        use_filter = False
        
    class commands(LeggedRobotCfg.commands):
        curriculum = False
        max_curriculum = 3.0
        num_commands = 5  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 5.  # time before command are changed[s]
        headingVel_command = True  # if true: compute ang vel command from heading error
        headingVel_kp = 0.5
        tracking_headingVel_with_lin_y = False
        global_reference = False
        use_angVel_dead_zone = False
        angVel_dead_zone = 0.1
        use_linVel_dead_zone = False
        linVel_dead_zone = 0.2
        lin_vel_add_p = 0.25

        class ranges:
            lin_vel_x = [1., 3.]  # min max [m/s]
            ang_vel_yaw = [-0.8, 0.8]  # min max [rad/s]
            heading = [-3.1415, 3.1415]
            # heading = [-1.57,1.57]
            
    class asset(LeggedRobotCfg.asset):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/bikeR/urdf/bikeR_low_effort.urdf"
        name = "bikeR"
        penalize_contacts_on = []
        terminate_after_contacts_on = ["base","front_turn"]
        self_collisions = 1
        flip_visual_attachments = False
        
    class normalization(LeggedRobotCfg.normalization):
        class obs_scales(LeggedRobotCfg.normalization.obs_scales):
            lin_vel = 2.0
            ang_vel = 0.25
            dof_pos = 1.0
            dof_vel = 0.05
            height_measurements = 5.0
        clip_observations = 100.
        #clip_actions = 1.2
        clip_actions = 20
        use_turn_clip = False
        turn_actions_clip = 40
        
    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_init_yaw = False
        randomize_init_roll = False
        init_yaw_range = [-3.1415,3.1415]
        init_roll_range = [-0.3,0.3]
        randomize_friction = True
        #friction_range = [0.2, 2.75]
        friction_range = [0., 1.5]
        randomize_restitution = False
        restitution_range = [0.0,1.0]
        randomize_base_mass = True
        #added_mass_range = [-1., 3.]
        # added_mass_range = [-1, 70.]
        added_mass_range = [-2., 2.]
        randomize_base_com = True
        added_com_range = [-0.1, 0.1]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 0.25

        randomize_motor = True
        motor_strength_range = [0.7, 1.1]

        randomize_kpkd = True
        kp_range = [0.9,1.1]
        ki_range = [0.9,1.1]
        kd_range = [0.9,1.1]

        randomize_lag_timesteps = True
        lag_timesteps = 3

        disturbance = True
        disturbance_range = [-10.0, 10.0]
        disturbance_interval = 8

        randomize_initial_joint_pos = True
        initial_joint_pos_range = [-1., 1.]
        
    class rewards(LeggedRobotCfg.rewards):
        #soft_dof_pos_limit = 0.9 
        # soft_dof_vel_limit = 0.9
        # soft_torque_limit = 0.9
        # base_height_target = 0.34
        # clearance_height_target = -0.24        
        bike_turn_limit = 1.6
        #base_height_target = 0.55
        #base_height_target2 = 0.85
        #clearance_height_target = -0.2

        only_positive_rewards = True
        # headingVel params
        lin_tracking_sigma = 0.25
        ang_tracking_sigma = 0.1
        ang_tracking_sigma_tight = 0.05
        heading_sigma = 0.05
        heading_wide_sigma = 0.15
        
        # no headingVel params
        # lin_tracking_sigma = 0.25
        # ang_tracking_sigma = 0.75
        # ang_tracking_sigma_tight = 0.05
        # heading_sigma = 0.025
        
        use_world_coord_constrain = False
        orientation_edge = 0.9
        class scales( LeggedRobotCfg.rewards.scales ):
            # foot_clearance = -1
            # foot_mirror = -0.1
            # foot_slide = -0.1
            
            # headingVel params
            orientation = -200.
            termination = -10000.0
            # addition_wheel_acc = -5.e-6
            # tracking_ang_vel = 1.5
            # tracking_ang_vel_tight = 1.
            # addition_turn_acc = -5.e-6
            # orientation_vel = -0.25
            # front_turn_vel = -0.05
            # wheel_slip = -0.5
            # ang_vel_xy = -0.1
            # tracking_lin_vel=0.
            tracking_lin_vel_x = 1.
            # tracking_lin_vel_y = 0.
            # heading = 1.5
            # heading_wide = 1.5
            # ang_vel_z = -0.05
            # ang_vel_xy = 0.
            # action_rate = -0.005
            # action_rate = -0.002#-0.01
            # action_smoothness=-0.001#-0.002
            # torques = -0.001
            
            tracking_lin_vel = 0.
            tracking_ang_vel = 0.
            lin_vel_z = 0.
            ang_vel_xy = 0.
            torques = 0.
            dof_vel = 0.
            dof_acc = 0.
            base_height = 0. 
            feet_air_time =  0.
            collision = 0.
            feet_stumble = 0.0 
            action_rate = 0.
            stand_still = 0.
            
    pass

class Bike2DofHeadCfgPPO(LeggedRobotCfgPPO):
    class policy( LeggedRobotCfgPPO.policy ):
        actor_hidden_dims = [128, 64, 32]
        critic_hidden_dims = [128, 64, 32]
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

    class algorithm( LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01

    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'flat_bike'
        load_run = -1
        max_iterations = 5000
    pass
import isaacgym
import torch 
from isaacgym import gymapi
import numpy as np
import argparse
import torch.nn.functional as F
from typing import * 
import yaml 
import os 

asset_loaded = False 

def load_cfg(cfg_path: str) -> Dict: 
    cfg = yaml.load(open(cfg_path, 'r'), Loader=yaml.FullLoader)
    return cfg 

def default_sim_params(use_gpu, up_axis = 'Z', hz = 60.0, substeps = 4, num_position_iterations = 8, num_velocity_iterations = 2):
    sim_params = gymapi.SimParams()
    sim_params.up_axis = gymapi.UP_AXIS_Y if up_axis == 'Y' else gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, -9.8, 0.0) if up_axis == 'Y' else gymapi.Vec3(0.0, 0.0, -9.8)
    sim_params.dt = 1.0 / hz
    sim_params.substeps = substeps
    sim_params.use_gpu_pipeline = use_gpu 
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = num_position_iterations
    sim_params.physx.num_velocity_iterations = num_velocity_iterations
    sim_params.physx.rest_offset = 0.0
    sim_params.physx.contact_offset = 0.001
    sim_params.physx.friction_offset_threshold = 0.001
    sim_params.physx.friction_correlation_distance = 0.0005
    sim_params.physx.num_threads = 4
    sim_params.physx.use_gpu = use_gpu
    return sim_params


def load_axis(gym, sim, device, size, asset_root = './assets'):

    robot_asset_file = "{}_axis.urdf".format(size)
    asset_options = gymapi.AssetOptions()
    asset_options.armature = 0.01
    asset_options.fix_base_link = True
    asset_options.disable_gravity = True
    robot_asset = gym.load_asset(sim, asset_root, robot_asset_file, asset_options)

    return robot_asset



def load_left_leap_hand_asset(gym, sim, device, asset_root = '../bidex_sim/assets'):

    robot_asset_file = "robots/hands/allegro_hand/allegro_hand_left.urdf"
    asset_options = gymapi.AssetOptions()
    asset_options.armature = 0.01
    asset_options.fix_base_link = False
    asset_options.disable_gravity = True
    asset_options.flip_visual_attachments = False
    asset_options.use_mesh_materials = True
    asset_options.vhacd_enabled = True
    asset_options.vhacd_params = gymapi.VhacdParams()
    asset_options.vhacd_params.resolution = 1000000
    robot_asset = gym.load_asset(sim, asset_root, robot_asset_file, asset_options)

    return robot_asset 

def load_ur3e_asset(gym, sim, device, asset_root = '../bidex_sim/assets', hand = None, chirality = None, control = {'arm': 'POS',  'hand': 'POS'}):

    if hand is None: 
        robot_asset_file = "robots/ur_description/urdf/ur3e.urdf"
    else:
        robot_asset_file = "robots/ur_description/urdf/ur3e_{}_{}.urdf".format(hand, chirality)

    asset_options = gymapi.AssetOptions()
    asset_options.armature = 0.01
    asset_options.fix_base_link = True
    asset_options.disable_gravity = True
    asset_options.flip_visual_attachments = False
    asset_options.use_mesh_materials = True
    asset_options.vhacd_enabled = True
    asset_options.vhacd_params = gymapi.VhacdParams()
    if control['arm'] == 'POS': 
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
    elif control['arm'] == 'EFFORT':
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_EFFORT
    asset_options.vhacd_params.resolution = 1000000
    robot_asset = gym.load_asset(sim, asset_root, robot_asset_file, asset_options)
    robot_dof_props = gym.get_asset_dof_properties(robot_asset)
    for i in range(6, 22):
        robot_dof_props['driveMode'][i] = gymapi.DOF_MODE_POS
        robot_dof_props['stiffness'][i] = 10000
        robot_dof_props['damping'][i] = 500
    return robot_asset, robot_dof_props


def refresh_tensors(gym, sim): 
    # refresh tensors
    gym.refresh_rigid_body_state_tensor(sim)
    gym.refresh_actor_root_state_tensor(sim)
    gym.refresh_dof_state_tensor(sim)
    gym.refresh_jacobian_tensors(sim)
    gym.refresh_mass_matrix_tensors(sim)

def setup_viewer_camera(gym, env, viewer): 
    cam_pos = gymapi.Vec3(0.0, -0.05, 1.55)
    cam_target = gymapi.Vec3(0.0, 0.5, 0.4)
    middle_env = env
    gym.viewer_camera_look_at(viewer, middle_env, cam_pos, cam_target)

def adjust_viewer_camera(gym, env, viewer, cam_pos, cam_target):
    gym.viewer_camera_look_at(viewer, env, cam_pos, cam_target)
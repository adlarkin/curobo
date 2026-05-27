from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.prims import SingleArticulation 
from isaacsim.core.utils.types import ArticulationAction
from omni.isaac.core import SimulationContext
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage

class WorldEngine():
    def __init__(self):
        self.world=World()
        self.world.scene.add_default_ground_plane()

        # add robot
        robot_str = "/World/Robot_1"
        robot_path = 'usds/iiwa_left_robotiq.usd'
        add_reference_to_stage(usd_path=robot_path, prim_path=robot_str)
        self.robot = SingleArticulation(prim_path=robot_str, name=robot_str)
        self.world.scene.add(self.robot)

        self.simulation_context = SimulationContext()
        self.simulation_context.initialize_physics()
        self.simulation_context.play()

        print(self.robot.dof_names)
        self.total_dof = len(self.robot.dof_names)
        self.left_knuckle_id = self.robot.dof_names.index('finger_joint')
        self.right_knuckle_id = self.robot.dof_names.index('right_outer_knuckle_joint')

    def step(self, gripper_value):
        q = self.robot.get_joint_positions()

        # the passive joints have non-zero value
        # print(q)

        # left side of the gripper
        q[self.left_knuckle_id] = gripper_value

        # right side of the gripper
        q[self.right_knuckle_id] = gripper_value

        self.robot.apply_action(ArticulationAction(joint_positions=q))
        self.simulation_context.step(render=True)

if __name__ == "__main__":
    engine = WorldEngine()
    while True:
        for i in range(100):
            engine.step(i/100.)
        for i in range(100):
            engine.step(1. - i/100.)

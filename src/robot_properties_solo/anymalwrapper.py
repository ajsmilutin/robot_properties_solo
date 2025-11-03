"""solo12wrapper

Solo12 interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""
import numpy as np
from robot_properties_solo.config import AnymalConfig

dt = 1e-3

class AnymalRobot():
    """
    Similar12 robot used for ROS + Gazebo projects
    """
    def __init__(self):

        self.urdf_path = AnymalConfig.urdf_path
        self.mjcf_path = AnymalConfig.mjcf_path

        # Create the robot wrapper in pinocchio.
        self.pin_robot = AnymalConfig.buildRobotWrapper()

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []

        for leg in ["LF", "RF", "LH", "RH"]:
            controlled_joints += [leg + "_HAA", leg + "_HFE", leg + "_KFE"]
            self.end_eff_ids.append(
                self.pin_robot.model.getFrameId(leg + "_FOOT")
            )
            self.end_effector_names.append(leg + "_FOOT")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        self.hl_index = self.pin_robot.model.getFrameId("LH_FOOT")
        self.hr_index = self.pin_robot.model.getFrameId("RH_FOOT")
        self.fl_index = self.pin_robot.model.getFrameId("LF_FOOT")
        self.fr_index = self.pin_robot.model.getFrameId("RF_FOOT")

    def update_pinocchio(self, q, dq):
        """Updates the pinocchio robot.
        This includes updating:
        - kinematics
        - joint and frame jacobian
        - centroidal momentum
        Args:
          q: Pinocchio generalized position vector.
          dq: Pinocchio generalize velocity vector.
        """
        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

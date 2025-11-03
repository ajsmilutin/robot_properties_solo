"""solo12wrapper

Solo12 interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""
import numpy as np
from robot_properties_solo.config import UnitreeGo2Config

dt = 1e-3

class UnitreeGo2Robot():
    """
    Similar12 robot used for ROS + Gazebo projects
    """
    def __init__(self):

        self.urdf_path = UnitreeGo2Config.urdf_path
        self.mjcf_path = UnitreeGo2Config.mjcf_path

        # Create the robot wrapper in pinocchio.
        self.pin_robot = UnitreeGo2Config.buildRobotWrapper()

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []

        for leg in ["FL", "FR", "RL", "RR"]:
            controlled_joints += [leg + "_HAA", leg + "_HFE", leg + "_KFE"]
            self.end_eff_ids.append(
                self.pin_robot.model.getFrameId(leg + "_foot")
            )
            self.end_effector_names.append(leg + "_foot")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        self.hl_index = self.pin_robot.model.getFrameId("HL_ANKLE")
        self.hr_index = self.pin_robot.model.getFrameId("HR_ANKLE")
        self.fl_index = self.pin_robot.model.getFrameId("FL_ANKLE")
        self.fr_index = self.pin_robot.model.getFrameId("FR_ANKLE")

    def forward_robot(self, q=None, dq=None):
        if q is None:
            q, dq = self.get_state()
        elif dq is None:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq, 0*dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def reset_to_initial_state(self) -> None:
        """Reset robot state to the initial configuration (based on Solo12Config)."""
        q0 = np.array(UnitreeGo2Config.initial_configuration)
        dq0 = np.array(UnitreeGo2Config.initial_velocity)
        self.reset_state(q0, dq0)

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

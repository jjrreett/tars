import time
from pathlib import Path

import pybullet as pb
import pybullet_data
import rich

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class JointInfo:
    index: int
    name: str
    type: int
    q_index: int
    u_index: int
    flags: int
    damping: float
    friction: float
    lower_limit: float
    upper_limit: float
    max_force: float
    max_velocity: float
    link_name: str
    axis: Tuple[float, float, float]
    parent_frame_pos: Tuple[float, float, float]
    parent_frame_orn: Tuple[float, float, float, float]
    parent_index: int

    @classmethod
    def from_tuple(cls, data):
        return cls(
            index=data[0],
            name=data[1].decode("utf-8"),
            type=data[2],
            q_index=data[3],
            u_index=data[4],
            flags=data[5],
            damping=data[6],
            friction=data[7],
            lower_limit=data[8],
            upper_limit=data[9],
            max_force=data[10],
            max_velocity=data[11],
            link_name=data[12].decode("utf-8"),
            axis=data[13],
            parent_frame_pos=data[14],
            parent_frame_orn=data[15],
            parent_index=data[16],
        )


@dataclass
class JointState:
    position: float
    velocity: float
    reaction_forces: Tuple[float, float, float, float, float, float]
    motor_torque: float

    @classmethod
    def from_tuple(cls, data):
        return cls(
            position=data[0],
            velocity=data[1],
            reaction_forces=data[2],
            motor_torque=data[3],
        )


def clamp(x, high, low):
    return min(max(x, low), high)


from plot import LivePlottingApp

# plot = LivePlottingApp(width=1200, mod=1)

# plot.set_names(["rightleg", "rightshuttle"])
# plot.set_colors([(0, 0, 255), (255, 0, 0)])

# try:
#     while not plot.is_quit():

#         pb.resetSimulation()
#         plane_id = pb.loadURDF(str(pybullet_data_path / "plane.urdf"))
#         tars_id = pb.loadURDF(str(parent / "tars.urdf"), start_pos, start_orientation)
#         pb.setGravity(0, 0, -10)
#         # initial control
#         pb.setJointMotorControlArray(tars_id, leg_control_vector_indicies, controlMode=pb.VELOCITY_CONTROL, targetVelocities=[0.0]*len(leg_control_vector_indicies), velocityGains=[1]*len(leg_control_vector_indicies))
#         pb.setJointMotorControlArray(tars_id, shuttle_control_vector_indicies, controlMode=pb.VELOCITY_CONTROL, targetVelocities=[0.0]*len(shuttle_control_vector_indicies), velocityGains=[3]*len(shuttle_control_vector_indicies))

#         frame_rate = 240.0
#         dt = 1.0 / frame_rate
#         total_time = 5
#         for t in np.arange(0, total_time, dt):
#             pb.stepSimulation()
#             plot.clock.tick(frame_rate)

#             pb.setJointMotorControl2(tars_id, joints["rightleg"].index, controlMode=pb.VELOCITY_CONTROL, targetVelocity=clamp(-t+1, .1, 0))

#             joint_states = pb.getJointStates(tars_id, control_vector_indicies)
#             joints_state: dict[str, JointState] = {joints_inv[joint_index]: JointState.from_tuple(joint_state) for joint_index, joint_state in zip(control_vector_indicies, joint_states)}
#             plot.feed(joints_state[name].motor_torque for name in plot.names)


# finally:
#     pb.disconnect()

# rich.print(joints)
# rich.print(joints_state)


frame_rate = 240.0


def run_simulation():
    here = Path(__file__)
    parent = here.parent
    pybullet_data_path = Path(pybullet_data.getDataPath())
    physics_client = pb.connect(pb.GUI)

    plane_id = pb.loadURDF(str(pybullet_data_path / "plane.urdf"))

    start_pos = [0, 0, 0.4]
    start_orientation = pb.getQuaternionFromEuler([0, 0, 0])
    tars_id = pb.loadURDF(str(parent / "tars.urdf"), start_pos, start_orientation)

    joints: dict[str, JointInfo] = {}
    for i in range(pb.getNumJoints(tars_id)):
        joint_info = JointInfo.from_tuple(pb.getJointInfo(tars_id, i))
        joints[joint_info.name] = joint_info

    joints_inv = {joints[name].index: name for name in joints}

    rich.print(joints)

    leg_control_vector_indicies = [
        joints["rightrightleg"].index,
        joints["rightleg"].index,
        joints["leftleg"].index,
        joints["leftleftleg"].index,
    ]

    shuttle_control_vector_indicies = [
        joints["rightrightshuttle"].index,
        joints["rightshuttle"].index,
        joints["leftshuttle"].index,
        joints["leftleftshuttle"].index,
    ]
    control_vector_indicies = (
        leg_control_vector_indicies + shuttle_control_vector_indicies
    )

    # pb.resetSimulation()
    # plane_id = pb.loadURDF(str(pybullet_data_path / "plane.urdf"))
    # tars_id = pb.loadURDF(str(parent / "tars.urdf"), start_pos, start_orientation)
    pb.setGravity(0, 0, -10)
    # initial control
    pb.setJointMotorControlArray(
        tars_id,
        leg_control_vector_indicies,
        controlMode=pb.VELOCITY_CONTROL,
        targetVelocities=[0.0] * len(leg_control_vector_indicies),
        velocityGains=[1] * len(leg_control_vector_indicies),
    )
    pb.setJointMotorControlArray(
        tars_id,
        shuttle_control_vector_indicies,
        controlMode=pb.VELOCITY_CONTROL,
        targetVelocities=[0.0] * len(shuttle_control_vector_indicies),
        velocityGains=[1] * len(shuttle_control_vector_indicies),
    )

    plot = LivePlottingApp(width=1200, mod=1)
    plot.set_names(
        [
            "leftshuttle.position",
            "leftshuttle.motor_torque",
            "rightshuttle.position",
            "rightshuttle.motor_torque",
        ]
    )
    plot.set_colors([(0, 0, 255), (255, 0, 0), (0, 255, 0), (255, 0, 255)])

    dt = 1.0 / frame_rate
    total_time = 5

    set_point = -0.1
    kp = 1

    # Simulation setup code here
    for t in np.arange(0, total_time, dt):
        pb.stepSimulation()

        # Send data to plotting process
        joint_states = pb.getJointStates(tars_id, control_vector_indicies)
        joints_state = {
            joints_inv[joint_index]: JointState.from_tuple(joint_state)
            for joint_index, joint_state in zip(control_vector_indicies, joint_states)
        }

        if t == 0.0:
            rich.print(joints_state)

        # Simulation code here
        pb.setJointMotorControl2(
            tars_id,
            joints["rightrightleg"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=kp * (set_point - joints_state["rightrightleg"].position),
        )
        pb.setJointMotorControl2(
            tars_id,
            joints["rightleg"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=kp * (0 - joints_state["rightleg"].position),
        )
        pb.setJointMotorControl2(
            tars_id,
            joints["leftleg"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=kp * (0 - joints_state["leftleg"].position),
        )
        pb.setJointMotorControl2(
            tars_id,
            joints["leftleftleg"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=kp * (set_point - joints_state["leftleftleg"].position),
        )

        pb.setJointMotorControl2(
            tars_id,
            joints["rightrightshuttle"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=0,
        )
        pb.setJointMotorControl2(
            tars_id,
            joints["rightshuttle"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=0,
        )
        pb.setJointMotorControl2(
            tars_id,
            joints["leftshuttle"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=10,
        )
        pb.setJointMotorControl2(
            tars_id,
            joints["leftleftshuttle"].index,
            controlMode=pb.VELOCITY_CONTROL,
            targetVelocity=0,
        )

        plot.feed(
            [
                joints_state["leftleftshuttle"].motor_torque,
                joints_state["leftshuttle"].motor_torque,
                joints_state["rightshuttle"].motor_torque,
                joints_state["rightrightshuttle"].motor_torque,
            ]
        )

        # plot.feed(joints_state[name].position for name in plot.names)
        plot.clock.tick(frame_rate)

    time.sleep(5)
    pb.disconnect()
    plot.close()


if __name__ == "__main__":
    run_simulation()

import pybullet as p
import os
import numpy as np

class Panda:
    def __init__(self, control_mode="torque", realtime=0, render = False):
        self.realtime = realtime
        self.control_mode = control_mode
        self.position_control_gain_p = [0.01,0.01,0.01,0.01,0.01,0.01,0.01]
        self.position_control_gain_d = [1.0,1.0,1.0,1.0,1.0,1.0,1.0]
        self.max_torque = [87,87,87,87,12,12,12]

        # connect pybullet
        if render:
            p.connect(p.GUI)
            #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            #p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.5])
        else:
            p.connect(p.DIRECT)
            

        p.resetSimulation()
        p.setRealTimeSimulation(self.realtime)

        # load models
        p.setAdditionalSearchPath(os.path.join(os.path.dirname(__file__), "../models"))

        self.plane = p.loadURDF("plane/plane.urdf",
                                useFixedBase=True)
        p.changeDynamics(self.plane,-1,restitution=.95)

        self.robot = p.loadURDF("panda/panda.urdf",
                                useFixedBase=True,
                                flags=p.URDF_USE_SELF_COLLISION)
        
        # robot parameters
        self.dof = p.getNumJoints(self.robot) - 1 # Virtual fixed joint between the flange and last link
        if self.dof != 7:
            raise Exception('wrong urdf file: number of joints is not 7')

        self.joints = []
        self.q_min = []
        self.q_max = []
        self.target_pos = []
        self.target_torque = []

        for j in range(self.dof):
            joint_info = p.getJointInfo(self.robot, j)
            self.joints.append(j)
            self.q_min.append(joint_info[8])
            self.q_max.append(joint_info[9])
            self.target_pos.append((self.q_min[j] + self.q_max[j])/2.0)
            self.target_torque.append(0.)

        self.reset()

    def reset(self, target_endeffector_loc = None):
        
        if target_endeffector_loc is not None:
            assert len(target_endeffector_loc) == 3
            target_pos = self.solveInverseKinematics(target_endeffector_loc, [1,0,0,0])
            assert len(target_pos) == 7
            for j in range(self.dof):
                p.resetJointState(self.robot,j,targetValue=target_pos[j])
            
        else:
            for j in range(self.dof):
                self.target_pos[j] = (self.q_min[j] + self.q_max[j])/2.0
                p.resetJointState(self.robot,j,targetValue=self.target_pos[j])

        self.setTargetVelocities([0. for i in range(self.dof)])
        self.setTargetTorques([0. for i in range(self.dof)])
        
        if self.control_mode == "torque":
            for j in range(self.dof):
                mode = p.VELOCITY_CONTROL
                p.setJointMotorControl2(self.robot, j,
                    controlMode=mode, force=0)
        else:
            raise NotImplementedError("Control mode not implemented")
        
        
    
    def step(self):
        p.stepSimulation()
    
    def setTargetPositions(self, target_pos):
        self.target_pos = target_pos
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=self.target_pos,
                                    forces=self.max_torque,
                                    positionGains=self.position_control_gain_p,
                                    velocityGains=self.position_control_gain_d)

    def setTargetTorques(self, target_torque):
        self.target_torque = target_torque
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=self.target_torque)
        
    def setTargetVelocities(self, target_vel):
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                    jointIndices=self.joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocities=target_vel,
                                    forces=self.max_torque,
                                    positionGains=self.position_control_gain_p,
                                    velocityGains=self.position_control_gain_d)

    def getJointStates(self):
        joint_states = p.getJointStates(self.robot, self.joints)
        joint_pos = [x[0] for x in joint_states]
        joint_vel = [x[1] for x in joint_states]
        joint_torque = [x[3] for x in joint_states]
        return joint_pos, joint_vel, joint_torque

    def solveInverseDynamics(self, pos, vel, acc):
        return list(p.calculateInverseDynamics(self.robot, pos, vel, acc))

    def solveInverseKinematics(self, pos, ori):
        return list(p.calculateInverseKinematics(self.robot, 7, pos, ori))
    
    def getEndEffectorCoordinates(self):
        return p.getLinkState(self.robot, 7)[4]
    
    def getEndeffectorVelocity(self):
        linkState = p.getLinkState(self.robot, 7, computeLinkVelocity=1)
        
        # getLinkState will only return endeffector velocity if it is greater than 0
        assert len(linkState) == 8, "linkState has wrong length"
        
        return np.array(linkState[6])
    
    def getPybulletId(self):
        return self.robot
    
    def _read_jacobian(self):
        _, joint_vel, _ = self.getJointStates()
        linear_jacobian, angular_jacobian = p.calculateJacobian(
            self.robot, 7, [0, 0, 0], list(joint_vel), [0]*9, [0]*9) # TODO: 9 joints--might cause problems
        linear_jacobian = np.asarray(linear_jacobian)[:, :7]
        angular_jacobian = np.asarray(angular_jacobian)[:, :7]
        full_jacobian = np.zeros((6, 7))
        full_jacobian[0:3, :] = linear_jacobian
        full_jacobian[3:6, :] = angular_jacobian
        self.jacobian['full_jacobian'] = full_jacobian
        self.jacobian['linear_jacobian'] = linear_jacobian
        self.jacobian['angular_jacobian'] = angular_jacobian

if __name__ == "__main__":
    robot = Panda(realtime=1)
    while True:
        pass

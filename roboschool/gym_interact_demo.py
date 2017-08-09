from roboschool.scene_abstract import SingleRobotEmptyScene
from roboschool.gym_mujoco_xml_env import RoboschoolMujocoXmlEnv
from roboschool.scene_abstract import cpp_household
from scene_conf import scene_config
import gym
import gym.spaces
import gym.utils
import gym.utils.seeding
import numpy as np
import os
import sys
import json

class InputHandler:
    def __init__(self, robot, pose, actions):
        """
        what to expect from InputHandler:
            1. the pose handler for the main robot
            2. the camera handler for following the main robot
            3. the state of the main robot
        """
        self.init_step = 1
        self.robot = robot
        self.pose = pose
        self.camera_phi = np.random.rand() * 2 * np.pi - np.pi
        self.speed_norm = 2.0
        self.speed = np.array([0.0, 0.0, 0.0])
        self.pitch_count = 0
        self.camera_mode = False # true: use third person
        self.actions = actions

    def update_camera(self):
        x, y, z = self.pose.xyz()
        if self.camera_mode:
            scale = 0.13
            self.camera_pos = [
                x - self.speed_x * scale, y - self.speed_y * scale, z + 0.2,
                x, y, z + 0.1
            ]
        else:
            scale = 0.025
            self.camera_pos = [
                x, y, z, x + self.speed_x, y + self.speed_y, z,
            ]

    def apply_action(self, a, contacted):
        """
        action:
            1. 'w': move forward
            3. 'a': turn left
            4. 'd': turn right
            5. 'j': jump
        """
        if self.init_step == 1:
            self.init_step = 0
            current_speed_z = 0.0
        else:
            self.pose = self.robot.pose()
            current_speed_z = self.robot.speed()[2]
        # print (self.pose.xyz())
        if a[0] == 'a' and 'a' in self.actions:
            self.camera_phi += np.pi / 6
        elif a[0] == 'd' and 'd' in self.actions:
            self.camera_phi -= np.pi / 6
        if self.camera_phi > np.pi:
            self.camera_phi -= 2 * np.pi
        if self.camera_phi < -np.pi:
            self.camera_phi += 2 * np.pi
        self.speed_x = self.speed_norm * np.cos(self.camera_phi)
        self.speed_y = self.speed_norm * np.sin(self.camera_phi)
        self.speed = np.array([self.speed_x, self.speed_y, current_speed_z])
        self.update_camera()
        if a[0] == 'w' and 'w' in self.actions:
            self.pitch_count += 1
            self.pose.set_rpy(0.0, self.pitch_count * \
                              self.speed_norm * 0.165 / np.pi, self.camera_phi)
            self.robot.set_pose_and_speed(self.pose, *self.speed)
        else:
            self.pose.set_rpy(0.0, self.pitch_count * \
                              self.speed_norm * 0.165 / np.pi, self.camera_phi)
            self.robot.set_pose_and_speed(self.pose, 0., 0., current_speed_z)
        collect = a[0] == 'c' and 'c' in self.actions
        if a[0] == 'z': self.camera_mode = not self.camera_mode
        if a[0] == 'j' and 'j' in self.actions and contacted:
            self.speed = np.array([0., 0., 3.0])
            self.robot.set_pose_and_speed(self.pose, *self.speed)
        return collect

    def get_direction(self):
        return self.speed_x / self.speed_norm, self.speed_y / self.speed_norm


class SceneBuilder:
    def __init__(self, scene, history):
        """
        what to expect:
            1. parse what the scene_config builds
        """
        self.scene = scene
        self.build_list, self.max_step, self.actions = scene_config(history)
        self.parse()

    def parse(self):
        self.b, self.g, self.m = [], [], []
        for item in self.build_list:
            if item[3] == 'agent':
                self.ax, self.ay, self.az = item[0], item[1], item[2]
                self.amodel = item[5]
            if item[3] == 'movable':
                self.m.append([item[0], item[1], item[2], item[4], item[5]])
            if item[3] == 'goal':
                self.g.append([item[0], item[1], item[2], item[4], item[5]])
            if item[3] == 'block':
                self.b.append([item[0], item[1], item[2], item[4], item[5]])
        self.unit = 0.1 # pre-defined unit length -- useful for collision

    def build(self):
        apose = cpp_household.Pose()
        dirname = os.path.dirname
        apose.set_xyz(self.ax * self.unit, self.ay * self.unit, self.az * self.unit)
        self.agent = self.scene.cpp_world.load_urdf(os.path.join(
            dirname(__file__), self.amodel), apose, False, False)

        cpose = cpp_household.Pose()
        self.b_list = []
        for b in self.b:
            cpose.set_xyz(b[0] * self.unit, b[1] * self.unit, b[2] * self.unit)
            cpose.set_rpy(0.0, 0.0, 0.0)
            self.b_list.append(self.scene.cpp_world.load_urdf(os.path.join(
                dirname(__file__), b[4]
            ), cpose, False, False))

        self.g_list, g_idx = [], 0
        for g in self.g:
            cpose.set_xyz(g[0] * self.unit, g[1] * self.unit, g[2] * self.unit)
            cpose.set_rpy(0.0, 0.0, 0.0)
            self.g_list.append(self.scene.cpp_world.load_urdf(os.path.join(
                dirname(__file__), g[4]
            ), cpose, False, False))
            self.g_list[-1].root_part.name = str(g_idx)
            self.g_list[-1].alive = True
            g_idx += 1

        self.m_list = []
        for m in self.m:
            cpose.set_xyz(m[0] * self.unit, m[1] * self.unit, m[2] * self.unit)
            cpose.set_rpy(0.0, 0.0, 0.0)
            self.m_list.append(self.scene.cpp_world.load_urdf(os.path.join(
                dirname(__file__), m[4]
            ), cpose, False, False))

        return self.agent, apose, self.max_step, self.actions


class RoboschoolDemo(RoboschoolMujocoXmlEnv):
    def __init__(self):
        RoboschoolMujocoXmlEnv.__init__(
            self, 'demo.xml', 'floor', action_dim=1, obs_dim=1)
        self.history = []

    def create_single_player_scene(self):
        return SingleRobotEmptyScene(gravity=98 * 0.5, timestep=0.00165, frame_skip=1)

    def robot_specific_reset(self):
        # load the ground
        stadium_pose = cpp_household.Pose()
        stadium_pose.set_xyz(0, 0, -0.05)
        self.stadium = self.scene.cpp_world.load_thingy(
            os.path.join(os.path.dirname(__file__),
                         "models_outdoor/stadium/pong1.obj"),
            stadium_pose, 0.01, 0, 0xFFFFFF, True)

        self.scene_builder = SceneBuilder(self.scene, self.history)
        self.agent, apose, self.max_step, actions = self.scene_builder.build()
        self.input_handler = InputHandler(self.agent, apose, actions)
        # set agent camera_phi
        goal = self.scene_builder.g[0]
        dx, dy = self.scene_builder.ax - goal[0], self.scene_builder.ay - goal[1]
        self.input_handler.camera_phi = np.arctan2(-dy, -dx)
        self.step_count = 0

    def apply_action(self, a, contacted):
        return self.input_handler.apply_action(a, contacted)

    def calc_state(self):
        return np.array([1, ])

    def _step(self, a):
        self.step_count += 1
        contact_list = self.agent.root_part.contact_list()
        collect = self.apply_action(a, len(contact_list) != 0)
        for i in range(10):
            self.scene.global_step()
        goal_collected, cossim = self.goal_step(collect)

        state = self.calc_state()
        if goal_collected:
            self.rewards = [1.0, ]
        elif collect:
            self.rewards = [-0.2, ]
        else:
            self.rewards = [-0.1, ]
        self.HUD(state, a, False)

        # done: 0 -- dead 1 -- alive 2 -- success
        done = 2
        for g in self.scene_builder.g_list:
            if g.alive == True:
                done = 1
                break
        if self.step_count > self.max_step:
        # if self.step_count > 2048:
            done = 0
        # update history
        if done == 2:
            self.history.append(1)
        elif done == 0:
            self.history.append(0)
        if done == 0:
            self.rewards = [-1.0, ]
        return state, sum(self.rewards), done, {}

    def camera_adjust(self):
        # camera follow the ball
        p = self.input_handler.camera_pos
        self.camera.move_and_look_at(p[0], p[1], p[2], p[3], p[4], p[5])

    def goal_step(self, collect):
        goal_collected = False
        goal_list = self.scene_builder.g_list
        idx, cossim = -1, -1
        if collect:
            ax, ay, az = self.agent.pose().xyz()[0:3]
            tx, ty = self.input_handler.get_direction()
            g_idx = -1
            for g in goal_list:
                g_idx += 1
                gx, gy, gz = g.pose().xyz()[0:3]
                dx, dy, dz = gx - ax, gy - ay, abs(gz - az)
                dist = np.sqrt(dx ** 2 + dy ** 2)
                if not (dist < 0.12 and dz < 0.05):
                    continue
                dx, dy = dx / dist, dy / dist
                sim = dx * tx + dy * ty
                if sim > 0.5 and sim > cossim:
                    cossim = sim
                    idx = g_idx
        if idx >= 0:
            goal_list[idx].alive = False
            goal_collected = True

        height = 0.01
        speed_norm = 0.03
        z = height - height * np.cos(speed_norm * self.step_count)
        yaw = speed_norm * self.step_count * 0.1
        for g in goal_list:
            gpose = g.pose()
            x, y = gpose.xyz()[0:2]
            r, p = gpose.rpy()[0:2]
            gpose.set_rpy(r, p, yaw)
            if g.alive:
                # gpose.set_xyz(x, y, z)
                # g.set_pose_and_speed(gpose, 0.0, 0.0, 0.0)
                pass
            else:
                gz = gpose.xyz()[2]
                if gz < 100:
                    g.set_pose_and_speed(gpose, 0., 0., 4.0)
                else:
                    g.set_pose_and_speed(gpose, 0., 0., 0.0)
        return goal_collected, cossim

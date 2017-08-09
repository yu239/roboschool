import gym
import roboschool
import numpy as np
import cv2

def demo_run():
    env = gym.make("RoboschoolDemo-v0")
    while 1:
        obs = env.reset()
        a = np.array(['s',])

        while 1:
            obs, r, done, _ = env.step(a)
            rgb = cv2.cvtColor(env.render("rgb_array"), cv2.COLOR_RGB2BGR)
            cv2.imshow('image', rgb)
            a = np.array([chr(cv2.waitKey(0)),])
            if done == 1: continue
            break

if __name__ == "__main__":
    demo_run()

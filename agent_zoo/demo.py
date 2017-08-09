import gym
import roboschool
import numpy as np

def demo_run():
    env = gym.make("RoboschoolDemo-v0")
    while 1:
        obs = env.reset()

        while 1:
            a = np.array([1, ])
            obs, r, done, _ = env.step(a)
            still_open = env.render("human")
            if still_open == False:
                return
            if not done: continue
            break

if __name__ == "__main__":
    demo_run()

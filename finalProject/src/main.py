from environment import *
from formation_planner import *
from robot import *
import time



env_params = {
    "env_height"  : 400,
    "env_width"   : 600,
    "nBots"     : 20,
    "base_pose" : [10,10],
    "window_name": "display"
}


if __name__ == "__main__":
    env = environment(env_params)
    n=0
    for i in range(len(env.robots)):
        if i%3==0:
            env.robots[i].consensus.set_active(n)
            env.robots[i].consensus.set_displacement(5)
            n=n+1

    i=0
    while True:
        env.update_env()
        if i%10==0:
            env.draw_env()
        i+=1
        cv2.waitKey(100)

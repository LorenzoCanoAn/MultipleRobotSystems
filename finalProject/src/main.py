from environment import *
from formation_planner import *
from robot import *
import time



env_params = {
    "base_pose":[10,10]
}


if __name__ == "__main__":
    "ahora mismo, iniciamos el environment con 20 robots cargados al 100% de bateria repartidos uniformemente por el environment"
    env = environment(env_params)
    n=0
    for i in range(len(env.robots)):
        if i%3==0:
            env.robots[i].consensus.set_active(n)
            env.robots[i].consensus.set_displacement()
            n=n+1

    #for i in env.robots:
        #env.robots[i].GoTo.set_goal(np.array([10,10]))
    i=0
    while True:
        env.update_env()
        if i%10==0:
            env.draw_env()
        #print('neighbours:', env.neighbours_information)
        print(env.robots[0].battery.battery)
        i=i+1
        cv2.waitKey(100)

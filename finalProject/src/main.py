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

    #for i in env.robots:
        #env.robots[i].GoTo.set_goal(np.array([10,10]))
    for i in range(2000):
        env.draw_env()
        env.update_robots()

        #print('neighbours:', env.neighbours_information)
        print(env.robots[0].battery.battery)
        cv2.waitKey(100)

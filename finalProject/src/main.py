from environment import *
from formation_planner import *
from robot import *



env_params = {
    "base_pose":[10,10]
}


if __name__ == "__main__":
    "ahora mismo, iniciamos el environment con 20 robots cargados al 100% de bateria repartidos uniformemente por el environment"
    env = environment(env_params)

    for i in range(20):
        env.draw_env()
        env.update_robots()

        print('neighbours:', env.neighbours_information)
        cv2.waitKey(0)
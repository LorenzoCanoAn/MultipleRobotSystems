from environment import *
from path_planner import *
from robot import *



env_params = {
    "base_pose":[20,20]
}


if __name__ == "__main__":
    
    env = environment(env_params)

    cv2.waitKey(0)
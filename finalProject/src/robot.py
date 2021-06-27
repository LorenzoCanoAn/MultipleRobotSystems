import numpy as np
import matplotlib.pyplot as plt
from random import randint


# Mensaje entre robots que tnga un timestamp y contenga la direccion d la formacion, el gap  y el centro

class battery:
    def __init__(self, robot, C=0.2, K=0.028, init_battery=100):

        self.robot = robot
        self.dT = robot.dT
        self.charging = False

        # Battery model:  battery[i+1]=battery[i]-K * v[i+1]- C * abs(v[i+1]-v[i]), con i+1 un segundo despues de i
        self.battery = init_battery #+ -randint(50, 70)
        self.K = K  # percentage/((m/s))
        self.C = C  # percentage/((m/s))
        self.charging_threshold = 20

        self.charging_rate = 30  # percentage/second
        self.remaining = (self.battery > 0)

    def update(self):

        if self.robot.state == 'charging':
            self.charging == True
            self.battery = self.battery + self.charging_rate * self.robot.dT
        else:
            self.charging == False
            if not self.robot.state == "waiting_active":
                self.battery = self.battery - self.K * np.linalg.norm(
                    self.robot.velocity) * self.dT - self.C * np.linalg.norm(
                    self.robot.velocity - self.robot.velocity_hist[-1]) * self.dT

        if self.battery < 0:
            self.battery = 0

        if self.battery > 100:
            self.battery = 100

        if self.battery < self.charging_threshold:
            return False
        else:
            return True


class GoTo:
    def __init__(self, robot):
        self.robot = robot
        self.active = False
        self.goal = None
        self.tolerance = 2
        self.v_max = robot.v_max
        self.a_max = robot.a_max
        self.dT = robot.dT

    def set_goal(self, goal):
        self.active = True
        self.goal = goal

    def stop(self):
        self.active = False
        self.goal = None

    def check_arrive(self, position):
        if np.linalg.norm(position-self.goal) < self.tolerance:
            self.stop()
            return True
        else:
            return False

    def compute_vel(self):
        v = 0.1 * (self.goal - self.robot.position)

        if np.linalg.norm(v) > self.v_max:
            v = v / np.linalg.norm(v)
            v = v * self.v_max

        a = (v - self.robot.velocity) / self.dT
        if np.linalg.norm(a) > self.a_max:
            a = a / np.linalg.norm(a)
            a = a * self.a_max
            v = self.robot.velocity + a * self.dT
        return v


class Consensus:
    def __init__(self, robot):

        self.robot = robot
        self.environment = self.robot.environment

        self.index = self.robot.index
        self.active = False
        self.tolerance = 0.5
        self.v_max = self.robot.v_max
        self.a_max = self.robot.a_max
        self.dT = self.robot.dT
        self.num_order = 0
        self.displacement = np.array([0, 0])

    def set_active(self, num_order):
        self.num_order = num_order
        self.active = True

    def set_off(self):
        self.num_order = 0
        self.active = False

    def compute_goal(self):
        # position:
            nei = self.environment.neighbours_information[self.index]
            sum = 0
            num = 0
            for i in range(len(nei)):
                if self.environment.robots[nei[i]].state in ['active', "going_formation", "waiting_active"]:
                    sum = sum+self.environment.robots[nei[i]].position
                    num = num+1
            self.goal = sum/num+self.displacement

    def set_displacement(self, displacement):
        self.displacement = displacement

    def compute_vel(self, position, velocity):
        self.compute_goal()
        v = 0.1 * (self.goal - position)
        if np.linalg.norm(v) > self.v_max:
            v = v / np.linalg.norm(v)
            v = v * self.v_max

        a = (v - velocity) / self.dT
        if np.linalg.norm(a) > self.a_max:
            a = a / np.linalg.norm(a)
            a = a * self.a_max
            v = velocity + a * self.dT
        return v


class robot:
    def __init__(self, environment, index, base_pose, init_battery=100, init_position=np.array([0, 0]), radius=11, dT=1, connection_radius=800):
        self.base_pose = base_pose
        self.index = index
        self.connection_radius = connection_radius
        self.radius = radius
        self.position = init_position
        self.velocity = np.array([0,0])
        self.position_hist = [init_position]
        self.velocity_hist = [np.array([0, 0])]
        self.dT = dT
        self.environment = environment

        self.v_max = 7  # (m/s)
        self.a_max = 1  # (m/s^2)

        self.state = 'waiting_base'  # possible states: Active, Waiting,  Returning, Charging
        self.consensus = Consensus(self)
        self.battery = battery(self, init_battery=init_battery)
        self.GoTo = GoTo(self)
        self.neighbours = []


        self.state_functions = {
            "active": self.active,
            "going_base": self.going_base,
            "charging": self.charging,
            "waiting_base": self.waiting_base,
            "going_formation": self.going_formation,
            "waiting_active": self.waiting_active
        }

    ###########################################
    # UTILITY FUNCTIONS
    ###########################################

    def update(self):
        self.neighbours = self.environment.neighbours_information[self.index]
        self.state_functions[self.state]()

        if not self.battery.update():
            if self.state != "going_base":
                self.change_state("going_base")

        self.velocity_hist.append(self.velocity)
        self.position_hist.append(self.position)

    def change_state(self, state, transition=1):
        self.state = state
        self.state_functions[state](transition=transition)

    ###########################################
    # STATE FUNCTIONS
    ###########################################
    # ---------------------------------------
    # 1 active state
    # ---------------------------------------
    def active(self, transition=None):
        if type(transition) != type(None):
            self.consensus.active == True
        else:
            self.velocity = self.consensus.compute_vel(
                self.position, self.velocity)
            self.position = self.position_hist[-1] + self.velocity*self.dT

    # ---------------------------------------
    # 2 going_base state
    # ---------------------------------------
    def going_base(self, transition=None):
        if type(transition) != type(None):
            for neighbor in self.neighbours:
                if neighbor == self.index:
                    pass
                else:
                    self.environment.robots[neighbor].change_state("waiting_active")
            self.environment.leaving_info = [self.consensus.num_order, self.position]
            print("I go to base, next robot has: {}".format(self.environment.leaving_info))

            self.consensus.active == False
            self.GoTo.active == True
            self.GoTo.set_goal(self.base_pose)

        self.velocity = self.GoTo.compute_vel()
        self.position = self.position_hist[-1] + self.velocity*self.dT

        arrive = self.GoTo.check_arrive(self.position_hist[-1])
        if arrive:
            self.change_state('charging')

    # ---------------------------------------
    # 3 charging state
    # ---------------------------------------
    def charging(self, transition=None):
        if type(transition) != type(None):
            pass
        self.velocity = np.array([0, 0])
        self.position = self.position_hist[-1] + self.velocity*self.dT
        if self.battery.battery == 100:
            self.change_state("waiting_base")

    # ---------------------------------------
    # 4 waiting_base state
    # ---------------------------------------
    def waiting_base(self, transition=None):
        if type(transition) != type(None):
            pass
        self.velocity = np.array([0, 0])
        self.position = self.position_hist[-1] + self.velocity*self.dT

        if self.battery.battery <= 70:
            self.change_state("charging")

    # ---------------------------------------
    # 5 going_formation state
    # ---------------------------------------
    def going_formation(self, transition=None):
        if type(transition) != type(None):
            self.GoTo.set_goal(transition)

        self.velocity = self.GoTo.compute_vel()
        self.position = self.position_hist[-1] + self.velocity*self.dT

        if self.GoTo.check_arrive(self.position):
            self.change_state("active")
            for neigh in self.neighbours:
                self.environment.robots[neigh].change_state("active")

    # ---------------------------------------
    # 6 waiting_active state
    # ---------------------------------------

    def waiting_active(self, transition=None):
        if type(transition) != type(None):
            pass
            self.GoTo.set_goal(self.position+np.array([0,0.01]))
        self.velocity = self.GoTo.compute_vel()#np.array([0,0]) #
        self.position = self.position_hist[-1] + self.velocity*self.dT
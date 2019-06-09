import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

import math
import time
import sys
sys.path.insert(0, '../src')
from robot import Robot


class wall_following():
    def __init__(self):
        self.NUM_SENSORS = 8
        self.max_speed = math.pi
        self.fast_speed = 2 * self.max_speed
        self.steps = 0.01

    def init_fuzzy(self):
        distance = []
        for i in range(0, self.NUM_SENSORS):
            distance.append(ctrl.Antecedent(np.arange(0, 5.0, self.steps), 's' + str(i)))

            distance[i]['danger']   = fuzz.trimf(distance[i].universe, [0.0, 0.0, 0.1])
            distance[i]['close']    = fuzz.trapmf(distance[i].universe, [0.0, 0.0, 0.15, 0.2])
            distance[i]['ideal']    = fuzz.trapmf(distance[i].universe, [0.15, 0.2, 0.3, 0.5])
            distance[i]['away']     = fuzz.trapmf(distance[i].universe, [0.3, 0.5, 0.6, 0.7])
            distance[i]['nothing']  = fuzz.trapmf(distance[i].universe, [0.6, 0.7, 5.0, 5.0])

        v_left  = ctrl.Consequent(np.arange(-self.max_speed, 1.0 + self.max_speed, 0.01), 'vl')
        v_right = ctrl.Consequent(np.arange(-self.max_speed, 1.0 + self.max_speed, 0.01), 'vr')

        v_right['fast']         = fuzz.trimf(v_left.universe, [0.1, self.max_speed + 0.1, self.max_speed + 0.1])
        v_right['foward']       = fuzz.trimf(v_left.universe, [0.0, self.max_speed, self.max_speed])
        v_right['slowTurning']  = fuzz.trimf(v_left.universe, [0.0, self.max_speed - 0.1, self.max_speed - 0.1])
        v_right['turning']      = fuzz.trimf(v_left.universe, [0.0, self.max_speed - 0.5, self.max_speed - 0.5])
        v_right['reverse']      = fuzz.trimf(v_left.universe, [-self.max_speed, -self.max_speed, 0.0])

        v_left['fast']          = fuzz.trimf(v_right.universe, [0.3, self.max_speed + 0.3, self.max_speed + 0.3])
        v_left['foward']        = fuzz.trimf(v_right.universe, [0.0, self.max_speed, self.max_speed])
        v_left['slowTurning']   = fuzz.trimf(v_right.universe, [0.0, self.max_speed - 0.1, self.max_speed - 0.1])
        v_left['turning']       = fuzz.trimf(v_right.universe, [0.0, self.max_speed - 0.5, self.max_speed - 0.5])
        v_left['reverse']       = fuzz.trimf(v_right.universe, [-self.max_speed + 0.1, -self.max_speed + 0.1, 0.0])

        rule_Wall = []
        #PROCURAR PAREDE
        #ESQUERDA
        rule_Wall.append(ctrl.Rule(distance[0]['nothing'] & distance[1]['nothing'] & distance[2]['nothing'] & distance[3]['nothing'] & distance[4]['nothing'] & distance[5]['nothing'] & distance[6]['nothing'] & distance[7]['nothing'], v_left['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['nothing']| distance[3]['away'] | distance[4]['nothing'] | distance[4]['nothing'], v_left['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & distance[7]['close'], v_left['reverse']))
        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & distance[7]['close'], v_left['reverse']))
        
        #DIREITA
        rule_Wall.append(ctrl.Rule(distance[0]['nothing'] & distance[1]['nothing'] & distance[2]['nothing'] & distance[3]['nothing'] & distance[4]['nothing'] & distance[5]['nothing'] & distance[6]['nothing'] & distance[7]['nothing'], v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['nothing']| distance[3]['away'] | distance[4]['nothing'] | distance[4]['nothing'], v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & distance[7]['close'], v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & distance[7]['close'], v_right['foward']))

        #SEGUIR PAREDE
        #DIREITA
        rule_Wall.append(ctrl.Rule(distance[7]['ideal'] , v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[7]['close'] , v_right['fast']))

        #ESQUERDA
        rule_Wall.append(ctrl.Rule(distance[7]['ideal'] , v_left['foward']))
        rule_Wall.append(ctrl.Rule(distance[7]['close'] , v_left['turning']))

        #QUINA
        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & (distance[7]['ideal'] | distance[7]['close']) , v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & (distance[7]['ideal'] | distance[7]['close']) , v_right['foward']))

        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & (distance[7]['ideal'] | distance[7]['close']), v_left['reverse']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & (distance[7]['ideal'] | distance[7]['close']) , v_left['reverse']))

        #PERDEU PAREDE
        rule_Wall.append(ctrl.Rule(distance[7]['ideal'] | distance[7]['close'] & distance[6]['away'], v_right['turning']))
        rule_Wall.append(ctrl.Rule(distance[7]['ideal'] | distance[7]['close'] & distance[6]['away'], v_left['foward']))
        
        rule_Wall.append(ctrl.Rule(distance[7]['danger'] , v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['danger'] | distance[4]['danger'] | distance[6]['danger'] , v_right['foward']))

        rule_Wall.append(ctrl.Rule(distance[7]['danger'] , v_left['reverse']))
        rule_Wall.append(ctrl.Rule(distance[3]['danger'] | distance[4]['danger'] | distance[6]['danger'], v_left['reverse']))

        vel_ctrl = ctrl.ControlSystem(rule_Wall)
        self.fuzzy_system = ctrl.ControlSystemSimulation(vel_ctrl)
        #print(" System Init")

    def get_vel(self, dist):
        for i in range(len(dist)):
            self.fuzzy_system.input['s' + str(i)] = dist[i]

        self.fuzzy_system.compute()
        return [self.fuzzy_system.output['vl'], self.fuzzy_system.output['vr']]



def robotStuck(robot):
    for i in range(5):
        robot.set_left_velocity(-2)
        robot.set_right_velocity(-2)
        time.sleep(0.2)

robot = Robot()
a = wall_following()
a.init_fuzzy()
ultrassonic = robot.read_ultrassonic_sensors()[0:8]
#print("Ultrassonic: ", ultrassonic)
#print("vel: ", a.get_vel(ultrassonic))
oldPos = 0

for x in range(1000):
    ultrassonic = robot.read_ultrassonic_sensors()[0:8]

    pos = robot.get_current_position()

    if (oldPos == pos):
        robotStuck(robot)
    else:
        vel = a.get_vel(ultrassonic)
        #print("Ultrassonic: ", ultrassonic)
        #print("Iteracao: ", x)
        #print("Esquerda: " , ultrassonic[0:3])
        #print("Frente: " , ultrassonic[3:5])
        #print("Direita: " , ultrassonic[5:8])
        #print("vel: ", a.get_vel(ultrassonic))
        robot.set_left_velocity(vel[0])  # rad/s
        robot.set_right_velocity(vel[1])
        time.sleep(0.2)

    oldPos = pos

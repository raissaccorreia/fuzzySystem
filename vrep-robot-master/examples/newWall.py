import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

import math
import time
import sys
sys.path.insert(0, '../src')
from robot import Robot


class wall_following():
    def __init__(self):
        self.NUM_SENSORS = 9
        self.max_speed = math.pi
        self.fast_speed = 2 * self.max_speed
        self.steps = 0.01

    def init_fuzzy(self):
        distance = []
        for i in range(0, self.NUM_SENSORS):
            distance.append(ctrl.Antecedent(np.arange(0, 5.0, self.steps), 's' + str(i)))

            distance[i]['danger']   = fuzz.trimf(distance[i].universe, [0.0, 0.0, 0.15])
            distance[i]['close']    = fuzz.trapmf(distance[i].universe, [0.0, 0.1, 0.15, 0.2])
            distance[i]['ideal']    = fuzz.trapmf(distance[i].universe, [0.15, 0.2, 0.3, 0.5])
            distance[i]['away']     = fuzz.trapmf(distance[i].universe, [0.3, 0.5, 0.6, 0.7])
            distance[i]['nothing']  = fuzz.trapmf(distance[i].universe, [0.6, 0.7, 5.0, 5.0])

        v_left  = ctrl.Consequent(np.arange(-self.max_speed, 1.0 + self.max_speed, 0.01), 'vl')
        v_right = ctrl.Consequent(np.arange(-self.max_speed, 1.0 + self.max_speed, 0.01), 'vr')

        #RODA ESQUERDA
        v_right['foward']       = fuzz.trimf(v_left.universe, [0.0, self.max_speed, self.max_speed])
        v_right['turning']      = fuzz.trimf(v_left.universe, [0.0, self.max_speed - 0.5, self.max_speed - 0.5])
        v_right['reverse']      = fuzz.trimf(v_right.universe, [-self.max_speed + 0.1, -self.max_speed + 0.1, 0.0])
        v_right['lostWall']     = fuzz.trimf(v_left.universe, [-self.max_speed - 0.2, -self.max_speed - 0.2 , 0.0])
        
        #RODA DIREITA
        v_left['foward']        = fuzz.trimf(v_right.universe, [0.0, self.max_speed, self.max_speed])
        v_left['turning']       = fuzz.trimf(v_right.universe, [0.0, self.max_speed - 0.5, self.max_speed - 0.5])
        v_left['reverse']       = fuzz.trimf(v_right.universe, [-self.max_speed + 0.1, -self.max_speed + 0.1, 0.0])
        v_left['lostWall']      = fuzz.trimf(v_right.universe, [0.3, self.max_speed + 0.3, self.max_speed + 0.3])

        rule_Wall = []
        #PROCURAR PAREDE
        #DIREITA
        rule_Wall.append(ctrl.Rule(distance[0]['nothing'] & distance[1]['nothing'] & distance[2]['nothing'] & distance[3]['nothing'] & distance[4]['nothing'] & distance[5]['nothing'] & distance[6]['nothing'] & distance[7]['nothing'], v_right['turning']))
        rule_Wall.append(ctrl.Rule(distance[3]['nothing']| distance[3]['away'] | distance[4]['nothing'] | distance[4]['nothing'], v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & distance[7]['close'], v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & distance[7]['close'], v_right['foward']))

        #ESQUERDA
        rule_Wall.append(ctrl.Rule(distance[0]['nothing'] & distance[1]['nothing'] & distance[2]['nothing'] & distance[3]['nothing'] & distance[4]['nothing'] & distance[5]['nothing'] & distance[6]['nothing'] & distance[7]['nothing'], v_left['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['nothing']| distance[3]['away'] | distance[4]['nothing'] | distance[4]['nothing'], v_left['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & distance[7]['close'], v_left['reverse']))
        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & distance[7]['close'], v_left['reverse']))
        

        #SEGUIR PAREDE
        #DIREITA
        rule_Wall.append(ctrl.Rule(distance[7]['ideal'] , v_right['turning']))
        rule_Wall.append(ctrl.Rule(distance[7]['close'] , v_right['foward']))

        #ESQUERDA
        rule_Wall.append(ctrl.Rule(distance[7]['ideal'] , v_left['foward']))
        rule_Wall.append(ctrl.Rule(distance[7]['close'] , v_left['turning']))

        #QUINA
        #DIREITA
        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & (distance[7]['ideal'] | distance[7]['close']) , v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & (distance[7]['ideal'] | distance[7]['close']) , v_right['foward']))

        #ESQUERDA
        rule_Wall.append(ctrl.Rule(distance[3]['ideal'] | distance[4]['ideal'] & (distance[7]['ideal'] | distance[7]['close']), v_left['reverse']))
        rule_Wall.append(ctrl.Rule(distance[3]['close'] | distance[4]['close'] & (distance[7]['ideal'] | distance[7]['close']) , v_left['reverse']))

        #PERDEU PAREDE
        #DIREITA   
        rule_Wall.append(ctrl.Rule((distance[7]['nothing'] | distance[7]['away']) & (distance[8]['close'] | distance[8]['ideal'] | distance[8]['away']), v_right['lostWall']))
        rule_Wall.append(ctrl.Rule((distance[7]['ideal'] | distance[7]['away']) & (distance[8]['nothing'] | distance[8]['away']) & (distance[3]['nothing'] & distance[4]['nothing']), v_right['lostWall']))

        #ESQUERDA
        rule_Wall.append(ctrl.Rule((distance[7]['nothing'] | distance[7]['away']) & (distance[8]['close'] | distance[8]['ideal'] | distance[8]['away']), v_left['lostWall']))
        rule_Wall.append(ctrl.Rule((distance[7]['ideal'] | distance[7]['away']) & (distance[8]['nothing'] | distance[8]['away']) & (distance[3]['nothing'] & distance[4]['nothing']), v_left['lostWall'])) 

        #DANGER 
        #DIREITA
        rule_Wall.append(ctrl.Rule(distance[7]['danger'] | distance[5]['danger'], v_right['foward']))
        rule_Wall.append(ctrl.Rule(distance[3]['danger'] | distance[4]['danger'] | distance[6]['danger'] , v_right['foward']))

        #ESQUERDA
        rule_Wall.append(ctrl.Rule(distance[7]['danger'] | distance[5]['danger'], v_left['turning']))
        rule_Wall.append(ctrl.Rule(distance[3]['danger'] | distance[4]['danger'] | distance[6]['danger'], v_left['reverse']))

        vel_ctrl = ctrl.ControlSystem(rule_Wall)
        self.fuzzy_system = ctrl.ControlSystemSimulation(vel_ctrl)


    def get_vel(self, dist):
        for i in range(len(dist)):
            self.fuzzy_system.input['s' + str(i)] = dist[i]

        self.fuzzy_system.compute()
        return [self.fuzzy_system.output['vl'], self.fuzzy_system.output['vr']]


    def printMembership(self):
        distance = np.arange(-0.1, 1, self.steps)

        danger   = fuzz.trimf(distance, [0.0, 0.0, 0.15])
        close    = fuzz.trapmf(distance, [0.0, 0.1, 0.15, 0.2])
        ideal    = fuzz.trapmf(distance, [0.15, 0.2, 0.3, 0.5])
        away     = fuzz.trapmf(distance, [0.3, 0.5, 0.6, 0.7])
        nothing  = fuzz.trapmf(distance, [0.6, 0.7, 1.0, 1.0])

        fig, (ax0) = plt.subplots(nrows = 1, figsize= (8,9))

        ax0.plot(distance, danger, 'r', lineWidth = 1.5, label = 'Danger')
        ax0.plot(distance, close, 'y', lineWidth = 1.5, label = 'Close')
        ax0.plot(distance, ideal, 'g', lineWidth = 1.5, label = 'Ideal')
        ax0.plot(distance, away, 'b', lineWidth = 1.5, label = 'Away')
        ax0.plot(distance, nothing, 'c', lineWidth = 1.5, label = 'Nothing')
        ax0.set_title('Distance from robot')
        ax0.legend()

        ax0.spines['top'].set_visible(False)
        ax0.spines['right'].set_visible(False)
        ax0.get_xaxis().tick_bottom()
        ax0.get_yaxis().tick_left()

        plt.tight_layout()
        plt.show()



robot = Robot()
plt.ion()
a = wall_following()
a.init_fuzzy()

while(robot.get_connection_status() != -1):
    ultrassonicRaw = robot.read_ultrassonic_sensors()[0:11]

    pos = robot.get_current_position()

    ultrassonic = ultrassonicRaw[0:8]
    ultrassonic.append(ultrassonicRaw[9])
    vel = a.get_vel(ultrassonic)
    #print("Ultrassonic: ", ultrassonic)
    #print("Iteracao: ", x)
    #print("Esquerda: " , ultrassonic[0:3])
    #print("Frente: " , ultrassonic[3:5])
    #print("Direita: " , ultrassonic[5:8])
    #print("Tras: ", ultrassonic[8])
    #print("vel: ", vel)
    robot.set_left_velocity(vel[0])  # rad/s
    robot.set_right_velocity(vel[1])
    time.sleep(0.2)

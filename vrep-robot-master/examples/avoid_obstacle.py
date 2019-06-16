from skfuzzy import control as ctrl
import skfuzzy as fuzz
import numpy as np
import math
import time
import sys
sys.path.insert(0, '../src')
from robot import Robot
import matplotlib.pyplot as plt

class avoid_obstacles():
    def __init__(self):
        self.NUM_SENSORS = 8
        self.max_speed = 2*math.pi
        self.steps = 0.01
        self.fuzzy_system = []

    def init_fuzzy(self, behavior):
        distance = []
        for i in range(0, self.NUM_SENSORS):
            #mantido pois se trata apenas do universo de possibilidades de distancia de 0 a 5 de 0.01 em 0.01
            distance.append(ctrl.Antecedent(np.arange(0, 5.0, self.steps), 's'+str(i)))
            #fuzzy sets criados pelo ped
            if(behavior == "normal"):
                distance[i]['close'] = fuzz.trapmf(
                    distance[i].universe, [0.0, 0.0, 0.3, 0.6])
                distance[i]['away'] = fuzz.trapmf(
                    distance[i].universe, [0.3, 0.6, 5.0, 5.0])
            #comportamento mais previnido evitando mais(ficava muito preso nos espaços)
            if(behavior == "previnido"):
                distance[i]['close'] = fuzz.trapmf(
                    distance[i].universe, [0.0, 0.0, 0.45, 0.9])
                distance[i]['away'] = fuzz.trapmf(
                    distance[i].universe, [0.45, 0.9, 5.0, 5.0])            
            #comportamento mais de mudanca em cima da hora
            if(behavior == "be1"):
                distance[i]['close'] = fuzz.trapmf(
                    distance[i].universe, [0.0, 0.0, 0.3, 0.3])
                distance[i]['away'] = fuzz.trapmf(
                    distance[i].universe, [0.15, 0.3, 5.0, 5.0])   
            #comportamento mais de mudanca em cima da hora (sai mais rapido de comodos mais fechados)
            if(behavior == "be2"):                
                distance[i]['close'] = fuzz.trapmf(
                    distance[i].universe, [0.0, 0.0, 0.1, 0.25])
                distance[i]['away'] = fuzz.trapmf(
                    distance[i].universe, [0.1, 0.25, 5.0, 5.0])        

        #mantido pois se trata apenas do universo de possibilidades de velocidade
        v_left = ctrl.Consequent(
            np.arange(-self.max_speed, 1.0+self.max_speed, 0.01), 'vl')
        v_right = ctrl.Consequent(
            np.arange(-self.max_speed, 1.0+self.max_speed, 0.01), 'vr')

        #indo em frente roda esquerda
        v_left['positive'] = fuzz.trimf(
            v_left.universe, [0.0, self.max_speed-0.1, self.max_speed-0.1])
        #indo para tras roda esquerda
        v_left['negative'] = fuzz.trimf(
            v_left.universe, [-self.max_speed, -self.max_speed, 0.0])
        #indo em frente roda direita
        v_right['positive'] = fuzz.trimf(
            v_left.universe, [0.0, self.max_speed, self.max_speed])
        #indo para tras roda esquerda
        v_right['negative'] = fuzz.trimf(
            v_left.universe, [-self.max_speed+0.1, -self.max_speed+0.1, 0.0])

        #resumo das regras originais:
        #sensores 0-3 proximos implica em virar a direita, com a esquerda indo para frente e a direita para tras
        rule_l1 = ctrl.Rule(distance[0]['close'] | distance[1]['close'] |
                            distance[2]['close'] | distance[3]['close'], v_left['positive'])
        rule_r1 = ctrl.Rule(distance[0]['close'] | distance[1]['close'] |
                            distance[2]['close'] | distance[3]['close'], v_right['negative'])

        #sensores 4-7 proximos implica em virar a esquerda, com a esquerda indo para tras e a direita para frente
        rule_l2 = ctrl.Rule(distance[4]['close'] | distance[5]['close'] |
                            distance[6]['close'] | distance[7]['close'], v_left['negative'])
        rule_r2 = ctrl.Rule(distance[4]['close'] | distance[5]['close'] |
                            distance[6]['close'] | distance[7]['close'], v_right['positive'])

        #sensores 0-7 distantes implica em ir em frente ambas as rodas
        rule_l3 = ctrl.Rule(distance[0]['away'] & distance[1]['away'] & distance[2]['away'] & distance[3]['away'] &
                            distance[4]['away'] & distance[5]['away'] & distance[6]['away'] & distance[7]['away'], v_left['positive'])
        rule_r3 = ctrl.Rule(distance[0]['away'] & distance[1]['away'] & distance[2]['away'] & distance[3]['away'] &
                            distance[4]['away'] & distance[5]['away'] & distance[6]['away'] & distance[7]['away'], v_right['positive'])

        vel_ctrl = ctrl.ControlSystem(
            [rule_l1, rule_l2, rule_l3, rule_r1, rule_r2, rule_r3]
        )
        self.fuzzy_system = ctrl.ControlSystemSimulation(vel_ctrl)

    def get_vel(self, dist):
        for i in range(len(dist)):
            self.fuzzy_system.input['s'+str(i)] = dist[i]

        self.fuzzy_system.compute()
        return [self.fuzzy_system.output['vl'], self.fuzzy_system.output['vr']]


robot = Robot()
Avoid = avoid_obstacles()

behavior_type = input()
#normal(como o ped), be1, be2, previnido

Avoid.init_fuzzy(behavior_type)
data_position_x = []
data_position_y = []
how_long = int(input())
t_start = time.time()

while(robot.get_connection_status() != -1):    
    dist = robot.read_ultrassonic_sensors()    
    vel = Avoid.get_vel(dist[:8])   
    #position = robot.get_current_position()
    #data_position_x.append(position[0])
    #data_position_y.append(position[1])    
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
    t_end = time.time()
    if (t_end - t_start) >= how_long:
        break


t = t_end - t_start 
print(t)
#plt.plot(data_position_x, data_position_y, linewidth=2.0)  
#plt.ylabel('Y Position')
#plt.xlabel('X Position')
#plt.show()

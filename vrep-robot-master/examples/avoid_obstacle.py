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
        self.max_speed = 2*math.pi #para aumentar o dinamismo da simulacao
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
            #comportamento mais brusco
            if(behavior == "brusco1"):
                distance[i]['close'] = fuzz.trapmf(
                    distance[i].universe, [0.0, 0.0, 0.15, 0.3])
                distance[i]['away'] = fuzz.trapmf(
                    distance[i].universe, [0.15, 0.3, 5.0, 5.0])   
            #comportamento ainda mais brusco (sai mais rapido de comodos mais fechados)
            if(behavior == "brusco2"):                
                distance[i]['close'] = fuzz.trapmf(
                    distance[i].universe, [0.0, 0.0, 0.1, 0.25])
                distance[i]['away'] = fuzz.trapmf(
                    distance[i].universe, [0.1, 0.25, 5.0, 5.0])        

        #mantido pois se trata apenas do universo de possibilidades de velocidade de cada roda
        v_left = ctrl.Consequent(np.arange(-self.max_speed, 1.0+self.max_speed, 0.01), 'vl')
        v_right = ctrl.Consequent(np.arange(-self.max_speed, 1.0+self.max_speed, 0.01), 'vr')               

        #indo em frente roda esquerda
        v_left['positive'] = fuzz.trimf(v_left.universe, [0.0, self.max_speed-0.1, self.max_speed-0.1])
        #indo para tras roda esquerda
        v_left['negative'] = fuzz.trimf(v_left.universe, [-self.max_speed, -self.max_speed, 0.0])
        #indo em frente roda direita
        v_right['positive'] = fuzz.trimf(v_right.universe, [0.0, self.max_speed, self.max_speed])
        #indo para tras roda esquerda
        v_right['negative'] = fuzz.trimf(v_right.universe, [-self.max_speed+0.1, -self.max_speed+0.1, 0.0])

        #v_left.view()
        #v_right.view()

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

        #regras abandonadas por entrar em conflito com as 3 primeiras, e nao terem contribuido
        #com as manobras ao encontrar de frente um obstaculo, onde a defuzzyficação de desviar pela esquerda
        #ou pela direita é mais dificil. Os 2 de frente muito proximos, o robô daria ré.
        #rule_l4 = ctrl.Rule(distance[3]['veryClose'] & distance[4]['veryClose'], v_left['negative'])
        #rule_r4 = ctrl.Rule(distance[3]['veryClose'] & distance[4]['veryClose'] v_right['negative'])

        vel_ctrl = ctrl.ControlSystem([rule_l1, rule_l2, rule_l3, rule_r1, rule_r2, rule_r3])
        self.fuzzy_system = ctrl.ControlSystemSimulation(vel_ctrl)

    #nao houve necessidade de alterar o get velocity
    def get_vel(self, dist):
        for i in range(len(dist)):
            self.fuzzy_system.input['s'+str(i)] = dist[i]

        self.fuzzy_system.compute()        
        return [self.fuzzy_system.output['vl'], self.fuzzy_system.output['vr']]

    def printMembership(self):
        distance = np.arange(-0.1, 1, self.steps)
        close    = fuzz.trapmf(distance, [0.0, 0.0, 0.1, 0.25])       
        away     = fuzz.trapmf(distance, [0.1, 0.25, 5.0, 5.0])        

        fig, (ax0) = plt.subplots(nrows = 1, figsize= (8,9))
        
        ax0.plot(distance, close, 'y', lineWidth = 1.5, label = 'Close')        
        ax0.plot(distance, away, 'b', lineWidth = 1.5, label = 'Away')        
        ax0.set_title('Distance from robot')
        ax0.legend()

        ax0.spines['top'].set_visible(False)
        ax0.spines['right'].set_visible(False)
        ax0.get_xaxis().tick_bottom()
        ax0.get_yaxis().tick_left()
        #plt.tight_layout()
        #plt.show()

#iniciando o robô e a classe fuzzy
robot = Robot()
Avoid = avoid_obstacles()

#pegando o tipo de comportamento
#normal(como o ped), previnido, brusco1, brusco2
print("Escolha o comportamento dos conjuntos fuzzy: normal, previnido, brusco1 ou brusco2")
behavior_type = input()

#iniciando o fuzzy com o comportamento
Avoid.init_fuzzy(behavior_type)

#controlando o tempo de execucao
print("Por quantos segundos a simulação deve rodar?")
how_long = int(input())
t_start = time.time()

while(robot.get_connection_status() != -1):    
    dist = robot.read_ultrassonic_sensors()    
    vel = Avoid.get_vel(dist[:8])       
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
    t_end = time.time()
    if (t_end - t_start) >= how_long:
        break

Avoid.printMembership()
t = t_end - t_start 
print(t)

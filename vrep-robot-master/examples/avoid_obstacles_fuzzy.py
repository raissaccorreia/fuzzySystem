import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class avoid_obstacles():
	def __init__(self):
		self.NUM_SENSORS = 8
		self.max_speed = math.pi
		self.steps = 0.01

	def init_fuzzy(self):
		distance=[]
		for i in range(0, self.NUM_SENSORS):
			distance.append(ctrl.Antecedent(np.arange(0, 5.0, self.steps), 's'+str(i)))
			distance[i]['close'] = fuzz.trapmf(distance[i].universe, [0.0, 0.0, 0.3, 0.6])
			distance[i]['away'] = fuzz.trapmf(distance[i].universe, [0.3, 0.6, 5.0, 5.0])

		v_left = ctrl.Consequent(np.arange(-self.max_speed, 1.0+self.max_speed, 0.01), 'vl')
		v_right = ctrl.Consequent(np.arange(-self.max_speed, 1.0+self.max_speed, 0.01), 'vr')

		v_left['positive'] = fuzz.trimf(v_left.universe, [0.0, self.max_speed-0.1, self.max_speed-0.1])
		v_left['negative'] = fuzz.trimf(v_left.universe, [-self.max_speed, -self.max_speed, 0.0])
		v_right['positive'] = fuzz.trimf(v_left.universe, [0.0, self.max_speed, self.max_speed])
		v_right['negative'] = fuzz.trimf(v_left.universe, [-self.max_speed+0.1, -self.max_speed+0.1, 0.0])

		rule_l1 = ctrl.Rule(distance[0]['close'] | distance[1]['close'] | distance[2]['close'] | distance[3]['close'], v_left['positive'])
		rule_l2 = ctrl.Rule(distance[4]['close'] | distance[5]['close'] | distance[6]['close'] | distance[7]['close'], v_left['negative'])
		rule_l3 = ctrl.Rule(distance[0]['away'] & distance[1]['away'] & distance[2]['away'] & distance[3]['away'] & distance[4]['away'] & distance[5]['away'] & distance[6]['away'] & distance[7]['away'], v_left['positive'])


		rule_r1 = ctrl.Rule(distance[0]['close'] | distance[1]['close'] | distance[2]['close'] | distance[3]['close'], v_right['negative'])
		rule_r2 = ctrl.Rule(distance[4]['close'] | distance[5]['close'] | distance[6]['close'] | distance[7]['close'], v_right['positive'])
		rule_r3 = ctrl.Rule(distance[0]['away'] & distance[1]['away'] & distance[2]['away'] & distance[3]['away'] & distance[4]['away'] & distance[5]['away'] & distance[6]['away'] & distance[7]['away'], v_right['positive'])

		vel_ctrl = ctrl.ControlSystem([rule_l1, rule_l2, rule_l3, rule_r1, rule_r2, rule_r3])
		self.fuzzy_system = ctrl.ControlSystemSimulation(vel_ctrl)

	def get_vel(self, dist):
		for i in range(len(dist)):
			self.fuzzy_system.input['s'+str(i)] = dist[i]

		self.fuzzy_system.compute()
		return [self.fuzzy_system.output['vl'], self.fuzzy_system.output['vr']]

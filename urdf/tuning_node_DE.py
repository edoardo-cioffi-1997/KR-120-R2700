#!/usr/bin/env python3

import rospy
import sys
from pid_tuning.evolutive_algorithms.dif_evolution import DifferentialEvolution
from pid_tuning.settings.control_gazebo import ControlGazebo

reset_control = ControlGazebo()

A = 10 #number of joints controlled
m = 3*A #number of design variables: kp,ki,kd for each controller, OF and SCV already included in DE algorithm
N = 100 #could be 10*A 
Gm = 10000
F = 0.65
C = 0.85
hz = 625 #1/(5s/(number of samples in .csv))

de = DifferentialEvolution(N, m, Gm, F, C, A, '/home/devadm/rmtp_ws/src/kr120/config/paths.json', epsilon_1=1, tm=28800)
X = de.gen_population()

def evol_loop():
    rospy.init_node("tuning_node")
    file = open("kr120_best_pid_values_DE.txt", 'w')

    reset_control.init_values()
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        de.evaluate(X, reset_control, rate)
        X_best = de.dif_evolution(X, reset_control, rate)
        file.write(str(X_best))
        file.close()
        break

if __name__ == '__main__':
    try:
        evol_loop()
    except rospy.ROSInterruptException:
        pass

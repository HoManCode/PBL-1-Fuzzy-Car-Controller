import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import CarController as fuzzyCtrl
import FuzzySystem as fuzzySys

"""
Uses the controller created in fuzzyCarFuzzySystem.py to simulate the control system for testing
purposes.
"""

# Viewing the fuzzy membership functions as graphs
view = False # Change this to true to view the membership function graphs
if view:
    fuzzySys.frontVehicleDistance.view()
    fuzzySys.currentSpeed.view()
    fuzzySys.speedChange.view()
    fuzzySys.plt.show()

# To view the fuzzy membership function based on the real inputs to the system, check the defuzzify() method in FuzzySystem.py

#Intialises the ACC system using the car control system
fuzzyCtrl.initialiseACCSystem()
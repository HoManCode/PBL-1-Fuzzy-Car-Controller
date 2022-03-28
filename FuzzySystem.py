import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

"""
The controller for the fuzzy logic component of the adaptive cruise control
Based on the tipping example on SciKitFuzzy's documentation website available at

https://pythonhosted.org/scikit-fuzzy/auto_examples/plot_tipping_problem_newapi.html#example-plot-tipping-problem-newapi-py
"""

# Default condition is a dry road, unless changed by fuzzyCarController.py
roadConditionDry = True

# Antecedent and consequent objects
frontVehicleDistance = ctrl.Antecedent(np.arange(0,200,1), 'frontVehicleDistance')
currentSpeed = ctrl.Antecedent(np.arange(40,110,1),'currentSpeed')
speedChange = ctrl.Consequent(np.arange(-1,1,.01), 'speedChange')

# Custom membership functions using skfuzzy's control API

# Distance membership function for the dry road conditon
if roadConditionDry:
    frontVehicleDistance['dangerous'] = fuzz.trimf(frontVehicleDistance.universe, [0,0,45])
    frontVehicleDistance['close'] = fuzz.trimf(frontVehicleDistance.universe, [0,85,170])
    frontVehicleDistance['safe'] = fuzz.trimf(frontVehicleDistance.universe, [145,200,200])

# Distance membership function for the wet road condition - based on a friction coefficient of 0.7
else:
    frontVehicleDistance['dangerous'] = fuzz.trimf(frontVehicleDistance.universe, [0,0,32])
    frontVehicleDistance['close'] = fuzz.trimf(frontVehicleDistance.universe, [0,69,150])
    frontVehicleDistance['safe'] = fuzz.trimf(frontVehicleDistance.universe, [113,200,200])

# Current speed membership function
currentSpeed['slow'] = fuzz.trimf(currentSpeed.universe, [40,40,75])
currentSpeed['normal'] = fuzz.trimf(currentSpeed.universe, [40,75,110])
currentSpeed['fast'] =fuzz.trimf(currentSpeed.universe, [75,110,110])

# Change in speed membership function
# Negative numbers refer to reducing speed, positive to increasing speed
speedChange['brake fast'] = fuzz.trapmf(speedChange.universe,[-1,-0.2,0,0])
speedChange['brake slowly'] = fuzz.trimf(speedChange.universe, [-0.4,-0.4,0])
speedChange['throttle slowly'] = fuzz.trapmf(speedChange.universe,[-.3,-.1,.1,.3])
speedChange['throttle medium'] = fuzz.trimf(speedChange.universe, [0,0.4,0.4])
speedChange['throttle fast'] = fuzz.trapmf(speedChange.universe,[0,0,0.2,1])
    

# Rules
rule1 = ctrl.Rule(frontVehicleDistance['dangerous'] & currentSpeed['slow'], speedChange['brake slowly'])
rule2 = ctrl.Rule(frontVehicleDistance['dangerous'] & currentSpeed['normal'], speedChange['brake slowly'])
rule3 = ctrl.Rule(frontVehicleDistance['dangerous'] & currentSpeed['fast'], speedChange['brake fast'])
rule4 = ctrl.Rule(frontVehicleDistance['close'] & currentSpeed['slow'], speedChange['throttle medium'])
rule5 = ctrl.Rule(frontVehicleDistance['close'] & currentSpeed['normal'], speedChange['throttle medium'])
rule6 = ctrl.Rule(frontVehicleDistance['close'] & currentSpeed['fast'], speedChange['throttle slowly'])
rule7 = ctrl.Rule(frontVehicleDistance['safe'] & currentSpeed['slow'], speedChange['throttle fast'])
rule8 = ctrl.Rule(frontVehicleDistance['safe'] & currentSpeed['normal'], speedChange['throttle medium'])
rule9 = ctrl.Rule(frontVehicleDistance['safe'] & currentSpeed['fast'], speedChange['throttle medium'])

# Creating the fuzzy control system
cruiseControl = ctrl.ControlSystem([rule1,rule2,rule3,rule4,rule5,rule6,rule7,rule8,rule9])

def defuzzify(frontVehicleDistanceInput, currentSpeedInput):
    """
    Outputs the speed change as computed by the fuzzification system
    """
    cruiseControlSim = ctrl.ControlSystemSimulation(cruiseControl)

    cruiseControlSim.input['frontVehicleDistance'] = frontVehicleDistanceInput
    cruiseControlSim.input['currentSpeed'] = currentSpeedInput

    cruiseControlSim.compute()

    #Uncomment these to view the simulated membership values
    #frontVehicleDistance.view(sim=cruiseControlSim)
    #currentSpeed.view(sim=cruiseControlSim)
    #speedChange.view(sim=cruiseControlSim)

    return cruiseControlSim.output['speedChange']
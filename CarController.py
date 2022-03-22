"""
Controller for Adaptive Cruise Control using fuzzy logic

"""


from multiprocessing.sharedctypes import Value
import time
import random
from skfuzzy import control as ctrl
import FuzzySystem as fuzzySys


def initialiseACCSystem():
    """
    Receives the speed setting from the user to be maintained by the ACC system.
    Begins the system process with the input speed by the user.
    Gets the current vehicle speed from the onboard speedometer.

    For the purposes of testing, this current speed will be randomly generated based on the set
    speed, since drivers set the speed by bringing the vehicle to that speed manually.
    """

    print("\nCruise control system initialised.")

    vehicleSetSpeed = validSpeedInput()
    vehicleCurrentSpeed = random.randrange(vehicleSetSpeed-10, vehicleSetSpeed+10)

    print("Current speed is " + str(vehicleCurrentSpeed))

    ACCSystemProcess(vehicleSetSpeed, vehicleCurrentSpeed)


def ACCSystemProcess(vehicleSetSpeed, vehicleCurrentSpeed):
    """
    Checks the road ahead for another vehicle within 145 m. If there is a vehicle ahead, checks the road conditions
    to see if the road is wet or dry and maintains a safe following speed using the Fuzzy System.

    If there are no vehicles ahead within 145m, maintains the speed set by the user.
    
    """

    #Engage the fuzzy system to receive appropriate outputs

    while True:

        vehicleAhead = checkRoadAhead()

        if not vehicleAhead:
            vehicleCurrentSpeed = maintainSpeed(vehicleSetSpeed, vehicleCurrentSpeed)

        if vehicleAhead:

            roadConditionDry = checkRoadFriction()
            if not roadConditionDry:
                fuzzySys.roadConditionDry = False # The default in the fuzzySys is True, so we only need to change if the road is wet

            speedChange = fuzzySys.defuzzify(vehicleAhead, vehicleCurrentSpeed)

            if speedChange < 0:
                vehicleCurrentSpeed = engageBrake(vehicleSetSpeed, vehicleCurrentSpeed,int(abs(speedChange) * 50)) # 50 is the brake force
            
            if speedChange == 0:
                maintainSpeed(vehicleSetSpeed, vehicleCurrentSpeed)

            if speedChange > 0:
                vehicleCurrentSpeed = engageThrottle(vehicleSetSpeed, vehicleCurrentSpeed, int(speedChange * 25)) # 25 is the throttle force

        time.sleep(1)
    
def maintainSpeed(vehicleSetSpeed, vehicleCurrentSpeed):
    """
    Maintains the current speed by engaging the throttle and brake appropriately.
    Returns the modified speed based on conditions.
    """
    if vehicleSetSpeed > vehicleCurrentSpeed:
        vehicleCurrentSpeed = engageThrottle(vehicleSetSpeed, vehicleCurrentSpeed, 0.4)
    elif vehicleSetSpeed < vehicleCurrentSpeed:
        vehicleCurrentSpeed = engageBrake(vehicleSetSpeed, vehicleCurrentSpeed, 0.2)
    else:
        print("Maintain speed at " + str(vehicleSetSpeed))
    return vehicleCurrentSpeed


def engageBrake(vehicleSetSpeed, vehicleCurrentSpeed, brakeForce):
    """
    Engages the brakes to decrease speed
    """
    vehicleCurrentSpeed -= brakeForce * 2 #Braking force of 2km/h per tick
    print("\nDecelerating... Speed is now " + str(int(vehicleCurrentSpeed)))
    return int(vehicleCurrentSpeed)


def engageThrottle(vehicleSetSpeed, vehicleCurrentSpeed, throttleForce):
    """
    Engages the throttle to increase speed
    """
    vehicleCurrentSpeed += throttleForce * 2 #Throttle force of 2km/h per tick
    print("\nAccelerating... Speed is now " + str(int(vehicleCurrentSpeed)))
    return int(vehicleCurrentSpeed)



def checkRoadFriction():
    """
    Checks the road slippage based on ABS standard sensors and returns bool based on the friction
    coefficient received from the sensors.
    If the system detects a very low friction coefficient it disengages the ACC system for safety
    reasons (such as icy roads).
    For the purposes of testing, this will be input as 1 or 0 by the user.
    """
    while True:
        try:
            inputConditions = int(input("Enter 1 for dry road conditions, 0 for wet: "))
            if inputConditions != 0 and inputConditions != 1:
                raise ValueError
            break
        except ValueError:
            print("You need to enter a 1 or a 0. Please try again.")
    
    if inputConditions == 1:
        return True
    else:
        return False
            

def checkRoadAhead():
    """
    Uses the vehicle radar to check the lane ahead for another vehicle. 
    Returns the distance to the vehicle as an int or False if there is no vehicle detected by the radar.
    For the purposes of testing, this will be input by the user.
    """
    while True:
        try:
            inputVehicleAhead = input("\nEnter the distance to the vehicle ahead in metres or False if there is not vehicle: ")
            if inputVehicleAhead == 'False':
                inputVehicleAhead = False
                break
            else:
                inputVehicleAhead = int(inputVehicleAhead)
            break
        except ValueError:
            print("You need to enter a number or 'False'. Please try again.")
    
    return inputVehicleAhead

def validSpeedInput():
    """
    Ensures the user enters a valid number for the speed setting
    Most cruise control systems do not engage below 40 km/h
    """
    while True:
        try:
            inputSpeed = int(input("\nEnter a speed in km/h for cruise control: "))
            if inputSpeed < 40 or inputSpeed > 110:
                print('Cruise control can only be set between 40 and 110 kmp/h.')
                raise ValueError
            break
        except ValueError:
            print("That wasn't a valid input. Please try again.")
    
    return inputSpeed
######################################################################
### Fairly Robust Launch Script
######################################################################
### Kerbart and I are working on a more versatile and pythonic
### launch script that uses the same logic as this one, but I 
### thought that it was still worth posting this simpler version
### for perusal (or use!)   
######################################################################

import krpc
import time
import math

from pid import PID
from node_executor import execute_next_node

# ----------------------------------------------------------------------------
# Script parameters
# ----------------------------------------------------------------------------

REFRESH_FREQ = 2    # refresh rate in hz
TELEM_DELAY = 5     #number of seconds between telemetry updates
ALL_FUELS = ('LiquidFuel', 'SolidFuel')
MAX_PHYSICS_WARP = 3 # valid values are 0 (none) through 3 (4x)
next_telem_time=time.time()

class MissionParameters(object):
    '''
    All mission parameters are stored in a single object to easily
    pass around
    '''
    def __init__(self,
                 max_auto_stage = 0,
                 orbit_alt = 100000,
                 grav_turn_finish = 55000,
                 inclination = 0,
                 force_roll = True,
                 roll = 90,
                 deploy_solar = True,
                 max_q = 20000):
        self.max_auto_stage = max_auto_stage
        self.orbit_alt = orbit_alt
        self.grav_turn_finish = grav_turn_finish
        self.inclination = inclination
        self.force_roll = force_roll 
        self.roll = roll
        self.deploy_solar = deploy_solar
        self.max_q = max_q
       
class Telemetry(object):
    def __init__(self, vessel, flight):
        self.apoapsis = vessel.orbit.apoapsis_altitude
        self.periapsis = vessel.orbit.periapsis_altitude
        self.time_to_apo = vessel.orbit.time_to_apoapsis
        self.time_to_peri = vessel.orbit.time_to_periapsis
        self.velocity = vessel.orbit.speed
        self.inclination = math.radians(vessel.orbit.inclination)
        self.altitude = flight.mean_altitude
        self.vertical_speed = flight.vertical_speed
        self.lat = flight.latitude
        self.lon = flight.longitude
        self.q = flight.dynamic_pressure
        self.g = flight.g_force

 

# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------

def main():
    '''
    main function is run when you just execute this file, but NOT when you 
    import it into another file - thus you can choose to call ascent later 
    to go to space, or just use the other functions in this file.
    '''

    ##  Setup KRPC and create a launch_params object with the default settings
    conn = krpc.connect(name='Launch')
    launch_params = MissionParameters() 
    ascent(conn,launch_params)
    
def ascent(conn, launch_params):
    '''
    Ascent Autopilot function.  Goes to space, or dies trying.
    '''
    #Setup KRPC and PIDs
    conn = krpc.connect(name='Launch')
    sc = conn.space_center
    v = sc.active_vessel
    telem=v.flight(v.orbit.body.reference_frame)
    thrust_controller = PID(P=.001, I=0.0001, D=0.01)
    thrust_controller.ClampI = launch_params.max_q
    thrust_controller.setpoint(launch_params.max_q) 
    

    #Prepare for Launch
    v.auto_pilot.engage()
    v.auto_pilot.target_heading=inc_to_heading(launch_params.inclination)
    if launch_params.force_roll: 
        v.auto_pilot.target_roll=launch_params.roll
    v.control.throttle=1.0
    
    #Gravity Turn Loop
    while apoapsis_way_low(v, launch_params.orbit_alt):
        gravturn(conn, launch_params)
        autostage(v , launch_params.max_auto_stage)
        limitq(conn, thrust_controller)
        telemetry(conn)
        time.sleep(1.0 / REFRESH_FREQ)        
    v.control.throttle = 0.0
    
    # Fine Tune APA
    v.auto_pilot.disengage()
    v.auto_pilot.sas=True
    time.sleep(.1)
    v.auto_pilot.sas_mode = v.auto_pilot.sas_mode.prograde
    v.auto_pilot.wait()
    boostAPA(conn, launch_params)  #fine tune APA

    # Coast Phase
    sc.physics_warp_factor = MAX_PHYSICS_WARP
    while still_in_atmosphere(conn):   
        if apoapsis_little_low(v , launch_params.orbit_alt):
            sc.physics_warp_factor = 0
            boostAPA(conn, launch_params)
            sc.physics_warp_factor = MAX_PHYSICS_WARP
        telemetry(conn)  
        time.sleep(1.0 / REFRESH_FREQ)       
    
    # Circularization Burn
    sc.physics_warp_factor = 0
    planCirc(conn)
    telemetry(conn)
    execute_next_node(conn)

    # Finish Up
    if launch_params.deploy_solar: v.control.solar_panels=True 
    telemetry(conn)
    v.auto_pilot.sas_mode= v.auto_pilot.sas_mode.prograde
 
# ----------------------------------------------------------------------------
# staging logic
# ----------------------------------------------------------------------------        

def autostage(vessel, MAX_AUTO_STAGE):
    '''
    activate next stage when there is no fuel left in the current stage
    '''
    if out_of_stages(vessel, MAX_AUTO_STAGE):   
        return
    res = get_resources(vessel)
    interstage = True   # flag to check if this is a fuel-less stage
    for fueltype in ALL_FUELS:
        if out_of_fuel(res, fueltype):
            next_stage(vessel)
            return
        if res.has_resource(fueltype):
            interstage = False
    if interstage:
        next_stage(vessel)

# ----------------------------------------------------------------------------
# guidance routines
# ----------------------------------------------------------------------------        

def gravturn(conn, launch_params):
    '''
    Execute quadratic gravity turn -  
    based on Robert Penner's easing equations (EaseOut)
    '''
    vessel = conn.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.non_rotating_reference_frame)
    progress=flight.mean_altitude/launch_params.grav_turn_finish
    vessel.auto_pilot.target_pitch= 90-(-90 * progress*(progress-2))
     
def boostAPA(conn, launch_params):
    '''
    function to increase Apoapsis using low thrust on a 
    tight loop with no delay for increased precision.
    '''
    vessel = conn.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.non_rotating_reference_frame)

    vessel.control.throttle=.2
    while apoapsis_little_low(vessel, launch_params.orbit_alt):
        autostage(vessel, launch_params.max_auto_stage)
        telemetry(conn) 
    vessel.control.throttle=0

def planCirc(conn):

    '''
    Plan a Circularization at Apoapsis.  
    V1 is velocity at apoapsis.  
    V2 is the velocity at apoapsis of a circular orbit.   
    Burn time uses Tsiolkovsky rocket equation.
    '''
    vessel = conn.space_center.active_vessel
    ut = conn.space_center.ut
    grav_param = vessel.orbit.body.gravitational_parameter
    apo = vessel.orbit.apoapsis
    sma = vessel.orbit.semi_major_axis
    v1 = math.sqrt(grav_param * ((2.0 / apo) - (1.0 / sma)))
    v2 = math.sqrt(grav_param * ((2.0 / apo) - (1.0 / apo)))
    vessel.control.add_node(ut + vessel.orbit.time_to_apoapsis, 
                            prograde=(v2 - v1))

def inc_to_heading(inc):
    '''
    Converts desired inclination to a compass heading the autopilot can track
    This only works for equatorial launches at the moment!   
    inc: inclination in degrees
    returns: heading in degrees
    '''
    if inc > 180 or inc < -180:
        return 90   #invalid entries get set to 0 inclination
    if inc >= 0:
        value = 90 - inc
    if inc < 0:
        value = -(inc - 90)
    if value < 0:
        value += 360
    return value

def limitq(conn, controller):
    '''
    limits vessel's throttle to stay under MAX_Q using PID controller
    '''
    vessel = conn.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.non_rotating_reference_frame)
    vessel.control.throttle= controller.update(flight.dynamic_pressure)
 
 
# ----------------------------------------------------------------------------
# post telemetry
# ----------------------------------------------------------------------------             

def telemetry(conn):
    '''
    Show telemetry data
    TODO: split between creating telemetry data (object? dict?)
    and displaying the data. This will make it easier to transition to a
    GUI later on. For this reason, no attempts to fit the lines has been
    made (yet)
    '''
    vessel = conn.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.non_rotating_reference_frame)
    global next_telem_time
    
    if time.time() > next_telem_time:
        display_telemetry(Telemetry(vessel, flight))
        next_telem_time += TELEM_DELAY

def display_telemetry(t):
    '''
    Take a Telemetry object t and display it in a pleasing way
    '''
    # define the data to be displayed in as many columns needed
    col1 = ('Apoapsis:       {apoapsis:8,.0f}',
            'Time to apo:       {time_to_apo:5,.0f}',
            'Altitude:         {altitude:6,.0f}',
            'Orbital velocity:  {velocity:5,.0f}',
            'Latitude:          {lat:5.1f}',
            'Dynamic Pressure: {q:6,.0f}')
    
    col2 = ('Periapsis:   {periapsis: 8,.0f}',
            'Time to peri:   {time_to_peri:5,.0f}',
            'Inclination:      {inclination: 3.0f}\n',
            'Vertical speed: {vertical_speed: 5,.0f}',
            'Longitude:      {lon:5.1f}\n',
            'G-force:         {g:4.1f}')
    # zip the columns together and display them
    print('-' * 60)
    for display_line in zip(col1, col2):
        print('     '.join(display_line).format(**t.__dict__))
    print('-' * 60)
    print('\n')
                  
        
# ----------------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------------                

def still_in_atmosphere(conn):
    vessel = conn.space_center.active_vessel
    flight = vessel.flight(vessel.orbit.body.non_rotating_reference_frame)
    return flight.mean_altitude<vessel.orbit.body.atmosphere_depth

def apoapsis_way_low(vessel, ORBIT_ALT):
    '''
    True if Apoapsis is less than 95% of target apoapsis
    '''
    return vessel.orbit.apoapsis_altitude<(ORBIT_ALT*.95)

def apoapsis_little_low(vessel, ORBIT_ALT):
    '''
    True if Apoapsis is less than target apoapsis at all
    '''
    return vessel.orbit.apoapsis_altitude<ORBIT_ALT

def out_of_stages(vessel, MAX_AUTO_STAGE):
    '''
    True if no more stages left to activate
    '''
    return vessel.control.current_stage <= MAX_AUTO_STAGE

def get_resources(vessel):
    '''
    get resources of the vessel in the decouple stage
    '''
    return vessel.resources_in_decouple_stage(
        vessel.control.current_stage - 1, 
        cumulative=False)
    
def out_of_fuel(resource, fueltype):
    '''
    return True if there is fuel capacity of the fueltype, but no fuel
    '''
    return resource.max(fueltype) > 0 and resource.amount(fueltype) == 0
    
def next_stage(vessel):
    '''
    activate the next stage
    '''
    vessel.control.activate_next_stage()
    
# ----------------------------------------------------------------------------
# Activate main loop, assuming we are executing THIS file explicitly.
# ----------------------------------------------------------------------------                          
if __name__ == "__main__" : 
    main()

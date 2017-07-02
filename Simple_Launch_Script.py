######################################################################
### Fairly Robust Launch Script
######################################################################
### Kerbart and I are working on a more versatile and pythonic
### launch script that uses the same logic as this one, but I 
### thought that it was still worth posting this simpler version
### for perusal (or use!)   Set the parameters at the top of the file
### to define the orbit and ascent parameters.
######################################################################

import krpc
import time
import math

# ----------------------------------------------------------------------------
# Launch parameters
# ----------------------------------------------------------------------------
ORBIT_ALT = 95000
GRAV_TURN_FINISH = 60000  #meters by which vessel should be at 0 pitch
MAX_AUTO_STAGE = 0  # last stage to separate automatically
MAX_Q = 50000
DEPLOY_SOLAR = True
FORCE_ROLL = True
ROLL = 90
INCLINATION = 0

# ----------------------------------------------------------------------------
# Script 
# ----------------------------------------------------------------------------
REFRESH_FREQ = 2    # refresh rate in hz
TELEM_DELAY = 5     #number of seconds between telemetry updates
ALL_FUELS = ('LiquidFuel', 'SolidFuel')
MAX_PHYSICS_WARP = 3 # valid values are 0 (none) through 3 (4x)
next_telem_time=time.time()

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

class PID(object):
    '''
    Generic PID Controller! 

    '''   
    
    def __init__(self, P=1.0, I=0.1, D=0.01):   
        self.Kp = P    #P controls reaction to the instantaneous error
        self.Ki = I    #I controls reaction to the history of error
        self.Kd = D    #D prevents overshoot by considering rate of change
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.SetPoint = 0.0  #Target value for controller
        self.ClampI = 1.0  #clamps i_term to prevent 'windup.'
        self.LastTime = time.time()
        self.LastMeasure = 0.0
                
    def update(self,measure):
        now = time.time()
        change_in_time = now - self.LastTime
        if not change_in_time:
            change_in_time = 1.0   #avoid potential divide by zero if PID just created.
       
        error = self.SetPoint - measure
        self.P = error
        self.I += error
        self.I = self.clamp_i(self.I)   # clamp to prevent windup lag
        self.D = (measure - self.LastMeasure) / (change_in_time)

        self.LastMeasure = measure  # store data for next update
        self.lastTime = now

        return (self.Kp * self.P) + (self.Ki * self.I) - (self.Kd * self.D)

    def clamp_i(self, i):   
        if i > self.ClampI:
            return self.ClampI
        elif i < -self.ClampI:
            return -self.ClampI
        else:
            return i
        
    def setpoint(self, value):
        self.SetPoint = value
        self.I = 0.0


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------

def main():
    '''
    main function is run when you just execute this file, but NOT when you 
    import it into another file - thus you can choose to call ascent() later 
    to go to space, or just use the other functions in this file.
    '''
    ascent()
    
def ascent():
    '''
    Ascent Autopilot function.  Goes to space, or dies trying.
    '''
    #Setup KRPC
    conn = krpc.connect(name='Launch')
    sc = conn.space_center
    v = sc.active_vessel
    telem=v.flight(v.orbit.body.reference_frame)
    thrust_controller = PID(P=.001, I=0.0001, D=0.01)
    thrust_controller.ClampI = MAX_Q
    thrust_controller.setpoint(MAX_Q) 
    

    #Prepare for Launch
    v.auto_pilot.engage()
    v.auto_pilot.target_heading=inc_to_heading(INCLINATION)
    if FORCE_ROLL: 
        v.auto_pilot.target_roll=ROLL
    v.control.throttle=1.0
    
    #Gravity Turn Loop
    while apoapsis_way_low(v):
        gravturn(v, telem)
        autostage(v)
        limitq(v, telem, thrust_controller)
        telemetry(v, telem)
        time.sleep(1.0 / REFRESH_FREQ)        
    v.control.throttle = 0.0
    
    # Fine Tune APA
    v.auto_pilot.disengage()
    v.auto_pilot.sas=True
    time.sleep(.1)
    v.auto_pilot.sas_mode = v.auto_pilot.sas_mode.prograde
    v.auto_pilot.wait()
    boostAPA(v, telem)  #fine tune APA

    # Coast Phase
    sc.physics_warp_factor = MAX_PHYSICS_WARP
    while still_in_atmosphere(v, telem):   
        if apoapsis_little_low(v):
            sc.physics_warp_factor = 0
            boostAPA(v, telem)
            sc.physics_warp_factor = MAX_PHYSICS_WARP
        telemetry(v, telem)  
        time.sleep(1.0 / REFRESH_FREQ)       
    
    # Circularization Burn
    sc.physics_warp_factor = 0
    planCirc(v, sc.ut)
    telemetry(v, telem)
    executeNextNode(v, sc)

    # Finish Up
    if DEPLOY_SOLAR: v.control.solar_panels=True 
    telemetry(v,telem)
    v.auto_pilot.sas_mode= v.auto_pilot.sas_mode.prograde
 
# ----------------------------------------------------------------------------
# staging logic
# ----------------------------------------------------------------------------        

def autostage(vessel):
    '''
    activate next stage when there is no fuel left in the current stage
    '''
    if out_of_stages(vessel):   
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

def gravturn(vessel, flight):
    '''
    Execute quadratic gravity turn -  
    based on Robert Penner's easing equations (EaseOut)
    '''
    progress=flight.mean_altitude/GRAV_TURN_FINISH
    vessel.auto_pilot.target_pitch= 90-(-90 * progress*(progress-2))
     
def boostAPA(vessel,flight):
    '''
    function to increase Apoapsis using low thrust on a 
    tight loop with no delay for increased precision.
    '''
    vessel.control.throttle=.2
    while apoapsis_little_low(vessel):
        autostage(vessel)
        telemetry(vessel, flight) 
    vessel.control.throttle=0

def planCirc(vessel, ut):
    '''
    Plan a Circularization at Apoapsis.  
    V1 is velocity at apoapsis.  
    V2 is the velocity at apoapsis of a circular orbit.   
    Burn time uses Tsiolkovsky rocket equation.
    '''
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

def limitq(vessel, flight, controller):
    '''
    limits vessel's throttle to stay under MAX_Q using PID controller
    '''
    vessel.control.throttle= controller.update(flight.dynamic_pressure)
 
# ----------------------------------------------------------------------------
# Execute Next Maneuver Node   
# ----------------------------------------------------------------------------   
def executeNextNode(vessel, space_center):
    '''
    Executes Next Maneuver Node for vessel.  General purpose but NOT entirely 
    self contained because it makes calls to telemetry!
    If you're stealing this function, delete those or implement 
    telemetry(vessel, flight).
    '''
    node=vessel.control.nodes[0]
    flight=vessel.flight(vessel.orbit.body.reference_frame)
    
    #orient to node
    vessel.auto_pilot.sas_mode = vessel.auto_pilot.sas_mode.maneuver
    vessel.auto_pilot.wait()
    telemetry(vessel, flight) 
     
    # Warp until burn
    m = vessel.mass
    isp = vessel.specific_impulse
    dv = node.delta_v
    F = vessel.available_thrust
    G = vessel.orbit.body.surface_gravity
    
    burn_time = (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G)) 
    space_center.warp_to(node.ut - (burn_time / 2.0) - 5.0)
    while node.time_to > (burn_time / 2.0):
        telemetry(vessel, flight) 

    # burn
    vessel.control.throttle = 1.0
    while node.remaining_delta_v > .1:
        autostage(vessel)
        telemetry(vessel, flight) 
        if node.remaining_delta_v < 10:     
            # Chop throttle to 5% for last 10m/s to minimize overshoot
            vessel.control.throttle = .05
        else:                               
            # Otherwise full power burn
            vessel.control.throttle = 1.0   
            
    vessel.control.throttle=0.0
    node.remove()

# ----------------------------------------------------------------------------
# post telemetry
# ----------------------------------------------------------------------------             

def telemetry(vessel, flight):
    '''
    Show telemetry data
    TODO: split between creating telemetry data (object? dict?)
    and displaying the data. This will make it easier to transition to a
    GUI later on. For this reason, no attempts to fit the lines has been
    made (yet)
    '''
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

def still_in_atmosphere(vessel, flight):
    return flight.mean_altitude<vessel.orbit.body.atmosphere_depth

def apoapsis_way_low(vessel):
    '''
    True if Apoapsis is less than 95% of target apoapsis
    '''
    return vessel.orbit.apoapsis_altitude<(ORBIT_ALT*.95)

def apoapsis_little_low(vessel):
    '''
    True if Apoapsis is less than target apoapsis at all
    '''
    return vessel.orbit.apoapsis_altitude<ORBIT_ALT

def out_of_stages(vessel):
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

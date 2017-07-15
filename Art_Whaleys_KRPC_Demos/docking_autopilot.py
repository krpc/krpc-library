######################################################################
### Docking Autopilot Library and Example
######################################################################
###   Like all of the scripts in my folder here, this file contains
###   functions you might want to include into your own scripts for  
###   actual use and a demo in the 'main' function that you can just 
###   run to see how it works.
###
###  This file demonstrates an automatic docking.  It assumes that
###  your vessel is already on the correct side of the vessel to dock
###  to.   In the near future I'll work on code to allow it to clear
###  the target vessel safely and put itself on the correct side, but
###  for now it's a great and fairly simple example of using some
###  basic PID methods to control vessels with precision.
######################################################################

import time
import krpc
import collections
import math

from pid import PID

v3 = collections.namedtuple('v3', 'right forward up') 

##############################################################################
## Main  - only run when this file is explicitly executed
##############################################################################
def main():
    conn = krpc.connect()
    dock(conn)

##############################################################################
## dock  - The actually interesting function in this file.   
## works by lining vessel up parallel, with 10m of separation between
## docking ports.  When this is confirmed, it moves forward slowly to dock.
##############################################################################
def dock(conn, speed_limit = 1.0):

    #Setup KRPC
    sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_docking_port
    ap = v.auto_pilot
    rf = v.orbit.body.reference_frame
    
    #Setup Auto Pilot
    ap.reference_frame = rf   
    ap.target_direction = tuple(x * -1 for x in t.direction(rf))
    ap.engage()

    #create PIDs
    upPID = PID(.75,.25,1)   
    rightPID = PID(.75,.25,1)
    forwardPID = PID(.75,.2,.5)

    proceed=False  
    #'proceed' is a flag that signals that we're lined up and ready to dock.
    # Otherwise the program will try to line up 10m from the docking port.
 
    #LineUp and then dock  - in the same loop with 'proceed' controlling whether
    #we're headed for the 10m safe point, or headed forward to dock.
    while True:
        offset = getOffsets(v, t)  #grab data and compute setpoints
        velocity = getVelocities(v, t)
        if proceedCheck(offset):  #Check whether we're lined up and ready to dock
            proceed = True
        setpoints = getSetpoints(offset, proceed, speed_limit)  
        
        upPID.setpoint(setpoints.up)  #set PID setpoints
        rightPID.setpoint(setpoints.right)
        forwardPID.setpoint(setpoints.forward)
        
        v.control.up = -upPID.update(velocity.up)  #steer vessel
        v.control.right = -rightPID.update(velocity.right)
        v.control.forward = -forwardPID.update(velocity.forward)
     
        time.sleep(.1)
             
##############################################################################
##  Helper Functions
##############################################################################
def getOffsets(v, t):
    '''
    returns the distance (right, forward, up) between docking ports.
    '''
    return v3._make(t.part.position(v.parts.controlling.reference_frame))

def getVelocities(v, t):
    '''
    returns the relative velocities (right, forward, up)
    '''
    return v3._make(v.velocity(t.reference_frame))

def getSetpoints(offset, proceed, speed_limit):
    '''
    returns the computed set points -
    set points are actually just the offset distances clamped to the
    speed_limit variable!   This way we slow down as we get closer to the right
    heading.
    '''
    tvup = max(min(offset.up, speed_limit), -speed_limit)
    tvright = -1 * (max(min(offset.right, speed_limit), -speed_limit))
    if proceed:
        tvforward = -.2
    else:
        tvforward = max(min((10 - offset.forward), speed_limit), -speed_limit)
    return v3(tvright, tvforward, tvup)
   
def proceedCheck(offset):
    '''
    returns true if we're lined up and ready to move forward to dock.
    '''
    return (offset.up < .1 and
            offset.right < .1 and
            math.fabs(10 - offset.forward)<.1)
             

 
##This calls the main function which is at the top of the file for readability's sake!
if __name__ == '__main__':
    main()
    print('--')

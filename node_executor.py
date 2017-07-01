####################################################################
##                    Node Executor Script
##
##  Contains functions to execute one, or all maneuver nodes created
##                        for a given vessel.
####################################################################


import krpc
import math
import time

def main():
    conn = krpc.connect()
    sc = conn.space_center
    v = sc.active_vessel

    execute_next_node(v, sc)
  #  execute_all_nodes(v, sc)       #executes ALL nodes instead of just the next one!
 
def execute_next_node(vessel, space_center):
    '''
    Executes the Next Maneuver Node for the vessel provided.
    If you just open and run this file, it will execute a node and exit.
    You can also include this file into your own script with the line

    import node_executor

    at the top of your script, and then anytime you want to execute a node
    you just have to call (execute_next_node(vessel, space_center) with your
    objects referencing the active vessel and the connection's space_center
    of course.

    I'm also demonstrating two different ways to point the vessel with the
    autopilot.  One relies on the vessel having SAS Node holding capabilty,
    the other uses the KRPC built-in auto-pilot.   The one built into
    KRPC can require some tuning depending on your vessel...  but works on
    any vessel regardless of pilot skill/probe core choice!   
    '''
# Grab the next node and the autopilot object
    try:
        node = vessel.control.nodes[0]
    except Exception:
        return    #Fail silently but gracefully if there was no node to execute
    
    
# Orient vessel to the node
    ap=vessel.auto_pilot

################## One Way To Orient Vessel!##############
    rf= vessel.orbit.body.reference_frame
    ap.reference_frame=rf
    ap.engage()
    ap.target_direction=node.remaining_burn_vector(rf)
    ap.wait()

##################  Another Way To Orient Vessel!########
    #ap.sas = True
    #time.sleep(.1)
    #ap.sas_mode = vessel.auto_pilot.sas_mode.maneuver
    #ap.wait()
        
# Calculate the length and start of burn
    m = vessel.mass
    isp = vessel.specific_impulse
    dv = node.delta_v
    F = vessel.available_thrust
    G= vessel.orbit.body.surface_gravity
    burn_time = (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

# Warp until burn
    space_center.warp_to(node.ut - (burn_time / 2.0) - 5.0)
    while node.time_to > (burn_time / 2.0):
        pass
    ap.wait()
    
# Actually Burn
    vessel.control.throttle = thrust_controller(vessel, node.remaining_delta_v)  
    while node.remaining_delta_v > .1:
        ap.target_direction=node.remaining_burn_vector(rf)#comment out this line
        #if using the vessel sas method to orient vessel
        vessel.control.throttle=thrust_controller(vessel, node.remaining_delta_v)  

# Finish Up
    ap.disengage()
    vessel.control.throttle = 0.0
    node.remove()

def execute_all_nodes(vessel, space_center):
    '''
    as the name implies - this function executes ALL maneuver nodes currently
    planned for the vessel in series.
    '''
    while vessel.control.nodes:
        execute_next_node(vessel, space_center)

def thrust_controller(vessel, deltaV):
    '''
    This function is somewhat arbitrary in it's working - there's not a 'rule'
    that I've found on how to feather out throttle towards the end of a burn
    but given that the chances of overshooting go up with TWR (since we fly
    in a world with discrete physics frames!) it makes sense to relate the
    throttle to the TWR for this purpose.
    '''
    TWR= vessel.max_thrust/vessel.mass
    if deltaV<TWR/3:
        return .05
    elif deltaV<TWR/2:
        return .1
    elif deltaV<TWR:
        return .25
    else:
        return 1.0

# ----------------------------------------------------------------------------
# Activate main loop, if we are executing THIS file explicitly.
# ----------------------------------------------------------------------------                          
if __name__ == "__main__" : 
    main()

import krpc
import time
import math
from collections import namedtuple
from math import atan2,degrees,cos,sqrt,asin,sin,radians

from pid import PID

latlon =  namedtuple('latlon', 'lat lon')

def main():
    conn = krpc.connect()

    #Create a waypoint to drive to
    wp1 = conn.space_center.waypoint_manager.add_waypoint(
        0.5,-78.12345, conn.space_center.active_vessel.orbit.body,"Waypoint1")

    #call the rover autopilot
    rover_go(conn, wp1)

def rover_go(conn, waypoint, speed = 10.0, savetime = 300 ):
    '''
    Function to drive a rover to the specified waypoint.  Must be called with
    an active KRPC connection and a valid waypoint.   Attempts to bring rover
    to a complete stop and quicksave a file called "rover_ap" at a regular 
    interval.  Defaults to 5 minutes.   This and rover speed can be specified
    as optional arguments.  A savetime of 0 turns this feature off.
    '''
    v=conn.space_center.active_vessel
    ground_telem=v.flight(v.orbit.body.reference_frame)
    surf_telem=v.flight(v.surface_reference_frame)
    target=latlon(waypoint.latitude, waypoint.longitude)

    steering= PID(.01,.01,.001)
    throttle = PID(.5,.01,.001)
    steering.setpoint(0) 
    throttle.setpoint(speed)
    autosave.lastsave = time.time()
    there_yet = False;

    while not there_yet:

        autosave(conn, savetime)

        #Steering
        location = latlon(ground_telem.latitude, ground_telem.longitude)
        target_heading = heading_for_latlon(target, location)
        course_correct= course_correction(surf_telem.heading, target_heading)
        steer_correct = steering.update(course_correct)
        v.control.wheel_steering= steer_correct

        #Throttle Management
        throttsetting=   throttle.update(ground_telem.speed)
        v.control.wheel_throttle = throttsetting

        #Check if we're there
        if distance(target, location, v.orbit.body) < 50:
            there_yet=True




def autosave(conn, savetime):
    if savetime == 0:
        return
    if time.time() - autosave.lastsave > savetime:
        v=conn.space_center.active_vessel
        telem=v.flight(v.orbit.body.reference_frame)

        if True: # placeholder for a set of tests to see if it's safe to save
            v.control.throttle = 0.0
            v.control.brakes = True
            while telem.speed > 0.01:
                pass
            time.sleep(.1)
            conn.space_center.save('rover_ap')
            v.control.brakes = False
            autosave.lastsave = time.time()
     
 

################################################################
##  Navigation Math Functions
################################################################
def heading_for_latlon(target, location):

    lat1 = math.radians(location.lat)
    lat2 = math.radians(target.lat)

    diffLong = math.radians(target.lon - location.lon)

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def distance(target, location, body):
  R = body.equatorial_radius # Earth radius in kilometers
 
  dLat = radians(target.lat - location.lat)
  dLon = radians(target.lon - location.lon)
  lat1 = radians(location.lat)
  lat2 = radians(target.lat)
 
  a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
  c = 2*asin(sqrt(a))
 
  return R * c

def course_correction(myheading, targetbearing):
    unadjusted= targetbearing-myheading
    if unadjusted < -180:
        return unadjusted +360
    if unadjusted > 180:
        return unadjusted -360
    return unadjusted


# ----------------------------------------------------------------------------
# Activate main loop, if we are executing THIS file explicitly.
# ----------------------------------------------------------------------------                          
if __name__ == "__main__" : 
    main()

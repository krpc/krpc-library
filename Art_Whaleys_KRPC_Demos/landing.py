######################################################################
### Automated Landing Library and Example
######################################################################
###   Like all of the scripts in my folder here, this file contains
###   functions you might want to include into your own scripts for  
###   actual use and a demo in the 'main' function that you can just 
###   run to see how it works.
###
###  This file shows how to complete an automated landing.  It
###  does so in three phases - a deorbital burn, a suicide burn to a 
###  safety altitude and speed, and then a speed limited descent to
###  final touch down.
######################################################################

import krpc
import math
import time
import numpy as np

from pid import PID

##############################################################################
###   Main function - demonstrates the use of this library. Simply lands
###   a vessel.   Only propulsive landing - so really designed for planets
###   without an atmosphere.
##############################################################################

def main():
        ''' Main function that demonstrates how to use the pieces of
        this landing library.

        Connects to KRPC server, performs a deorbit burn, then a
        suicide burn, then a final descent.
        '''
        conn = krpc.connect()
        sc = conn.space_center
        v = sc.active_vessel


        deorbit(v, .3)
        suicide_burn(conn, v)
        final_descent(v)

###############################################################################
###     High Level Functions
###  
###     These are the ones you'd actually maybe want to call from your script.
###############################################################################

def deorbit(vessel, eccentricity):
        '''executes a deorbit burn until the given eccentricity
        is achieved.   If periapsis remains above the equatorial
        radius, it continues burning.
        '''
        print ("Deorbiting Vessel...")
        vessel.control.speed_mode = vessel.control.speed_mode.surface
        ap = vessel.auto_pilot
        ap.sas = True
        time.sleep(.1)
        ap.sas_mode = ap.sas_mode.retrograde

        while (vessel.orbit.eccentricity < eccentricity or
                   vessel.orbit.periapsis_altitude > 0):
                vessel.control.throttle = 1.0
        vessel.control.throttle = 0.0
		
def suicide_burn(conn, vessel, autowarp = True):
        '''
        Performs a 'Suicide Burn' using the calculator class.
        '''
        print "Calculating Suicide Burn..."
        telem = vessel.flight(vessel.orbit.body.reference_frame)
        vessel.control.speed_mode = vessel.control.speed_mode.surface
        rf = vessel.orbit.body.reference_frame
        ap = vessel.auto_pilot
        ap.sas = True
        time.sleep(.1)
        ap.sas_mode = ap.sas_mode.retrograde

        ##calculate initial burn
        computer = suicide_burn_calculator(conn, vessel, 5000)
        computer.update()  # run once with altitude of 5000m to get an estimated groundtrack distance
        touchdown = coords_down_bearing(telem.latitude, telem.longitude, (180 + telem.heading),computer.ground_track,vessel.orbit.body)
        #update actual safe altitude over the groundtrack
        computer.alt = check_terrain(telem.latitude, telem.longitude, touchdown[0], touchdown[1], vessel.orbit.body)
        
        #Initial burn - aiming for the 'safe alt' (highest point over course)
        # and a vertical velocity of 10 m/s
        burn_until_velocity(conn, vessel, telem, 10, computer, True)
        
        ## update computer ti aim for 10m above current altitude.
        computer.alt = vessel.orbit.body.surface_height(telem.latitude, telem.longitude) + 10

        #Second half of burn - aiming for ground alt + 10m and a vertical
        #velocity of 5 m/s
        burn_until_velocity(conn, vessel, telem, 5, computer, True)
        
def burn_until_velocity(conn, vessel, telem, thresh, computer, autowarp):
        '''
        Helper Function for suicide_burn function - actually executes a burn
        at the calculated time and burns until it reaches the threshold
        vertical velocity.
        
        '''
        countdown = computer.update()
        print ("Burn in {} seconds".format(countdown))
        
        if autowarp and (countdown > thresh):
                conn.space_center.warp_to(conn.space_center.ut + countdown - 10)  # warp to 10 seconds before burn
                
        while countdown > 0.0:  #  Wait until suicide burn
                time.sleep(.1)
                countdown = computer.update()
                
        while telem.vertical_speed < (-1 * thresh):  #Loop until we're ready for final descent
                countdown = computer.update()
                vessel.control.throttle= .95  #95% throttle 
                if countdown < 0.0:  #use the emergencty 5% of throttle when needed
                        vessel.control.throttle = 1.0 
        vessel.control.throttle = 0.0        

def final_descent(v):
        ''' manages final descent.  At the moment keeps vertical velocity limited to 1/10 of the
        current altitude above terrain - so at 200m would try to descend at 20m/s and at 10 m/s locks
        descent speed to 1m/s.
        '''
        print "final descent"
        telem = v.flight(v.orbit.body.reference_frame)
        v.control.speed_mode = v.control.speed_mode.surface
        ap = v.auto_pilot
        ap.sas = True
        time.sleep(.1)
        ap.sas_mode = ap.sas_mode.retrograde
        p = PID(.25, .25, .025)
        while v.situation is not v.situation.landed:
                #ap.sas_mode = ap.sas_mode.retrograde   # SAS leaves retrograde mode if velocity gets to zero.
                safe_descent = telem.surface_altitude / -10
                #print safe_descent
                if safe_descent < -15.0:
                                safe_descent = -15.0
                p.setpoint(safe_descent)
                v.control.throttle = p.update(telem.vertical_speed)
        v.control.throttle = 0

###############################################################################
##     Burn Calculator Class
###############################################################################


class suicide_burn_calculator(object):
        '''
        Class that calculates time until suicide burn.
        '''
        def __init__(self, conn, v, alt):
                self.conn = conn
                self.v = v
                self.sc = conn.space_center
                self.burn_time = np.inf
                self.burn_duration = np.inf
                self.ground_track = np.inf
                self.effective_decel = np.inf
                self.radius = np.inf
                self.angle_from_horizontal = np.inf
                self.impact_time = np.inf
                self.alt = alt


                self.rf = self.sc.ReferenceFrame.create_hybrid(
                        position=self.v.orbit.body.reference_frame,
                        rotation=self.v.surface_reference_frame)

        def update(self):
                '''
                Returns an estimate of how many seconds until you need to burn at 95% throttle to avoid crashing.
                This gives a 5% safety margin.
                I do not even PRETEND to understand all of the math in this function.  It's essentially a porting
                of the routine from the Mechjeb orbit extensions.
                '''

                if self.v.orbit.periapsis_altitude > 0:    ## We're not on a landing trajectory yet.
                        self.burn_time = np.inf
                        self.burn_duration = np.inf
                        self.ground_track = np.inf
                        self.effective_decel = np.inf
                        self.angle_from_horizontal = np.inf
                        self.impact_time = np.inf
                        return self.burn_time
                

                rf = self.v.orbit.body.reference_frame
                
                #calculate sin of angle from horizontal - 
                v1 = self.v.velocity(self.rf)
                v2 = (0,0,1)
                self.angle_from_horizontal =  angle_between(v1, v2)
                sine = math.sin(self.angle_from_horizontal)

                #estimate deceleration time
                g = self.v.orbit.body.surface_gravity
                T = (self.v.max_thrust / self.v.mass) *.95  # calculating with 5% safety margin!
                self.effective_decel = .5 * (-2 * g * sine + math.sqrt((2 * g * sine) * (2 * g * sine) + 4 * (T*T - g*g)))
                self.decel_time = self.v.flight(self.rf).speed / self.effective_decel

                #estimate time until burn
                radius = self.v.orbit.body.equatorial_radius + self.alt
                TA = self.v.orbit.true_anomaly_at_radius(radius)
                TA = -1 * TA   #look on the negative (descending) side of the orbit
                self.impact_time = self.v.orbit.ut_at_true_anomaly(TA)
                self.burn_time = self.impact_time - self.decel_time/2
                self.ground_track = ((self.burn_time- self.sc.ut) * self.v.flight(self.rf).speed) + (
                        .5 * self.v.flight(self.rf).speed * self.decel_time)
                return self.burn_time - self.sc.ut

    



###############################################################################
##     Ground Navigation Functions   -   Probably ought to move to their
##                                        own library file some day.
###############################################################################
        
def coords_down_bearing(lat, lon, bearing, distance, body):
        '''
        Takes a latitude, longitude and bearing in degrees, and a
        distance in meters over a given body.  Returns a tuple
        (latitude, longitude) of the point you've calculated.
        '''
        bearing = math.radians(bearing)
        R = body.equatorial_radius
        lat = math.radians(lat)
        lon = math.radians(lon)
        
        lat2 = math.asin( math.sin(lat)*math.cos(distance/R) +
                     math.cos(lat)*math.sin(distance/R)*math.cos(bearing))

        lon2 = lon + math.atan2(math.sin(bearing)*math.sin(distance/R
                        )*math.cos(lat),math.cos(distance/R)-math.sin(lat
                        )*math.sin(lat2))

        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        return (lat2, lon2)


def check_terrain(lat1, lon1, lat2, lon2, body):
        '''
        Returns an estimate of the highest terrain altitude betwen
                two latitude / longitude points.
        '''
        lat = lat1
        lon = lon1
        highest_lat = lat
        highest_lon = lon
        highest_alt = body.surface_height(lat, lon)
        latstep = (lat2 - lat1) / 20
        lonstep = (lon2 - lon1) / 20

        for x in range (20):
                test_alt = body.surface_height(lat, lon)
                if  test_alt> highest_alt:
                        highest_lat = lat
                        highest_lon = lon
                        highest_alt = test_alt
                lat = lat+latstep
                lon = lon+lonstep
        return highest_alt



###############################################################################
##     Vector Math Functions        -   Probably ought to move to their
##                                        own library file some day.
###############################################################################

def unit_vector(vector):
	""" Returns the unit vector of the vector provided.  """
	return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
	""" Returns the angle in radians between vectors 'v1' and 'v2'"""
	v1_u = unit_vector(v1)
	v2_u = unit_vector(v2)
	return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


main()


	

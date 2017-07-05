import time
import krpc
import collections
import math

speed_limit = 1.0
v3 = collections.namedtuple('v3', 'right forward up')
####################################################
## Main
####################################################
def main():
    #Setup KRPC
    conn = krpc.connect()
    sc = conn.space_center
    v = sc.active_vessel
    t = sc.target_docking_port
    ap = v.auto_pilot
    rf = v.orbit.body.reference_frame
    
    ap.reference_frame = rf  #setup auto_pilot
    ap.target_direction = tuple(x * -1 for x in t.direction(rf))
    ap.engage()

    upPID = PID(.75,.25,1)   #create PIDs
    rightPID = PID(.75,.25,1)
    forwardPID = PID(.75,.2,.5)

    proceed=False  #If we're on course, move forward and dock
 
    #LineUp and then dock
    while True:
        offset = getOffsets(v, t)  #grab data and compute setpoints
        velocity = getVelocities(v, t)
        if proceedCheck(offset):
            proceed = True
        setpoints = getSetpoints(offset, proceed)
        
        upPID.setpoint(setpoints.up)  #set PID setpoints
        rightPID.setpoint(setpoints.right)
        forwardPID.setpoint(setpoints.forward)
        
        v.control.up = -upPID.update(velocity.up)  #steer vessel
        v.control.right = -rightPID.update(velocity.right)
        v.control.forward = -forwardPID.update(velocity.forward)
     
        time.sleep(.1)
             
def getOffsets(v, t):
    '''
    returns the distance (right, forward, up) between docking ports.
    '''
    return v3._make(t.part.position(v.parts.controlling.reference_frame))

def getVelocities(v, t):
    '''
    returns the relative velocities right, forward, up)
    '''
    return v3._make(v.velocity(t.reference_frame))

def getSetpoints(offset, proceed):
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
             

class PID(object):
    '''
    Generic PID Controller Class
    You shouldn't have to modify anything in it!
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

 
##This calls the main function which is at the top of the file for readability's sake!
if __name__ == '__main__':
    main()
    print('--')

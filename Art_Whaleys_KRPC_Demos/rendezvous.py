import krpc
import node_executor

def main():
    global conn
    conn = krpc.connect()
    sc = conn.space_center
    v = sc.active_vessel
    t= sc.target_vessel

    match_velocities(v,t,sc)



def match_velocities(vessel, targetvessel, space_center):
    rv = targetvessel.velocity(vessel.orbital_reference_frame)
    print rv
    p = rv[1]
    n = rv[2]
    r = rv[0]
    vessel.control.add_node(space_center.ut, p, n, r)



# ----------------------------------------------------------------------------
# Activate main loop, if we are executing THIS file explicitly.
# ----------------------------------------------------------------------------                          
if __name__ == "__main__" : 
    main()

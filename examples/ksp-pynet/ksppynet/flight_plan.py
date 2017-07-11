#!/usr/bin/python3
import asyncio
import argparse
import krpc
import time
import datetime
import os
import sys
from ksppynet.flight_plan_node import ManeuverNode

__all__ = (
    "FlightPlan",
)

class KStream(object):
    def __init__(self, conn, item, attr):
        self.stream = conn.add_stream(getattr, item, attr)

    def __call__(self):
        return self.stream()

class FlightPlan(object):
    def msg(self, s, duration=5):
        print("{} <==> {}".format(datetime.datetime.now().isoformat(' '), s))
        self.conn.ui.message(s, duration=duration)
        if self.debug_handler:
            self.debug_handler(s, duration=duration)


    def __init__(self, conn, vessel, debug_handler=None):
        self.debug_handler = debug_handler
        self.conn = conn
        self.vessel = vessel
        self.ap = vessel.auto_pilot
        self.autostaging_disabled = False
        # Defaults
        self.loop = asyncio.get_event_loop()
        self.attr = {}
        self.attr["ut"] = KStream(conn, conn.space_center, 'ut')
        self.attr["altitude"] = KStream(conn, vessel.flight(), 'mean_altitude')
        self.attr["vertical_speed"] = KStream(conn, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
        self.attr["apoapsis"] = KStream(conn, vessel.orbit, 'apoapsis_altitude')
        self.attr["periapsis"] = KStream(conn, vessel.orbit, 'periapsis_altitude')
        self.attr["real_apoapsis"] = KStream(conn, vessel.orbit, 'apoapsis')
        self.attr["real_periapsis"] = KStream(conn, vessel.orbit, 'periapsis')
        self.attr["eccentricity"] = KStream(conn, vessel.orbit, 'eccentricity')

        self.sequence = asyncio.Queue()
        self.seq_defs = {
            "pre_launch" : self._pre_launch,
            "launch" : self._launcher,
            "deorbit" : self._deorbiter,
            "orbit" : self._orbiter,
            "land" : self._lander,
            "quit" : self._quit
        }

    def add_sequence(self, name, *args, **kwargs):
        self.msg("Appending sequence: {}".format(name))
        asyncio.async(self.sequence.put((name,
                                         self.seq_defs[name],
                                         args,
                                         kwargs)))

    @asyncio.coroutine
    def _quit(self):
        self.loop.stop()

    @asyncio.coroutine
    def _start_sequence(self):
        while True:
            seq = yield from self.sequence.get()
            # Wait for the coroutine with its args/kwargs before dequeuing
            # the next in the sequence
            yield from asyncio.wait_for(seq[1](*seq[2], **seq[3]), None)

    @asyncio.coroutine
    def _autostager(self):
        while True:
            yield
            if self.autostaging_disabled:
                self.msg("Autostaging Disabled")
                return
            stage = self.vessel.control.current_stage
            parts = self.vessel.parts.in_stage(stage)
            for part in parts:
                if part.parachute:
                    self.msg("Chutes in stage. Disabling autostaging")
                    return

            parts = self.vessel.parts.in_decouple_stage(stage-1)
            fuel_in_stage = False
            for part in parts:
                engine = part.engine
                if engine and engine.active and engine.has_fuel:
                    fuel_in_stage = True


            if not fuel_in_stage:
                self.msg("No fuel in stage. Staging...")
                self.vessel.control.activate_next_stage()
            else:
                yield from asyncio.sleep(0.2)

    @asyncio.coroutine
    def _launcher(self, altitude):
        self.msg("Executing Launch")
        self.desired_altitude = altitude
        self.turn_start_altitude = 250.0
        self.turn_mid_altitude = self.vessel.orbit.body.atmosphere_depth * 0.60
        self.turn_end_altitude = self.vessel.orbit.body.atmosphere_depth * 0.80
        def proportion(val, start, end):
            return (val - start) / (end - start)
        while True:
            yield
            altitude = self.attr["altitude"]()
            apoapsis = self.attr["apoapsis"]()
            if altitude < self.turn_start_altitude:
                self.ap.target_pitch_and_heading(90,self.desired_heading)
            elif self.turn_start_altitude <= altitude < self.turn_mid_altitude:
                #Only shallow out once we've got through the thicker part of the atmosphere.
                frac = proportion(altitude,
                                  self.turn_start_altitude,
                                  self.turn_mid_altitude)
                self.ap.target_pitch_and_heading(45 + 45*(1-frac),self.desired_heading)
            elif self.turn_mid_altitude <= altitude < self.turn_end_altitude:
                frac = proportion(altitude,
                                  self.turn_mid_altitude,
                                  self.turn_end_altitude)
                self.ap.target_pitch_and_heading(35*(1-frac)+5 ,self.desired_heading)
            else:
                self.ap.target_pitch_and_heading(5, self.desired_heading)

            if altitude > self.vessel.orbit.body.atmosphere_depth:
                fudge_factor = 1.0
            else:
                #Try and overshoot the desired altitude a little to account for resistence in the atmosphere
                fudge_factor = 1 + (self.vessel.orbit.body.atmosphere_depth - altitude) / (25 * self.vessel.orbit.body.atmosphere_depth)
            if apoapsis > self.desired_altitude * fudge_factor:
                self.vessel.control.throttle = 0
                if altitude > self.vessel.orbit.body.atmosphere_depth * 0.90:
                    # Wait until we're mostly out of the atmosphere before setting maneuver nodes
                    self.ap.disengage()
                    return
            #else: control the throttle?

    @asyncio.coroutine
    def warp_to(self, target_ut, orig_warp_factor=4, lead_time=5):
        while True:
            yield
            warp_factor = orig_warp_factor
            if self.conn.space_center.rails_warp_factor != warp_factor:
                if not self.conn.space_center.can_rails_warp_at(warp_factor):
                    warp_factor = self.conn.space_center.maximum_rails_warp_factor
                self.conn.space_center.rails_warp_factor = warp_factor

            ut = self.attr["ut"]()
            if ut > target_ut - lead_time:
                self.msg("Warp finished")
                self.drop_warp()
                return

    def drop_warp(self):
        self.conn.space_center.rails_warp_factor = 0
        self.conn.space_center.physics_warp_factor = 0

    @asyncio.coroutine
    def _pre_launch(self, heading):
        self.msg("Executing Prelaunch")
        self.desired_heading = heading
        self.vessel.control.sas = True # Is this ok?
        self.vessel.control.rcs = False
        self.vessel.control.throttle = 1
        self.ap.reference_frame = self.vessel.surface_reference_frame
        self.ap.target_pitch_and_heading(90, self.desired_heading)
        self.ap.target_roll = float('nan')
        self.ap.engage()

    @asyncio.coroutine
    def _deorbiter(self, periapsis, end_altitude):
        self.msg("Executing Deorbit")
        self.ap.reference_frame = self.vessel.orbital_reference_frame
        destage_altitude = self.vessel.orbit.body.atmosphere_depth * 0.90
        self.ap.target_direction = (0,-1,0)
        yield from asyncio.sleep(10) ## wait to turn.
        self.ap.engage()
        while True:
            yield
            cur_periapsis = self.attr["periapsis"]()
            self.ap.target_direction = (0,-1,0)
            if cur_periapsis > periapsis:
                self.vessel.control.throttle = 0.5
            else:
                self.vessel.control.throttle = 0
                break

        ut = self.attr["ut"]()
        self.loop.create_task(self.warp_to(ut + vessel.orbit.time_to_periapsis))
        # The warp should stop in the atmosphere.
        while True:
            yield
            altitude = self.attr["altitude"]()
            if altitude < destage_altitude:
                break
        #disable warping
        self.drop_warp()
        self.msg("Turning")
        self.ap.target_direction = (0,-1,0)
        yield from asyncio.sleep(10) ## wait to turn.
        self.msg("Deceleration burn")
        self.vessel.control.throttle = 1
        yield from asyncio.sleep(20) ## Crude
        self.vessel.control.throttle = 0
        yield from asyncio.sleep(1) # Wait to check throttle is off before destaging
        self.autostaging_disabled = True

        chutes = False
        while not chutes:
            stage = self.vessel.control.current_stage
            parts = self.vessel.parts.in_stage(stage-1)
            for part in parts:
                if part.parachute:
                    chutes = True
            if chutes:
                self.msg("Chutes in next stage.")
            else:
                self.msg("Destaging for landing")
                self.vessel.control.activate_next_stage()

        self.msg("Deorbit Complete, brace for landing!!")
        self.ap.disengage()

    def get_node(self):
        return ManeuverNode(self.conn, self.vessel, self.attr["ut"])

    @asyncio.coroutine
    def _orbiter(self, apoapsis, periapsis):
        node = self.get_node()

        self.msg("Changing Periapsis to new Apoapsis {}".format(apoapsis))
        node.change_periapsis(apoapsis)
        yield from asyncio.wait_for(node.execute(), None)

        self.msg("Changing new Periapsis to {}".format(periapsis))
        node.change_apoapsis(periapsis)
        yield from asyncio.wait_for(node.execute(), None)

    @asyncio.coroutine
    def _lander(self, chute_altitude):
        self.msg("Executing Landing")
        self.ap.reference_frame = self.vessel.orbital_reference_frame
        self.ap.target_direction = (0,-1,0)
        self.ap.engage()
        while True:
            yield
            altitude = self.attr["altitude"]()
            self.ap.target_direction = (0,-1,0)
            if altitude < chute_altitude:
                while True:
                    stage = self.vessel.control.current_stage
                    parts = self.vessel.parts.in_stage(stage-1)
                    self.vessel.control.activate_next_stage()
                    for part in parts:
                        if part.parachute:
                            self.msg("Chute stage activated")
                            return

    def run_sequence(self):
        self.loop.create_task(self._start_sequence())

    def launch(self):
        # Start the sequence
        self.run_sequence()
        for i in range (5, 0, -1):
            self.msg("{} ...".format(i))
            time.sleep(1)

        self.vessel.control.activate_next_stage()

    def set_autostaging(self):
        self.loop.create_task(self._autostager())

    def close(self):
        self.conn.close()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--apoapsis', type=int)
    parser.add_argument('--periapsis', type=int)
    parser.add_argument('--down', action='store_true')
    parser.add_argument('--diags', action='store_true')
    parser.add_argument('--test',  nargs='+')
    args = parser.parse_args()


    conn = krpc.connect(name="{}.{}".format(os.path.basename(__file__), os.getpid()))
    vessel = conn.space_center.active_vessel

    fp = FlightPlan(conn, vessel)

    loop = asyncio.get_event_loop()
    def exit_on_exception(loop, context):
        print(context["message"])
        if (not "exception" in context
            or not isinstance(context["exception"], ConnectionResetError)):
            loop.default_exception_handler(context)
        loop.stop()
    loop.set_exception_handler(exit_on_exception)

    if args.diags:
        #print("Vessel")
        #print(dir(vessel))
        #print("Orbit")
        #print(dir(vessel.orbit))
        #print("Periapsis:\nperiapsis: {}, altitude: {}".format(vessel.orbit.periapsis, vessel.orbit.periapsis_altitude))
        #print("Apoapsis:\napoapsis: {}, altitude: {}".format(vessel.orbit.apoapsis, vessel.orbit.apoapsis_altitude))
        #print(dir(vessel.orbit.body))
        print("Altitude: {}, Destage at: {}, Vertical speed: {}".format(vessel.flight().mean_altitude,
                                                    vessel.orbit.body.atmosphere_depth * 0.90,
                                                    vessel.flight().vertical_speed))
        print("Altitude: {}, Destage at: {}, Vertical speed: {}".format(vessel.flight().mean_altitude,
                                                    vessel.orbit.body.atmosphere_depth * 0.90,
                                                    vessel.flight(vessel.orbit.body.reference_frame).vertical_speed))
    elif args.test:
        if args.test[0] == "warp":
            assert(args.test[1])
            fp.msg("Warping ahead {} seconds".format(args.test[1]))
            loop.create_task(fp.warp_to(fp.attr["ut"]() + float(args.test[1])))

    elif args.down:
        #fp.msg("Changing Apoapsis to {}".format(70000))
        #node = fp.get_node()
        #node.change_apoapsis(70000)
        #loop.run_until_complete(node.execute())
        fp.add_sequence("deorbit", periapsis=45000, end_altitude=4000)
        fp.add_sequence("land", chute_altitude=3000)
        fp.run_sequence()
    elif args.apoapsis:
        apoapsis = args.apoapsis
        node = fp.get_node()
        fp.msg("Changing Apoapsis to {}".format(apoapsis))
        node.change_apoapsis(apoapsis)
        loop.create_task(node.execute())
    elif args.periapsis:
        periapsis = args.periapsis
        node = fp.get_node()
        fp.msg("Changing Periapsis to {}".format(periapsis))
        node.change_periapsis(periapsis)
        loop.create_task(node.execute())
    else:
        fp.set_autostaging()

        fp.add_sequence("pre_launch", heading=90)
        fp.add_sequence("launch", altitude=80000)
        #fp.add_sequence("orbit", apoapsis=100000, periapsis=100000)
        fp.add_sequence("orbit", apoapsis=75000, periapsis=75000)
        fp.add_sequence("quit")
        #fp.add_sequence("deorbit", periapsis=45000, end_altitude=4000)
        #fp.add_sequence("land", chute_altitude=3000)

        fp.launch()

    pending = asyncio.Task.all_tasks()
    loop.run_until_complete(asyncio.gather(*pending))

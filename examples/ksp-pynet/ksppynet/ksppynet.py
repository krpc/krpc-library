#!/usr/bin/python3
import asyncio
import ksppynet.flight_plan as flight_plan
from threading import Thread
import krpc
import queue
import traceback
import os

__all__ = (
    "Pynet",
)

testing_ui_on = False

class PynetException(Exception):
    pass

class PynetHandler(object):
    # Main thread
    def __init__(self):
        print("Setting up message queues and event loop")
        self.requests = queue.Queue()
        self.replies = queue.Queue()
        self.fp = None

        self.methods = {
            "connect" : self.D_connect,
            "disconnect" : self.D_disconnect,
            "plan" : self.D_plan,
            "orbital" : self.D_orbital,
            "maneuver" : self.D_maneuver,
        }
        if testing_ui_on:
            self.methods = {
                "orbital" : self.D_test_orbital,
            }

    def default_method(self, *args, **kwargs):
        if not hasattr(self, "test_counter"):
            self.test_counter = 0
        self.test_counter += 1
        self.message("Sleep for {}".format(self.test_counter),
                     self.test_counter)
        return self.test_counter


    def D_test_orbital(self, on):
        self.test_counter += 1
        pm = PynetMessage("orbital_notify")
        pm.set_result({"body_name": "Earth",
                       "body_radius" : 500000,
                       "real_apoapsis" : 500000 + 10000 * self.test_counter,
                       "real_periapsis" : 400000})
        self.replies.put(pm)

    def D_maneuver(self, **kwargs):
        node = self.fp.get_node()
        mfns = { "apoapsis" : node.change_apoapsis,
                 "periapsis" : node.change_periapsis }
        for k, v in kwargs.items():
            self.fp.msg("Changing {} to {}".format(k, v))
            mfns[k](v)
            self.loop.create_task(node.execute())

    @asyncio.coroutine
    def send_orbital_params(self):
        self.sending_orbital = True
        while self.sending_orbital:
            yield from asyncio.sleep(0.2)
            pm = PynetMessage("orbital_notify")
            pm.set_result({"body_name" : self.fp.vessel.orbit.body.name,
                           "body_radius" : self.fp.vessel.orbit.body.equatorial_radius,
                           "real_apoapsis" : self.fp.attr["real_apoapsis"](),
                           "real_periapsis" : self.fp.attr["real_periapsis"]()})
            self.replies.put(pm)


    def D_orbital(self, on):
        if on:
            self.loop.create_task(self.send_orbital_params())
        else:
            self.sending_orbital = False


    def D_plan(self, heading=0, target_altitude=75000,
               apoapsis=90000, periapsis=90000):
        self.fp.set_autostaging()

        self.fp.add_sequence("pre_launch", heading=heading)
        self.fp.add_sequence("launch", altitude=target_altitude)
        self.fp.add_sequence("orbit", apoapsis=apoapsis, periapsis=periapsis)
        self.fp.add_sequence("quit")
        self.fp.launch()

    def D_disconnect(self):
        self.disconnect(self)

    def D_connect(self, ip=None, port=None):
        if self.fp:
            raise PynetException("Already connected")
        conn = krpc.connect(name="{}.{}".format("KSP-PyNet", os.getpid()))
        vessel = conn.space_center.active_vessel
        self.fp = flight_plan.FlightPlan(conn, vessel, self.message)
        return "Connected with IP {}, PORT {}".format(ip, port)

    def message(self, msg, duration=10):
        pm = PynetMessage("message")
        pm.set_result({"msg" : msg, "duration" : duration})
        self.replies.put(pm)

    def debug(self, msg):
        pm = PynetMessage("debug")
        pm.set_result({"msg" : msg})
        self.replies.put(pm)

    def disconnect(self):
        if self.fp:
            self.fp.close()
            self.fp = None
        self.replies.put(PynetMessage("disconnect"))

    @asyncio.coroutine
    def dispatch(self, req, *args, **kwargs):
        try:
            if req in self.methods:
                result = self.methods[req](*args, **kwargs)
            else:
                result = self.default_method(*args, **kwargs)
        except Exception as e:
            result = {"error" : str(traceback.format_exc()) }
        if not isinstance(result, dict):
            result = {"result" : result }
        return result

    @asyncio.coroutine
    def request_handler(self):
        print("Started request handler")
        while True:
            try:
                msg = self.requests.get(False) # Don't block
            except queue.Empty:
                yield
            else:
                print("Got request")
                msg.result = yield from self.dispatch(msg.req, *msg.args, **msg.kwargs)
                print("Sending reply: {}".format(msg.result))
                self.replies.put(msg)
                if msg.req == "disconnect":
                    print("Disconnecting PyNet")
                    self.loop.stop()
                    return

    def exception_handler(self, loop, context):
        print(context["message"])
        if (not "exception" in context
            or not isinstance(context["exception"], ConnectionResetError)):
            self.loop.default_exception_handler(context)
            self.debug("Hit exception:\n{}".format(context["message"]))
        self.loop.stop()

    def run(self, loop):
        self.fp = None
        asyncio.set_event_loop(loop)
        self.loop = asyncio.get_event_loop()
        print("Pynet running")
        # Create a task to handle incoming requests then wait for
        # all tasks to complete.
        self.loop.set_exception_handler(self.exception_handler)

        # Start the request handler
        self.loop.create_task(self.request_handler())

        pending = asyncio.Task.all_tasks()
        try:
            self.loop.run_until_complete(asyncio.gather(*pending))
        except RuntimeError:
            print("Pynet Loop stopped, disconnected")

        self.disconnect()

    # Main thread
    def start_thread(self):
        loop = asyncio.get_event_loop()
        self.t = Thread(target=self.run, args=(loop,))
        self.t.start()

    # Main thread
    def join(self):
        if self.t:
            self.t.join()
        return

    # Main thread
    def put(self, msg):
        self.requests.put(msg)

    # Main thread
    def iter_results(self):
        empty = False
        while not empty:
            try:
                item = self.replies.get(False)
                yield item
            except queue.Empty:
                empty = True

class PynetMessage(object):
    def __init__(self, req, *args, **kwargs):
        self.req = req
        self.args = args
        self.kwargs = kwargs
        self.result = None

    def set_result(self, result_dict):
        self.result = result_dict

class Pynet(object):
    def __init__(self):
        self.pynet_handler = None
        self.callbacks = {}
        self.default_callback = None

    def connect(self, connection_callback,
                ip=None, port=None,
                default_callback=None):
        self.callbacks = {}
        if not self.pynet_handler:
            self.pynet_handler = PynetHandler()
            print("starting Pynet thread")
            self.pynet_handler.start_thread()
        self.default_callback = default_callback
        self.register("connect", connection_callback)
        self.register("disconnect", connection_callback)
        self.send_async("connect", ip=ip, port=port)

    def disconnect(self):
        if self.pynet_handler:
            self.send_async("disconnect")
            self.pynet_handler.join()
            # Drain the queue of replies before fully disconnecting.
            self.recv_async()
            self.pynet_handler = None
            self.default_callback = None
            self.callbacks = {}


    def register(self, req, callback):
        self.callbacks[req] = callback

    def recv_async(self):
        if self.pynet_handler:
            for res in self.pynet_handler.iter_results():
                if res.req in self.callbacks:
                    self.callbacks[res.req](res.req, res.result)
                elif self.default_callback:
                    self.default_callback(res.req, res.result)
                else:
                    print("Ignoring response: {}: {}".format(res.req, res.result))

    def send_async(self, req, *args, **kwargs):
        print("send async: {}".format(req))
        self.pynet_handler.put(PynetMessage(req, *args, **kwargs))

if __name__ == "__main__":
    pass

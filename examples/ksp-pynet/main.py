#!/usr/bin/python3
import sys
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.stacklayout import StackLayout
from kivy.uix.scrollview import ScrollView
from kivy.properties import ObjectProperty, StringProperty, NumericProperty
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Ellipse, Color
import re
import math
import ksppynet
import traceback
from functools import partial



class MenuActive(ScrollView):
    pass

class DisplayManeuver(GridLayout):
    def __init__(self, **kwargs):
        super(DisplayManeuver, self).__init__(**kwargs)

        self.maneuvers = {"apoapsis" : "120000",
                          "periapsis": "100000"}


        self.dropdown = DropDown()
        self.input = FloatInput(size=(200, 30),
                                multiline=False,
                                size_hint=(None, None))

        for m in self.maneuvers.keys():
            btn = Button(text=m, size_hint_y=None, height=30)

            # Pass the text of the button as the data of the selection.
            btn.bind(on_release=lambda btn: self.dropdown.select(btn.text))
            self.dropdown.add_widget(btn)

        self.mainbutton = Button(text='Maneuver',
                                 size=(160, 50),
                                 size_hint=(None, None))

        self.mainbutton.bind(on_release=self.dropdown.open)

        def dropdown_action(instance, m):
            self.input.text = self.maneuvers[m]
            self.mainbutton.text = m

        self.dropdown.bind(on_select=dropdown_action)

        self.go_button = Button(text='Go',
                                size=(160, 50),
                                size_hint=(None, None))
        self.go_button.bind(on_press=self.go)

        self.add_widget(self.mainbutton)
        self.add_widget(self.input)
        self.add_widget(self.go_button)

    def go(self, *args):
        kwargs = {self.mainbutton.text : float(self.input.text)}
        app = App.get_running_app()
        app.pynet.send_async("maneuver", **kwargs)

class Viewer(Widget):
    body_name = StringProperty()

    def __init__(self, **kwargs):
        super(Viewer, self).__init__(**kwargs)
        with self.canvas:
            Color(0.0, 0.1, 0.9, 0.8)
            self.orbit = Ellipse()
            Color(0.1, 0.1, 0.2, 1)
            self.orbit_inner = Ellipse()
            Color(0.2, 0.3, 0.1, 1)
            self.body = Ellipse()
        self.bind(pos=self.update, size=self.update)
        self.real_apoapsis = 1000000
        self.real_periapsis = 400000
        self.real_radius = 50000
        self.rescale()

    def rescale(self):
        # Put the body in the middle, with the apoapsis near the edge of the
        # screen.
        try:
            factor1 = self.width / (float(max(self.real_apoapsis, self.real_radius)) * 2)
            factor2 = self.height / (float(max(math.sqrt(self.real_apoapsis *
                                                         self.real_periapsis),
                                               self.real_radius)) * 2)
            factor = min(factor1, factor2)
            factor *= 0.9
            self.radius = self.real_radius * factor
            self.apoapsis = self.real_apoapsis * factor
            self.periapsis = self.real_periapsis * factor
        except:
            # if we cant' rescale, don't
            print(traceback.format_exc())


    def update(self, *args):
        self.rescale()
        self.body.pos = self.body_pos()
        self.body.size = self.body_size()
        self.orbit.pos = self.orbit_pos()
        self.orbit.size = self.orbit_size()
        self.orbit_inner.pos = (self.orbit.pos[0] + 2, self.orbit.pos[1] + 2)
        self.orbit_inner.size = (self.orbit.size[0] - 4, self.orbit.size[1] - 4)

    @property
    def center(self):
        return(self.width/2.0, self.height/2.0)

    @property
    def _x(self):
        return math.sqrt((self.apoapsis * self.periapsis))

    def body_pos(self):
        return (self.center[0] - self.radius, self.center[1] - self.radius)

    def body_size(self):
        return (2*self.radius, 2*self.radius)

    def orbit_pos(self):
        return (self.center[0] - self.periapsis, self.center[1] - self._x)

    def orbit_size(self):
        return (self.apoapsis + self.periapsis, 2 * self._x)

    def update_params(self, msgid, odict):
        self.real_apoapsis = odict["real_apoapsis"]
        self.real_periapsis = odict["real_periapsis"]
        self.real_radius = odict["body_radius"]
        self.body_name = odict["body_name"]
        self.update()

    def register(self, app, on):
        app.pynet.register("orbital_notify", self.update_params)
        app.pynet.send_async("orbital", on)


class Menu(BoxLayout):
    connection = ObjectProperty()
    dynamic = ObjectProperty()
    alerts = ObjectProperty()

    def __init__(self, **kwargs):
        super(Menu, self).__init__(**kwargs)
        self.active_widget = None
        self.message_buffer = [{}]
        Clock.schedule_interval(self.update_msg, 1.0)

    def active(self, enable):
        if enable and not self.active_widget:
            self.active_widget = MenuActive()
            self.connection.background_color = (0.5,1,0.5,0.8)
            self.dynamic.add_widget(self.active_widget)
        elif not enable and self.active_widget:
            self.connection.background_color = (1,0.1,0.1,1)
            self.dynamic.remove_widget(self.active_widget)
            self.active_widget = None

    def update_msg(self, dt):
        try:
            self.message_buffer.pop(0)
            out = "\n".join([m for m in self.message_buffer if m]).strip()
            self.alerts.text = out
        except:
            self.alerts.text = ""

    def add_msg(self, msg, duration=5):
        extra = duration - len(self.message_buffer) + 1
        if extra > 0:
            self.message_buffer = self.message_buffer + [""] * extra
        self.message_buffer[duration] = "\n".join([self.message_buffer[duration], msg]).strip()

class FloatInput(TextInput):
    pat = re.compile('[^0-9]')
    def insert_text(self, substring, from_undo=False):
        pat = self.pat
        if '.' in self.text:
            s = re.sub(pat, '', substring)
        else:
            s = '.'.join([re.sub(pat, '', s) for s in substring.split('.', 1)])
        return super(FloatInput, self).insert_text(s, from_undo=from_undo)

class DisplayConnection(GridLayout):
    connected = ObjectProperty()
    connected_state = ObjectProperty()
    def go(self, app):
        info = { k: self.ids[k] for k in ("ip", "port")}
        if self.connected_state == str(True):
            app.disconnect()
        else:
            app.connect(**info)

    def set_connected(self, on):
        self.connected_state = str(on)
        if on:
            self.connected.text = "Disconnect"
            self.connected.background_color = (1,0.1,0.1,1)
        else:
            self.connected.text = "Connect"
            self.connected.background_color = (0.5,1,0.5,0.8)

class DisplayFlightPlan(GridLayout):
    def go(self, app):
        info = { k: float(self.ids[k].state) for k in ("heading", "target_altitude",
                                                       "apoapsis", "periapsis")}
        app.pynet.send_async("plan", **info)

class DisplayState(BoxLayout):
    pass

class DisplayDiagnostics(BoxLayout):
    pass

class Display(RelativeLayout):
    sm = ObjectProperty()

class Console(BoxLayout):
    display = ObjectProperty()
    menu = ObjectProperty()


class KpConsoleApp(App):
    def connection_handler(self, msgid, msg_dict):
        if msgid == "connect" and "error" not in msg_dict:
            self.console.display.dconn.set_connected(True)
            self.console.menu.active(True)
        else:
            self.console.display.dconn.set_connected(False)
            self.console.menu.active(False)
        print("{}: {}".format(msgid, msg_dict))

    @property
    def screen(self):
        return self.console.display.sm

    def default_recv_msg_handler(self, msgid, msg_dict):
        if msgid == "message" and "msg" in msg_dict:
            self.console.menu.add_msg(**msg_dict)
        else:
            print("Unhandled response: {}: {}".format(msgid, msg_dict))

    def pynet_response_handler(self, dt):
        self.pynet.recv_async()

    def disconnect(self):
        self.pynet.disconnect()

    def connect(self, **kwargs):
        self.pynet.connect(self.connection_handler,
                           default_callback=self.default_recv_msg_handler,
                           **kwargs)
        self.console.menu.add_msg("Connecting...")
        Clock.schedule_interval(self.pynet_response_handler, 1.0 / 60.0)

    def build(self):
        self.pynet = ksppynet.Pynet()
        self.console = Console()
        return self.console

if __name__ == '__main__':
    if len(sys.argv) > 1:
        #Turn on Test/debug mode.
        ksppynet.ksppynet.testing_ui_on = True
    KpConsoleApp().run()

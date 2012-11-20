from __future__ import division

from gi.repository import Gtk, Gdk
import math

from . import renderers, comm

def all_gains():
    for axis in ['roll', 'pitch', 'yaw']:
        for t in ['p', 'd']:
            yield axis + '_' + t

class BaseStation(object):
    def __init__(self):
        self.orientation = renderers.OrientationRenderer()
        self.graphtop = renderers.GraphRenderer(3, 50)
        self.graphbottom = renderers.GraphRenderer(3, 50)

        self.builder = Gtk.Builder()
        self.builder.add_from_file("main.glade")

        self.main_window = self.builder.get_object('main_window')
        self.main_window.connect('delete-event', Gtk.main_quit)
        self.main_window.show_all()

        self.gains_dialog = self.builder.get_object('gains_dialog')
        def callback(arg, arg2):
            self.gains_dialog.hide()
            return True
        self.gains_dialog.connect('delete-event', callback)

        self.builder.get_object('orientation_draw').connect('draw', self.orientation.draw_callback)
        self.builder.get_object('graphtop_draw').connect('draw', self.graphtop.draw_callback)
        self.builder.get_object('graphbottom_draw').connect('draw', self.graphbottom.draw_callback)

        self.builder.get_object('gains_button').connect('clicked', self.gains_callback)
        self.builder.get_object('ok_button').connect('clicked', self.gains_ok_callback)
        self.builder.get_object('cancel_button').connect('clicked', lambda ev: self.gains_dialog.hide())

        self.quadstate = comm.QuadState('/dev/ttyXBee', self.quad_callback)

    def quad_callback(self):
        self.orientation.set_roll_pitch(self.quadstate['roll'], self.quadstate['pitch'])

        Gdk.threads_enter()
        self.main_window.queue_draw()
        Gdk.threads_leave()

    def gains_callback(self, event):
        for gain in all_gains():
            spin = self.builder.get_object('gain_' + gain + '_spin')
            spin.set_value(self.quadstate['gain_' + gain]*1000)
        self.gains_dialog.show_all()

    def gains_ok_callback(self, event):
        gains = {}
        for gain in all_gains():
            spin = self.builder.get_object('gain_' + gain + '_spin')
            gains[gain] = spin.get_value_as_int()/1000
        self.quadstate.send_gains(gains)
        self.gains_dialog.hide()

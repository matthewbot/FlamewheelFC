from __future__ import division

from gi.repository import Gtk, Gdk
import math
import serial

from . import renderers, comm

def all_gains():
    for axis in ['roll', 'pitch', 'yaw']:
        for t in ['p', 'd']:
            yield axis + '_' + t

all_escs = ['fl', 'fr', 'rr', 'rl']

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
        self.builder.get_object('plot_combo').connect('changed', self.plot_callback)

        self.builder.get_object('ok_button').connect('clicked', self.gains_ok_callback)
        self.builder.get_object('cancel_button').connect('clicked', lambda ev: self.gains_dialog.hide())

        self.current_plot = 'attitude'
        try:
            self.quadstate = comm.QuadState('/dev/ttyXBee', self.quad_callback)
        except serial.SerialException:
            self.quadstate = None

    def quad_callback(self):
        self.orientation.set_roll_pitch(self.quadstate['roll'], self.quadstate['pitch'])

        if self.current_plot == 'attitude':
            self.graphtop.add_sample([self.quadstate['roll'], self.quadstate['pitch'], self.quadstate['yaw']])
            self.graphbottom.add_sample([self.quadstate['roll_rate'], self.quadstate['pitch_rate'], self.quadstate['yaw_rate']])
        elif self.current_plot == 'control':
            self.graphtop.add_sample([self.quadstate['roll_p'], self.quadstate['pitch_p'], self.quadstate['yaw_p']])
            self.graphbottom.add_sample([self.quadstate['roll_d'], self.quadstate['pitch_d'], self.quadstate['yaw_d']])
        elif self.current_plot == 'altitude':
            self.graphtop.add_sample([self.quadstate['altitude'], 0, 0])
            self.graphbottom.add_sample([self.quadstate['altitude_rate'], 0, 0])

        Gdk.threads_enter()
        for esc in all_escs:
            self.builder.get_object(esc+'_bar').set_fraction(self.quadstate['esc_'+esc])

        powerfrac = (self.quadstate['battery'] - 10) / 2.6
        powerfrac = min(1, max(0, powerfrac))
        self.builder.get_object('power_bar').set_fraction(powerfrac)
        self.main_window.queue_draw()
        Gdk.threads_leave()

    def plot_callback(self, combo):
        tree_iter = combo.get_active_iter()
        if tree_iter != None:
            model = combo.get_model()
            self.current_plot = model[tree_iter][0]
        else:
            self.current_plot = None

        self.graphtop.clear_samples()
        self.graphbottom.clear_samples()
        print self.current_plot

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

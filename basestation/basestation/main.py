from __future__ import division

from gi.repository import Gtk, Gdk
import math

from . import renderers, comm

class BaseStation(object):
    def __init__(self):
        self.orientation = renderers.OrientationRenderer()
        self.graphtop = renderers.GraphRenderer(3, 50)
        self.graphbottom = renderers.GraphRenderer(3, 50)

        builder = Gtk.Builder()
        builder.add_from_file("main.glade")

        self.main_window = builder.get_object('main_window')
        self.main_window.connect('delete-event', Gtk.main_quit)
        self.main_window.show_all()

        builder.get_object('orientation_draw').connect('draw', self.orientation.draw_callback)
        builder.get_object('graphtop_draw').connect('draw', self.graphtop.draw_callback)
        builder.get_object('graphbottom_draw').connect('draw', self.graphbottom.draw_callback)

        self.quadstate = comm.QuadState('/dev/ttyXBee', self.quad_callback)

    def quad_callback(self):
        self.orientation.set_roll_pitch(self.quadstate['roll'], self.quadstate['pitch'])

        Gdk.threads_enter()
        self.main_window.queue_draw()
        Gdk.threads_leave()



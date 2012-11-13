from __future__ import division
import math

from gi.repository import Gtk
import cairo

class OrientationRenderer(object):
    def __init__(self):
        self.roll = 10/180*math.pi
        self.pitch = 20/180*math.pi

    def set_roll_pitch(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch

    def draw_callback(self, widget, cr):
        width = widget.get_allocated_width()
        height = widget.get_allocated_height()
        size = max(width, height)
        cr.translate(width/2, height/2)
        cr.scale(size/2, size/2)
        cr.rotate(self.roll)
        cr.translate(0, self.pitch_to_offset(self.pitch))

        self.draw_horizon(cr)
        self.draw_pitch_markers(cr)

        cr.identity_matrix()
        cr.translate(width/2, height/2)
        cr.scale(size/2, size/2)

        self.draw_airplane(cr)

    def draw_horizon(self, cr):
        ground = cairo.LinearGradient(0, 0, 0, 2)
        ground.add_color_stop_rgb(0, .5, .3, .1)
        ground.add_color_stop_rgb(1, .1, .05, 0)
        cr.set_source(ground)

        cr.rectangle(-2, 0, 4, 2)
        cr.fill()

        sky = cairo.LinearGradient(0, -2, 0, 0)
        sky.add_color_stop_rgb(0, .8, .8, 1)
        sky.add_color_stop_rgb(1, .4, .4, 1)
        cr.set_source(sky)

        cr.rectangle(-2, -2, 4, 2)
        cr.fill()

    def draw_pitch_markers(self, cr):
        cr.set_source_rgb(1, 1, 1)
        cr.set_line_width(.01)

        for i in xrange(-72, 72):
            angle = i*5
            if angle == 0:
                linewidth=2
            elif angle%10 == 0:
                linewidth=.10+abs(angle)/90
            else:
                linewidth=.05
            anglepos = self.pitch_to_offset(angle)
            cr.move_to(-linewidth, anglepos)
            cr.line_to(linewidth, anglepos)
            cr.stroke()

            if angle%10 == 0 and angle != 0:
                anglestr = str(int(abs(angle)))
                x_bearing, y_bearing, width, height = cr.text_extents(anglestr)[:4]
                cr.set_font_size(.1)
                cr.move_to(-linewidth-.05-width-x_bearing, anglepos+height/2)
                cr.text_path(anglestr)
                cr.move_to(linewidth+.05-x_bearing, anglepos+height/2)
                cr.text_path(anglestr)
                cr.fill()

    def draw_airplane(self, cr):
        cr.set_source_rgb(1, 1, 0)
        cr.set_line_width(.02)

        cr.move_to(-.4, 0)
        cr.line_to(-.05, 0)
        cr.line_to(-.05, .05)
        cr.move_to(.4, 0)
        cr.line_to(.05, 0)
        cr.line_to(.05, .05)
        cr.stroke()

    def pitch_to_offset(self, pitch):
        return pitch/60

class GraphRenderer(object):
    def __init__(self, signalcount, maxsamples):
        self.samples = []
        self.signalcount = signalcount
        self.maxsamples = maxsamples

    def add_sample(self, sample):
        assert(len(sample) == self.signalcount)
        self.samples.append(sample)
        if len(self.samples) > self.maxsamples:
            del self.samples[0:len(self.samples)-self.maxsamples]

    def min_max_values(self):
        minval = self.samples[0][0]
        maxval = self.samples[0][0]
        for sample in self.samples:
            for val in sample:
                if val < minval:
                    minval = val
                if val > maxval:
                    maxval = val

        return (minval, maxval)

    def draw_callback(self, widget, cr):
        width = widget.get_allocated_width()
        height = widget.get_allocated_height()
        (minval, maxval) = self.min_max_values()
        cr.translate(width/2, height/2)
        cr.scale(width/self.maxsamples*.95, -height/(maxval-minval)*.95)
        cr.translate(-self.maxsamples/2, minval + (maxval-minval)/2)

        cr.set_line_width(.01)
        cr.set_source_rgb(0, 0, 0)
        for signal in xrange(self.signalcount):
            cr.move_to(0, self.samples[0][signal])
            for x, val in enumerate((sample[signal] for sample in self.samples[1:]), 1):
                cr.line_to(x, val)
        cr.stroke()


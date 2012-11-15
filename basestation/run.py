from gi.repository import Gtk, Gdk, GLib
from basestation import main, comm

Gdk.threads_init()

basestation = main.BaseStation()
GLib.threads_init()
Gdk.threads_init()
Gdk.threads_enter()
Gtk.main()
Gdk.threads_leave()

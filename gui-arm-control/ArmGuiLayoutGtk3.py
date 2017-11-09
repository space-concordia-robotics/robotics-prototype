# This uses Gtk 3, so make sure that is installed before proceeding.
import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk

'''
This will access and run the glade file
If your glade file is not in the same directory as this python file,
then you must specify the directory in the
builder.add_from_file("xxx"), where xxx is your directory.

The glade file is currently using textboxes as placeholders
for the windows that will be implemented in the future.
'''

builder = Gtk.Builder()
builder.add_from_file("ArmGuiLayout.glade")

window = builder.get_object("ArmGuiLayoutWindow")

# CSS Styling part
screen = Gdk.Screen.get_default()
css_provider = Gtk.CssProvider()
css_provider.load_from_path('Styling.css')
context = Gtk.StyleContext()
context.add_provider_for_screen(screen, css_provider,
  Gtk.STYLE_PROVIDER_PRIORITY_USER)


window.show_all()

Gtk.main()


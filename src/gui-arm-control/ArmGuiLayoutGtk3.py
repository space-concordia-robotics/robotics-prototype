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

# CSS Styling part to make GUI look better
screen = Gdk.Screen.get_default()
css_provider = Gtk.CssProvider()
css_provider.load_from_path('Styling.css')
context = Gtk.StyleContext()
context.add_provider_for_screen(screen, css_provider,
Gtk.STYLE_PROVIDER_PRIORITY_USER)
# End of CSS

# Test button for error log. Will remove once error log can
# read errors from other windows
def testButton(button):
    textbuffer = textarea.get_buffer()
    textbuffer.set_text(textinput.read())

#def clearButton(button):
    #textbuffer = textarea.get_buffer()
    

builder = Gtk.Builder()
builder.add_from_file("ArmGuiLayout.glade")

# This is used to connect signals sent by the widgets in Glade
# and assign them functionality using python code
handlers = {
    "onTestClick": testButton,
    #"onClearClick": clearButton
}
builder.connect_signals(handlers)

textarea = builder.get_object("Error Log ")
textinput = open("ErrorLogTest.txt", "r")

window = builder.get_object("ArmGuiLayoutWindow")
window.connect("destroy", Gtk.main_quit)
window.show_all()

# Need to close file, but program won't run if I include it
# textinput.close()
Gtk.main()

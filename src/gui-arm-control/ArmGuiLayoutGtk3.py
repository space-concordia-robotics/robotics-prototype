# This uses Gtk 3, so make sure that is installed before proceeding.
import gi
import datetime

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
    
# // definitions of the handlers for the buttons
def yawLeftButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Yaw Left Clicked " + now.strftime("%Y-%m-%d %H:%M") + "\n")

def picthUpButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Pitch Up Clicked " + str(now) + "\n")

def yawRightButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Yaw Right Clicked " + str(now) + "\n")

def rollLeftButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Roll Left Clicked " + str(now) + "\n")

def pitchDownButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Pitch Down Clicked " + str(now) + "\n")

def rollRightButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Roll Right Clicked " + str(now) + "\n")

def clawOpenButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Claw Open Clicked " + str(now) + "\n")

def clawCloseButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Claw Close Clicked " + str(now) + "\n")

def armUpButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Arm Up Clicked " + str(now) + "\n")

def armLeftButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Arm Left Clicked " + str(now) + "\n")

def armDownButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Arm Down Clicked " + str(now) + "\n")

def armRightButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Arm Right Clicked " + str(now) + "\n")

def armBackButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Arm Back Clicked " + str(now) + "\n")

def armFwdButtonClick(button):
    now = datetime.datetime.now()
    text_buffer = textarea.get_buffer()
    end_iter = text_buffer.get_end_iter()
    text_buffer.insert(end_iter, "> Arm Fwd Clicked " + str(now) + "\n")

builder = Gtk.Builder()
builder.add_from_file("ArmGuiLayout2.glade")

# This is used to connect signals sent by the widgets in Glade
# and assign them functionality using python code
handlers = {
    "onTestClick": testButton,
    #"onClearClick": clearButton
    "onYawLeftClicked": yawLeftButtonClick,
    "onPitchUpClicked": picthUpButtonClick,
    "onYawRightClicked": yawRightButtonClick,
    "onRollLeftClicked": rollLeftButtonClick,
    "onPitchDownClicked": pitchDownButtonClick,
    "onRollRightClicked": rollRightButtonClick,
    "onClawOpenClicked": clawOpenButtonClick,
    "onClawCloseClicked": clawCloseButtonClick,
    "onArmUpClicked": armUpButtonClick,
    "onArmLeftClicked": armLeftButtonClick,
    "onArmDownClicked": armDownButtonClick,
    "onArmRightClicked": armRightButtonClick,
    "onArmBackClicked": armBackButtonClick,
    "onArmFwdClicked": armFwdButtonClick
}
builder.connect_signals(handlers)

textarea = builder.get_object("Error Log ")
textinput = open("ErrorLogTest.txt", "r")

window = builder.get_object("ArmGuiLayoutWindow")
window.set_title("Space Concordia Robtotics GUI")
window.connect("destroy", Gtk.main_quit)
window.show_all()

# Need to close file, but program won't run if I include it
# textinput.close()
Gtk.main()

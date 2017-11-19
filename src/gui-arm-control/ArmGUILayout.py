import gtk
import sys


'''
This is the class for the gui layout of the robot arm.
Currently, the textView() boxes are used as place holders
for the actual features that will be implemented later.
'''


class ArmGUILayout(gtk.Window):
    def __init__(self):
        super(ArmGUILayout, self).__init__()

        self.set_title("ROBOTICS GUI LAYOUT")
        self.set_size_request(1250, 600)
        self.set_border_width(8)
        self.set_position(gtk.WIN_POS_CENTER)

        table = gtk.Table(32, 16, False)
        table.set_col_spacings(3)

        # ARM Position, Lidar, and Logs Window Section:

        # ARM Position Window
        title = gtk.Label("ARM Position")

        halign = gtk.Alignment(0, 0, 0, 0)
        halign.add(title)

        table.attach(halign, 0, 1, 0, 1, 0, 0)

        wins = gtk.TextView()
        wins.set_editable(False)
        wins.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(5140, 5140, 5140))
        wins.set_cursor_visible(False)
        table.attach(wins, 0, 3, 1, 4)

        # LIDAR Window
        title2 = gtk.Label("LIDAR")

        halign2 = gtk.Alignment(0, 0, 0, 0)
        halign2.add(title2)

        table.attach(halign2, 0, 10, 0, 1, 0,
                     0, 0, 0)
        wins2 = gtk.TextView()
        wins2.set_editable(False)
        wins2.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(5140, 5140, 5140))
        wins2.set_cursor_visible(False)
        table.attach(wins2, 3, 8, 1, 6)

        # LOGS
        title3 = gtk.Label("LOGS")

        halign3 = gtk.Alignment(1, 0.78, 0, 0)
        halign3.add(title3)

        table.attach(halign3, 0, 2, 1, 6, gtk.FILL,
                     gtk.FILL, 0, 0)
        wins3 = gtk.TextView()
        wins3.set_editable(False)
        wins3.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(5140, 5140, 5140))
        wins3.set_cursor_visible(False)
        table.attach(wins3, 1, 3, 5, 6, gtk.FILL,
                     gtk.FILL, 0, 0)

        # End of ARM Position, Lidar, and Logs Window Section

        # Arm and Claw Button Section:

        # Arm UP Button
        halign5 = gtk.Alignment(0.81, 0.885, 0, 0)
        arm_up = gtk.Button("Arm\n"
                        "  Up")
        arm_up.set_size_request(40, 40)
        halign5.add(arm_up)
        table.set_row_spacing(3, 6)
        table.attach(halign5, 0, 1, 1, 5, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Arm DOWN Button
        halign6 = gtk.Alignment(0.81, 0.753, 0, 0)
        arm_down = gtk.Button("Arm\n"
                          "Down")
        arm_down.set_size_request(40, 40)
        halign6.add(arm_down)
        table.set_row_spacing(3, 6)
        table.attach(halign6, 0, 1, 1, 6, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Arm LEFT Button
        halign7 = gtk.Alignment(0.62, 0.748, 0, 0)
        arm_left = gtk.Button("Arm\n"
                          "Left")
        arm_left.set_size_request(40, 40)
        halign7.add(arm_left)
        table.set_row_spacing(3, 6)
        table.attach(halign7, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Arm Right Button
        halign7 = gtk.Alignment(0.99, 0.748, 0, 0)
        arm_right = gtk.Button("Arm\n"
                          "Right")
        arm_right.set_size_request(40, 40)
        halign7.add(arm_right)
        table.set_row_spacing(3, 6)
        table.attach(halign7, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Claw Out Button
        halign8 = gtk.Alignment(0.195, 0.675, 0, 0)
        claw_out = gtk.Button("Claw\n"
                          " Out")
        claw_out.set_size_request(40, 40)
        halign8.add(claw_out)
        table.set_row_spacing(3, 6)
        table.attach(halign8, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Claw In Button
        halign9 = gtk.Alignment(0.195, 0.748, 0, 0)
        claw_in = gtk.Button("Claw\n"
                          "  In")
        claw_in.set_size_request(40, 40)
        halign9.add(claw_in)
        table.set_row_spacing(3, 6)
        table.attach(halign9, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Claw Left Button
        halign10 = gtk.Alignment(0, 0.748, 0, 0)
        claw_left = gtk.Button("Claw\n"
                          "Left")
        claw_left.set_size_request(40, 40)
        halign10.add(claw_left)
        table.set_row_spacing(3, 6)
        table.attach(halign10, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Claw Right Button
        halign11 = gtk.Alignment(0.39, 0.748, 0, 0)
        claw_right = gtk.Button("Claw\n"
                          "Right")
        claw_right.set_size_request(40, 40)
        halign11.add(claw_right)
        table.set_row_spacing(3, 6)
        table.attach(halign11, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Claw Open Button
        halign12 = gtk.Alignment(0.39, 0.817, 0, 0)
        claw_open = gtk.Button("Claw\n"
                                "Open")
        claw_open.set_size_request(40, 40)
        halign12.add(claw_open)
        table.set_row_spacing(3, 6)
        table.attach(halign12, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # Claw Close Button
        halign13 = gtk.Alignment(0, 0.817, 0, 0)
        claw_close = gtk.Button("Claw\n"
                               "Left")
        claw_close.set_size_request(40, 40)
        halign13.add(claw_close)
        table.set_row_spacing(3, 6)
        table.attach(halign13, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        # End of Button Section

        # Battery Percentage
        halign14 = gtk.Alignment(0, 0.98, 0, 0)
        carbon_percentage = gtk.ProgressBar(adjustment=None)
        carbon_percentage.set_size_request(100, 30)
        carbon_percentage.set_fraction(0.25)
        carbon_percentage.set_text("Battery%")
        halign14.add(carbon_percentage)
        table.set_row_spacing(3, 6)
        table.attach(halign14, 0, 1, 1, 7, gtk.FILL,
                     gtk.FILL, 0, 0)

        self.add(table)

        self.connect("destroy", gtk.main_quit)
        self.show_all()


ArmGUILayout()
gtk.main()

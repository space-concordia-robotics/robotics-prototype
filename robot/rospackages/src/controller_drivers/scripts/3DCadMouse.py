devpath = "/dev/hidraw2"

axis = [0, 0, 0, 0, 0, 0]
button = [False, False]
battery = -1
charged = False

with open(devpath, "rb", buffering=0) as MouseByteStream:
  while True:
    data = MouseByteStream.read(16)
    id = data[0]
    if id == 1:
      for i in range(0, 6):
        axis_raw = data[2 * i + 1] + 256 * data[2 * i + 2]
        if axis_raw > 32768:
          axis_raw = axis_raw - 65536
        axis[i] = axis_raw
      print(axis)
    elif id == 3:
      button_raw = data[1]
      button[0] = (button_raw & 0x1) != 0
      button[1] = (button_raw & 0x2) != 0
      print("Button: " + str(button))
    elif id == 23:
      battery = data[1]
      charged = data[2] != 0
      print("Battery:" + str(battery) + ", Charging: " + str(charged))


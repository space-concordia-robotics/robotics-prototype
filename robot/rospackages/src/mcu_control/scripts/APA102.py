def constrain(val, min_val, max_val):
  return min(max_val, max(min_val, val))


class APA102:
  def __init__(self, spi, num_of_leds):
    self.spi = spi
    self.num_of_leds = num_of_leds
    self.buf = []
    for i in range(4):
      self.buf.append(0x00)
    for i in range(4, num_of_leds + 4):
      self.buf.append(0xE0)
      for j in range(3):
        self.buf.append(0x00)
    size_end_frame = num_of_leds // 8 # size in bytes
    if size_end_frame * 8 < (num_of_leds / 2):
      size_end_frame += 1
    for i in range(size_end_frame):
      self.buf.append(0xFF)
  def send(self):
    self.spi(self.buf)
  def set_colour(self, number, r, g, b, brightness = 31):
    self.buf[4 * number + 4] = 0xE0 + constrain(brightness, 0, 31)
    self.buf[4 * number + 5] = constrain(b, 0, 255)
    self.buf[4 * number + 6] = constrain(g, 0, 255)
    self.buf[4 * number + 7] = constrain(r, 0, 255)
  def set_all(self, r, g, b, brightness = 31):
    for i in range(self.num_of_leds):
      self.set_colour(i, r, g, b, brightness)
    # print(self.buf)
  def set_to_off(self):
    self.set_all(0, 0, 0, 0)
  def set_pins(self, pins, r, g, b, brightness = 31):
    self.set_to_off()
    for i in pins:
      self.set_colour(i, r, g, b, brightness)

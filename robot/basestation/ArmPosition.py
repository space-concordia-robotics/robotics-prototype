class ArmPosition:
    def __init__(self):
        self.motors = []

    def update(self, frequency):
        return False

    def render(self, angle_positions):
        return False

    def change_perspective(self, xpos, ypos, zpos):
        return False

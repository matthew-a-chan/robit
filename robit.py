import odrive
import time
import math






class Robit ():

    def __init__ (self):
        print ("looking for device")
        self.device = odrive.find_any ()
        print ("found device")

    def set_state (self, axis, state):
        motor = self.get_motor (axis)
        if state == 'IDLE':
            motor.requested_state = 1 # IDLE
        if state == 'HOLD':
            motor.requested_state = 8 # Lock position
        if state == 'CALIBRATE':
            motor.requested_state = 3 # CALIBRATION
        
    def snap (self, axis, position):
        motor = self.get_motor (axis)
        motor.controller.pos_setpoint = position

    #def go_to (self, axis, position):
    #    motor = self.get_motor (axis)
    #    motor.controller.move_to_pos (position)

    def get_motor (self, axis):
        if axis == 0:
            return self.device.axis0
        else:
            return self.device.axis1
        
    def test_sin (self, revolutions, amplitude):
        iterations = 10000
        for i in range (iterations):
            self.snap (0, math.sin(revolutions * 6.282 * i / iterations) * amplitude)
            time.sleep (50 / iterations)















if __name__ == '__main__':
    pass
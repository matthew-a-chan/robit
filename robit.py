import odrive
import time
import math




class Robit ():

    def __init__ (self, motors):
        self.devices = dict()
        self.motor_table = dict()
        for serial, axis, label in motors:
            print (f"looking for device {serial}")
            device = odrive.find_any (serial_number = serial)
            print ("found device")

            self.devices[serial] = device
            self.motor_table[label] = (serial, axis)
                

    def set_state (self, label, state):
        motor = self.get_motor (label)
        if state == 'IDLE':
            motor.requested_state = 1 # IDLE
        if state == 'CONTROL':
            motor.requested_state = 8 # Controlled
        if state == 'CALIBRATE':
            motor.requested_state = 3 # CALIBRATION
        
    def snap (self, label, position):
        motor = self.get_motor (label)
        motor.controller.pos_setpoint = position

    def measure_resistance (self, label):
        motor = self.get_motor (label)
        return motor.motor.current_meas_phB

    def get_motor (self, label):
        # If need be, we may easily optimize this.
        serial, axis = self.motor_table[label]
        device = self.devices[serial]
        if axis == 0:
            return device.axis0
        else:
            return device.axis1
        
    def test_sin (self, label, revolutions, amplitude):
        self.set_state (label, 'CONTROL')
        iterations = 1000
        for i in range (iterations):
            self.snap (label, math.sin(revolutions * 6.282 * i / iterations) * amplitude)
            time.sleep (5.0 / iterations)
        self.set_state (label, 'IDLE')

        














if __name__ == '__main__':
    pass
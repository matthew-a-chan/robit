import odrive
import time
import math
import tinyik



class Robit ():

    def __init__ (self, controllers):
        self.devices = dict()
        self.motor_table = dict()
        for serial, label1, label2 in controllers:
            print (f"looking for device {serial}")
            device = odrive.find_any (serial_number = serial)
            print ("found device")

            self.devices[serial] = device
            self.motor_table[label1] = (serial, 0)
            self.motor_table[label2] = (serial, 1)
        
        self.legs = dict()

        self.legs['Front_Left'] =  tinyik.Actuator(['x', [0., -20., .0], 'y', [0., 0., -116.], 'y', [0., 10., -116.]])
        self.legs['Front_Right'] = tinyik.Actuator(['x', [0., 20., .0], 'y', [0., 0., -116.], 'y', [0., -10., -116.]])
        self.legs['Back_Left'] =   tinyik.Actuator(['x', [0., -20., .0], 'y', [0., 0., -116.], 'y', [0., 10., -116.]])
        self.legs['Back_Right'] =  tinyik.Actuator(['x', [0., 20., .0], 'y', [0., 0., -116.], 'y', [0., -10., -116.]])


    def set_state (self, label, state):
        motor = self.get_motor (label)
        if state == 'IDLE':
            motor.requested_state = 1 # IDLE
        if state == 'CONTROL':
            motor.requested_state = 8 # Controlled
        if state == 'CALIBRATE':
            motor.requested_state = 3 # CALIBRATION
    
    def snap (self, labels, positions):
        SCALE = 9*7200/6.14159 # ~10551.1
        for label, position in zip (labels, positions):
            self.snap_units (label, SCALE * position)
    
    def snap_units (self, label, position):
        motor = self.get_motor (label)
        motor.controller.pos_setpoint = position

    def measure_resistance (self, label):
        motor = self.get_motor (label)
        return motor.motor.current_meas_phB

    def reset (self, labels, positions):
        for label, position in zip (labels, positions):
            self.set_state (label, 'IDLE')
            motor = self.get_motor (label)
            motor.encoder.set_linear_count(position)
            self.snap_units (label, position)
            self.set_state (label, 'CONTROL')

    
    def get_motor (self, label):
        # If need be, we may easily optimize this.
        serial, axis = self.motor_table[label]
        device = self.devices[serial]
        if axis == 0:
            return device.axis0
        else:
            return device.axis1
    
    def place (self, positions):
        print ("This is a really _really_ _REALLY_ bad idea to run this function right now.")
        return
        if 'Front_Left' in positions:
            self.legs['Front_Left'].ee = positions['Front_Left']
            self.snap (['FL_1', 'FL_2', 'FL_3'], self.legs['Front_Left'].angles)
        if 'Front_Right' in positions:
            self.legs['Front_Right'].ee = positions['Front_Right']
            self.snap (['FR_1', 'FR_2', 'FR_3'], self.legs['Front_Right'].angles)
        if 'Back_Left' in positions:
            self.legs['Back_Left'].ee = positions['Back_Left']
            self.snap (['BL_1', 'BL_2', 'BL_3'], self.legs['Back_Left'].angles)
        if 'Back_Right' in positions:
            self.legs['Back_Right'].ee = positions['Back_Right']
            self.snap (['BR_1', 'BR_2', 'BR_3'], self.legs['Back_Right'].angles)
        











        
    def test_sin (self, labels, revolutions, amplitude):
        #self.set_state (labels, 'CONTROL')
        iterations = 1000
        for i in range (iterations):
            self.snap (labels, [math.sin(revolutions * 6.282 * i / iterations) * amplitude] * len(labels))
            time.sleep (5.0 / iterations)
        #self.set_state (label, 'IDLE')

        











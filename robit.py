import odrive
import time
import math
import ik


class Robit():

    def __init__(self, controllers, enable_diagonistic=False):
        self.devices = dict()
        self.motor_table = dict()
        for serial, label1, label2 in controllers:
            print(f"looking for device {serial}")
            device = odrive.find_any(serial_number=serial)
            print("found device")

            self.devices[serial] = device
            self.motor_table[label1] = (serial, 0)
            self.motor_table[label2] = (serial, 1)

        self.enable_diagonistic = enable_diagonistic

    def debug(self, *args):
        if self.enable_diagonistic:
            print(args)

    def set_state(self, label, state):
        motor = self.get_motor(label)
        if state == 'IDLE':
            motor.requested_state = 1  # IDLE
        if state == 'CONTROL':
            motor.requested_state = 8  # Controlled
        if state == 'CALIBRATE':
            motor.requested_state = 3  # CALIBRATION

    def calibrate(self):
        pass

    def snap(self, labels, positions):
        SCALE = 9 * 7200 / 6.14159  # ~10551.1
        for label, position in zip(labels, positions):
            self.snap_units(label, SCALE * position)

    def snap_units(self, label, position):
        motor = self.get_motor(label)
        motor.controller.pos_setpoint = position

    def measure_resistance(self, label):
        motor = self.get_motor(label)
        return motor.motor.current_meas_phB

    def reset(self, labels, positions):
        for label, position in zip(labels, positions):
            self.set_state(label, 'IDLE')
            motor = self.get_motor(label)
            motor.encoder.set_linear_count(position)
            self.snap_units(label, position)
            self.set_state(label, 'CONTROL')

    def get_motor(self, label):
        if not label in self.motor_table:
            self.debug('Invalid motor! requested label: ', label)
            return None
        # If need be, we may easily optimize this.
        serial, axis = self.motor_table[label]
        device = self.devices[serial]
        if axis == 0:
            return device.axis0
        else:
            return device.axis1

    def place(self, positions):
        print("This is a really _really_ _REALLY_ bad idea to run this function right now.")
        return

        # Remember, ik.solve returns [ Hip Abductor, Hip Angle, Knee Angle ]
        if 'Front_Left' in positions:
            angles = ik.solve(positions['Front_Left'], 'Front_Left')
            self.snap(['FL_1', 'FL_2', 'FL_3'], angles)
        if 'Front_Right' in positions:
            angles = ik.solve(positions['Front_Right'], 'Front_Right')
            self.snap(['FR_1', 'FR_2', 'FR_3'], self.legs['Front_Right'].angles)
        if 'Back_Left' in positions:
            angles = ik.solve(positions['Back_Left'], 'Back_Left')
            self.snap(['BL_1', 'BL_2', 'BL_3'], self.legs['Back_Left'].angles)
        if 'Back_Right' in positions:
            angles = ik.solve(positions['Back_Right'], 'Back_Right')
            self.snap(['BR_1', 'BR_2', 'BR_3'], self.legs['Back_Right'].angles)

    def autoconfig(self):

        for serial in self.devices:

            device = self.devices[serial]

            device.config.brake_resistance = 0.1

        for label in self.motor_table:

            motor = self.get_motor(label)

            motor.motor.config.current_lim = 20
            motor.motor.config.calibration_current = 5
            motor.motor.config.pole_pairs = 12
            motor.motor.config.motor_type = 0

            motor.encoder.config.cpr = 8192

            motor.controller.config.vel_limit = 100000
            motor.controller.config.control_mode = 3
            motor.controller.config.pos_gain = 50
            motor.controller.config.vel_gain = 6 / 100000
            motor.controller.config.vel_integrator_gain = 0
            motor.controller.config.vel_limit_tolerance = 1.2
            motor.controller.config.vel_ramp_rate = 10000

    def reboot(self):
        for device in self.devices.values():
            try:
                print('SAVING DEVICE:', device)
                device.save_configuration()
                device.reboot()
            except Exception as e:
                self.debug(str(e))

    def dump_faults(self, clear=True):
        for serial in self.devices:
            print(serial)
            odrive.utils.dump_errors(self.devices[serial], clear)

import odrive
import time
import math
import ik


def detect(count):
    while count > 0:
        device = odrive.find_any()
        print('Found device!')
        print(device)
        count -= 1


class Robit():

    def __init__(self, enable_diagnostic=False):
        self.devices = dict()
        self.motor_table = dict()

        self.enable_diagnostic = enable_diagnostic

    def add_controller(self, controller):
        serial, label1, label2 = controller

        if serial in self.devices:
            print("This controller already exists! please delete it first.")

        print(f"looking for device {serial}")
        device = odrive.find_any(serial_number=serial)
        print("found device")

        self.devices[serial] = device
        self.motor_table[label1] = (serial, 0)
        self.motor_table[label2] = (serial, 1)

    def delete_controller(self, serial):
        self.devices.pop(serial)

    def debug(self, *args):
        if self.enable_diagnostic:
            print(args)

    def set_state(self, label, state):
        motor = self.get_motor(label)
        if state == 'IDLE':
            motor.requested_state = 1  # IDLE
        if state == 'CONTROL':
            motor.requested_state = 8  # Controlled
        if state == 'CALIBRATE':
            motor.requested_state = 3  # CALIBRATION

    def set_states(self, labels, states):
        for label, state in zip(labels, states):
            self.set_state(label, state)

    def calibrate(self):
        # Establish limits from resistance and position
        pass

    def snap(self, labels, positions):
        SCALE = 9 * 7200 / 6.14159  # ~10551.1
        for label, position in zip(labels, positions):
            self.snap_units(label, SCALE * position)

    def snap_units(self, label, position):
        motor = self.get_motor(label)
        #        motor.controller.pos_setpoint = position
        motor.controller.move_to_pos(int(position))

    def measure_resistance(self, label):
        motor = self.get_motor(label)
        return round(motor.motor.current_control.Iq_measured, 4)

    def measure_resistances(self, labels):
        resistances = dict()
        for label in labels:
            resistances[label] = self.measure_resistance(label)
        return resistances

    def reset(self, labels, positions, set_states=False):
        for label, position in zip(labels, positions):
            self.set_state(label, 'IDLE')
            motor = self.get_motor(label)
            motor.encoder.set_linear_count(position)
            self.snap_units(label, position)
            if set_states:
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

        # Remember, ik.solve returns [ Hip, Upper Leg, Lower Leg ]
        if 'Front_Left' in positions:
            angles = ik.solve(positions['Front_Left'], 'Front_Left')
            self.snap(['Front_Left_Hip', 'Front_Left_Upper', 'Front_Left_Lower'], angles)
        if 'Front_Right' in positions:
            angles = ik.solve(positions['Front_Right'], 'Front_Right')
            self.snap(['Front_Right_Hip', 'Front_Right_Upper', 'Front_Right_Lower'], angles)
        if 'Back_Left' in positions:
            angles = ik.solve(positions['Back_Left'], 'Back_Left')
            self.snap(['Back_Left_Hip', 'Back_Left_Upper', 'Back_Left_Lower'], angles)
        if 'Back_Right' in positions:
            angles = ik.solve(positions['Back_Right'], 'Back_Right')
            self.snap(['Back_Right_Hip', 'Back_Right_Upper', 'Back_Right_Lower'], angles)

    def autoconfig(self):

        for label in self.motor_table:

            motor = self.get_motor(label)

            motor.motor.config.current_lim = 20
            motor.motor.config.calibration_current = 5
            motor.motor.config.pole_pairs = 12
            motor.motor.config.motor_type = 0

            motor.encoder.config.cpr = 8192

            motor.controller.config.vel_limit = 100000
            motor.controller.config.control_mode = 3
            motor.controller.config.pos_gain = 80
            motor.controller.config.vel_gain = 6 / 100000
            motor.controller.config.vel_integrator_gain = 0
            motor.controller.config.vel_limit_tolerance = 1.2
            motor.controller.config.vel_ramp_rate = 10000

            motor.trap_traj.config.vel_limit = 100000
            motor.trap_traj.config.accel_limit = 500000
            motor.trap_traj.config.decel_limit = 500000

        for serial in self.devices:

            device = self.devices[serial]

            device.config.brake_resistance = 0.0

            device.save_configuration()

    def reboot(self):
        for serial in self.devices:
            try:
                serial = self.devices[serial]
                print('SAVING DEVICE', serial)
                device.save_configuration()
                device.reboot()
            except Exception as e:
                self.debug(str(e))

    def dump_faults(self, clear=True):
        for serial in self.devices:
            print(serial)
            odrive.utils.dump_errors(self.devices[serial], clear)

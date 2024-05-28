#
# drone_flight_options.py -- Select drone flight path from menu.
# Description -- User selects drone control option (i.e. pair, takeoff,
#           calibrate, basic_XYZ path, cubic path).
#
# by Christopher Abel
# Revision History
# ----------------
# 03/10/2024 -- Original
#
# -------------------------------------------------
import csv
import datetime
import math
import sys
from codrone_edu.drone import *


def main():
    # Check whether '-f' was included as a command-line option
    # If so, open output datafile.
    if '-f' in sys.argv:
        write_datafile = True
        filename = 'drone_path'
        now = datetime.datetime.now()
        date_time_str = f"{now.month:02d}{now.day:02d}{now.year}_{now.hour:02d}{now.minute:02d}{now.second:02d}"
        file_out = open(filename + date_time_str + '.csv', 'w', newline='')
        writer = csv.writer(file_out)
        writer.writerow(['CoDrone Location vs Time', date_time_str])
    else:
        write_datafile = False
        file_out = None
        writer = None

    # Create drone object and DroneCalibrated object
    drone = Drone()
    drone_cal = DroneCalibrated(drone, write_datafile, writer)

    # Display menu and get user input
    selected = option_select()
    while selected != 7:
        selected = option_select()
        match selected:
            case 1: drone_cal.drone_pair()
            case 2: drone_cal.drone_takeoff(0.15)
            case 3: drone_cal.move_cal(20, 2)
            case 4: drone_cal.mov_xyz_abs(0.5, 0.5)
            case 5: drone_cal.move_xyz_simple(0.5, 0.5)
            case 6: drone_cal.drone_land()
            case _: break

    # If command-line option '-f' was present, close
    # open output datafile.
    if write_datafile:
        file_out.close()


def display_menu():
    #
    # Print user menu to console
    #
    print('\nDrone Flight Control')
    print('----------------------')
    print('Select from the following options --')
    print('1) Pair drone')
    print('2) Takeoff')
    print('3) Calibrate power of roll, pitch, throttle')
    print('4) Fly measured XYZ path')
    print('5) Fly simple XYZ path')
    print('6) Land')
    print('7) Exit program\n')


def option_select():
    selected = int(0)
    while selected < 1:
        display_menu()
        selected_str = input('Select option (1 - 7): ')
        if selected_str.isdigit():
            selected = int(selected_str)
            if not (1 <= selected <= 7):
                selected = int(0)
        if selected == 0:
            print('Please enter a single digit 1 - 7')
    return selected


class DroneCalibrated:
    # Constructor for class containing fields that store drone calibration values
    # and methods that use the calibrated values to produce atomic drone movements.
    def __init__(self, drone, write_datafile=False, file_writer=None):
        self.drone = drone
        self.paired = False
        self.takeoff = False
        self.calibrated = False
        self.pitch_f = 1.0
        self.pitch_b = 1.0
        self.roll_r = 1.0
        self.roll_l = 1.0
        self.throttle_u = 1.0
        self.throttle_d = 1.0
        self.yaw_cw = 1.0
        self.yaw_ccw = 1.0
        self.write_datafile = write_datafile
        self.file_writer = file_writer
        self.dwell = 5.0

    def get_drone_cal(self):
        # Getter method for drone calibration parameter values set in
        # this instance of DroneCalibrated object.
        return [self.pitch_f, self.pitch_b, self.roll_r, self.roll_l,
                self.throttle_u, self.throttle_d, self.yaw_cw, self.yaw_ccw]

    def set_drone_cal(self, cal_param_list):
        # Set pitch, roll, and throttle scale factors to specified values
        self.pitch_f = cal_param_list[0]
        self.pitch_b = cal_param_list[1]
        self.roll_r = cal_param_list[2]
        self.roll_l = cal_param_list[3]
        self.throttle_u = cal_param_list[4]
        self.throttle_d = cal_param_list[5]
        self.yaw_cw = cal_param_list[6]
        self.yaw_ccw = cal_param_list[7]
        self.calibrated = True

    def drone_pair(self):
        print('Pairing drone ...')
        self.drone.pair()
        self.paired = True
        batt = self.drone.get_battery()
        drone_temp = self.drone.get_temperature("C")
        [t, x, y, z] = self.drone.get_position_data()
        print(f'Temperature = {drone_temp:.2f}\tBattery = {batt:.2f}')
        if self.write_datafile:
            now = datetime.datetime.now()
            date_time_str = (f'{now.month:02d}/{now.day:02d}/{now.year} {now.hour:02d}:{now.minute:02d}'
                             f':{now.second:02d}.{now.microsecond:06d}')
            self.file_writer.writerow(['Pairing', date_time_str, 'temp', drone_temp, 'battery', batt,
                                       'position', t, x, y, z])

    def drone_takeoff(self, delta_z=0.0):
        #
        #   Set trim values to zero, sleep for 1s to allow them to take effect,
        #   then initiate takeoff, followed by hover for 3s.
        #
        if not self.paired:
            print('Drone must be paired before takeoff')
        else:
            # Change trim values
            self.drone.set_trim(0, 0)
            print("Adjusted trim values (roll, pitch): ", self.drone.get_trim())
            time.sleep(1)
            self.drone.takeoff()
            self.takeoff = True
            [t, x, y, z] = self.drone.get_position_data()
            time.sleep(self.dwell)
            print(f'Increasing height from {z:.2f}m to {z + delta_z:.2f}m')
            self.send_abs_pos_verif(x, y, z, x, y, z + delta_z, 0.2, 0.5, pattern='Takeoff')
            [t, x, y, z] = self.drone.get_position_data()
            time.sleep(self.dwell)
            if self.write_datafile:
                now = datetime.datetime.now()
                date_time_str = (f'{now.month:02d}/{now.day:02d}/{now.year} {now.hour:02d}:{now.minute:02d}'
                                 f':{now.second:02d}.{now.microsecond:06d}')
                batt = self.drone.get_battery()
                drone_temp = self.drone.get_temperature("C")
                self.file_writer.writerow(['Takeoff', date_time_str, 'temp', drone_temp, 'battery', batt,
                                           'position', t, x, y, z])
            self.drone.hover(3)

    def drone_land(self):
        #
        # Drone hovers for 3s, and then lands.
        #
        self.hover_w_del(2, 1)
        print('Drone Landing ...')
        batt = self.drone.get_battery()
        drone_temp = self.drone.get_temperature("C")
        print(f'Temperature = {drone_temp:.2f}\tBattery = {batt:.2f}')
        [t, x, y, z] = self.drone.get_position_data()
        # Issue landing command several times to ensure it was received
        for i in range(0, 2):
            self.drone.land()
        if self.write_datafile:
            now = datetime.datetime.now()
            date_time_str = (f'{now.month:02d}/{now.day:02d}/{now.year} {now.hour:02d}:{now.minute:02d}'
                             f':{now.second:02d}.{now.microsecond:06d}')
            self.file_writer.writerow(['Landing', date_time_str, 'temp', drone_temp, 'battery', batt,
                                       'position', t, x, y, z])
        self.takeoff = False

    def move_cal(self, power_lev, duration):
        #
        # Function to produce scale values for pitch, roll, and throttle
        # power levels so that, to a first order, a pitch, roll, and throttle movement
        # of a given nominal power and duration move the drone in approximately equal
        # distances along the X, Y, and Z axes.
        #
        # Input: pointer to drone
        # Output: list of scale values [pitch_f, pitch_b, roll_r, roll_l, throttle_u, throttle_d]
        #
        pitch_b = []
        roll_r = []
        roll_l = []
        throttle_u = []
        throttle_d = []
        repeats = 2
        if not (self.paired and self.takeoff):
            print('The drone must be paired, and must have taken off, before calibration.')
        else:
            for i in range(0, repeats):
                self.drone.set_throttle(0)
                self.drone.set_roll(0)
                ##################################
                #   Calibrate pitch
                #
                # Move drone forward at pitch=power_lev for duration time
                self.drone.set_pitch(power_lev)
                self.drone.hover(2)
                [t0, x, y0, z] = self.drone.get_position_data()
                time.sleep(1)
                print('Calibration -- moving forward')
                self.drone.move(duration)
                self.drone.hover(2)
                [t1, x, y1, z] = self.drone.get_position_data()
                delta_y_forward = y1 - y0
                # Move drone backward at pitch=power_lev for duration time
                self.drone.set_pitch(-power_lev)
                time.sleep(1)
                print('Calibration -- moving backward')
                self.drone.move(duration)
                self.drone.hover(2)
                [t2, x, y2, z] = self.drone.get_position_data()
                delta_y_back = y2 - y1
                pitch_b.append(delta_y_forward / delta_y_back)
                print(f'Forward: {delta_y_forward}\tBack: {delta_y_back}')
                self.drone.set_pitch(0)
                ##################################
                #   Calibrate roll
                #
                # Move drone right at roll=power_lev for duration time
                # self.drone.set_roll(power_lev)
                # time.sleep(1)
                # self.drone.hover(2)
                # [t0, x0, y, z] = self.drone.get_position_data()
                # print('Calibration -- moving right')
                # self.drone.move(duration)
                # self.drone.hover(2)
                # [t1, x1, y, z] = self.drone.get_position_data()
                # delta_x_right = x1 - x0
                # # Move drone left at roll=power_lev for duration time
                # self.drone.set_roll(-power_lev)
                # print('Calibration -- moving left')
                # self.drone.move(duration)
                # self.drone.hover(2)
                # [t2, x2, y, z] = self.drone.get_position_data()
                # delta_x_left = x2 - x1
                # roll_r.append(delta_y_forward / delta_x_right)
                # roll_l.append(delta_y_forward / delta_x_left)
                # self.drone.set_roll(0)
                ##################################
                #   Calibrate throttle
                #
                # Move drone up at throttle=power_lev for duration time
                # self.drone.set_throttle(power_lev)
                # self.drone.hover(2)
                # [t0, x, y, z0] = self.drone.get_position_data()
                # print('Calibration -- moving up')
                # self.drone.move(duration)
                # self.drone.hover(2)
                # [t1, x, y, z1] = self.drone.get_position_data()
                # delta_z_up = z1 - z0
                # Move drone down at throttle=power_lev for duration time
                # self.drone.set_throttle(-power_lev)
                # print('Calibration -- moving down')
                # self.drone.move(duration)
                # self.drone.hover(2)
                # [t2, x, y, z2] = self.drone.get_position_data()
                # delta_z_down = z2 - z1
                # throttle_u.append(delta_y_forward / delta_z_up)
                # throttle_d.append(delta_y_forward / delta_z_down)

            # Print average of each scale factor to console, and form list and
            # assign scale factors to corresponding instance parameters.
            print('Calibration Scale Factors:')
            print(f'\tForward = {1.0:.3f}\tBack = {sum(pitch_b) / repeats:.3f}')
            # print(f'\tRight = {sum(roll_r) / repeats:.3f}\tLeft = {sum(roll_l) / repeats:.3f}')
            # print(f'\tUp = {sum(throttle_u) / repeats:.3f}\tDown = {sum(throttle_d) / repeats:.3f}')
            self.set_drone_cal([1.0, sum(pitch_b) / repeats, sum(roll_r) / repeats, sum(roll_l) / repeats,
                                sum(throttle_u) / repeats, sum(throttle_d) / repeats])
            self.set_drone_cal([1.0, sum(pitch_b) / repeats, 1.0, 1.0, 1.0, 1.0])
            if self.write_datafile:
                now = datetime.datetime.now()
                date_time_str = f'{now.month:02d}{now.day:02d}{now.year}_{now.hour:02d}{now.minute:02d}{now.second:02d}'
                self.file_writer.writerow(['Calibration', date_time_str, self.pitch_f, self.pitch_b, self.roll_r,
                                           self.roll_l, self.throttle_u, self.throttle_d, self.yaw_cw, self.yaw_ccw])

    def mov_xyz_abs(self, movement_lim, velocity):
        #
        # Move along X, Y, and Z axes, with an absolute distance and velocity, using corresponding
        # send_abs_pos_w_output() command.
        #   Parameters:
        #       movement_lim = movement amount, from center point, in X, Y, and Z directions (m)
        #       velocity = movement velocity (m/s)
        #
        [t0, x0, y0, z0] = self.drone.get_position_data()
        repeats = 2
        pattern = 'XYZ_abs'
        for i in range(0, repeats):
            # Move vertically (Z axis)
            z1 = z0 + movement_lim
            self.send_abs_pos_w_output(x0, y0, z0, x0, y0, z1, velocity, pattern)
            self.drone.hover(1)
            z2 = z0 - movement_lim
            self.send_abs_pos_w_output(x0, y0, z1, x0, y0, z2, velocity, pattern)
            self.drone.hover(1)
            self.send_abs_pos_w_output(x0, y0, z2, x0, y0, z0, velocity, pattern)
            # Move left / right (Y axis)
            y1 = y0 + movement_lim
            self.send_abs_pos_w_output(x0, y0, z0, x0, y1, z0, velocity, pattern)
            self.drone.hover(1)
            y2 = y0 - movement_lim
            self.send_abs_pos_w_output(x0, y1, z0, x0, y2, z0, velocity, pattern)
            self.drone.hover(1)
            self.send_abs_pos_w_output(x0, y2, z0, x0, y0, z0, velocity, pattern)
            # Move forward / backward (X axis)
            x1 = x0 + movement_lim
            self.send_abs_pos_w_output(x0, y0, z0, x1, y0, z0, velocity, pattern)
            self.drone.hover(1)
            x2 = x0 - movement_lim
            self.send_abs_pos_w_output(x1, y0, z0, x2, y0, z0, velocity, pattern)
            self.drone.hover(1)
            self.send_abs_pos_w_output(x2, y0, z0, x0, y0, z0, velocity, pattern)

    def move_xyz_simple(self, dist, velocity):
        #
        # Simple method to move drone forward and backward, right and left, and
        # up and down, using send_absolute_position() function.
        #
        pattern = 'XYZ_simple'
        [t0, x0, y0, z0] = self.drone.get_position_data()
        self.output_position(pattern)
        repeats = 2
        for i in range(0, repeats):
            # Move forward and backward (X axis)
            print('Moving forward')
            self.drone.send_absolute_position(x0 + dist, y0, z0, velocity, 0, 0)
            time.sleep(self.dwell + dist/velocity)
            self.output_position(pattern)
            print('Moving backward')
            self.drone.send_absolute_position(x0 - dist, y0, z0, velocity, 0, 0)
            time.sleep(self.dwell + dist/velocity)
            self.output_position(pattern)
            print('Returning to center')
            self.drone.send_absolute_position(x0, y0, z0, velocity, 0, 0)
            time.sleep(self.dwell + dist/velocity)
            self.output_position(pattern)

            # Move right and left (Y axis)
            print('Moving left')
            self.drone.send_absolute_position(x0, y0 + dist, z0, velocity, 0, 0)
            time.sleep(self.dwell + dist / velocity)
            self.output_position(pattern)
            print('Moving right')
            self.drone.send_absolute_position(x0, y0 - dist, z0, velocity, 0, 0)
            time.sleep(self.dwell + dist / velocity)
            self.output_position(pattern)
            print('Returning to center')
            self.drone.send_absolute_position(x0, y0, z0, velocity, 0, 0)
            time.sleep(self.dwell + dist / velocity)
            self.output_position(pattern)

            # Move up and down (Z axis)
            print('Moving up')
            self.drone.send_absolute_position(x0, y0, z0 + dist, velocity, 0, 0)
            time.sleep(self.dwell + dist / velocity)
            self.output_position(pattern)
            print('Moving down')
            self.drone.send_absolute_position(x0, y0, z0 - dist, velocity, 0, 0)
            time.sleep(self.dwell + dist / velocity)
            self.output_position(pattern)
            print('Returning to center')
            self.drone.send_absolute_position(x0, y0, z0, velocity, 0, 0)
            time.sleep(self.dwell + dist / velocity)
            self.output_position(pattern)

    def send_abs_pos_w_output(self, x0, y0, z0, x1, y1, z1, velocity, pattern='Movement'):
        #
        # Customized version of send_absolute_position() function, which moves
        # drone from [x0, y0, z0] to [x1, y1, z1] at a speed of velocity (m/s).
        # The method calculates the total translation distance, and splits the
        # movement up into smaller movements, each of duration approx. delta_t.
        # If self.write_datafile = True, then output drone position after each
        # movement.
        #
        delta_t = 0.05    # Segment time, in seconds
        r_total = math.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
        num_segments = round((r_total / velocity)/delta_t)
        delta_x = 0 if num_segments == 0 else (x1 - x0)/num_segments
        delta_y = 0 if num_segments == 0 else (y1 - y0)/num_segments
        delta_z = 0 if num_segments == 0 else (z1 - z0) / num_segments
        for i in range(0, num_segments):
            self.drone.send_absolute_position(x0 + i*delta_x, y0 + i*delta_y, z0 + i*delta_z, velocity, 0, 0)
            if self.write_datafile:
                now = datetime.datetime.now()
                date_time_str = (f'{now.month:02d}/{now.day:02d}/{now.year} {now.hour:02d}:{now.minute:02d}'
                                 f':{now.second:02d}.{now.microsecond:06d}')
                drone_pos = self.drone.get_position_data()
                self.file_writer.writerow([pattern, date_time_str, drone_pos[0], drone_pos[1], drone_pos[2],
                                           drone_pos[3]])

    def send_abs_pos_verif(self, x0, y0, z0, x1, y1, z1, vel, min_delay=0.0, complete=0.75, pattern='Movement',
                           write_note=False):
        #
        # Wrapper around send_absolute_position() that puts the command in a while loop, causing it to
        # be repeated until movement has been verified.
        #
        start_time = time.time()
        time_expired = False
        movement_complete = False
        iteration = 0
        while not time_expired or not movement_complete:
            iteration += 1
            self.drone.send_absolute_position(x1, y1, z1, vel, 0, 0)
            # Get drone position after command issued, and save to output file
            [t, x, y, z] = self.drone.get_position_data()
            if self.write_datafile:
                now = datetime.datetime.now()
                date_time_str = (f'{now.month:02d}/{now.day:02d}/{now.year} {now.hour:02d}:{now.minute:02d}'
                                 f':{now.second:02d}.{now.microsecond:06d}')
                if write_note:
                    self.file_writer.writerow([pattern, date_time_str, t, x, y, z,
                                               'Notes -- send_abs_pos', iteration, x0, y0, z0, x1, y1, z1])
                else:
                    self.file_writer.writerow([pattern, date_time_str, t, x, y, z])
            # Check whether execution time exceeds minimum delay time AND whether intended
            # drone displacement is substantially complete.  If not, continue in while loop.
            time_expired = (time.time() - start_time >= min_delay)
            movement_complete = ((math.fabs(x - x0) >= complete*math.fabs(x1 - x0))
                                 and (math.fabs(y - y0) >= complete*math.fabs(y1 - y0))
                                 and (math.fabs(z - z0) >= complete*math.fabs(z1 - z0)))

    def hover_w_del(self, t_hover, t_sleep, pattern='Hover', write_note=False):
        start_time = time.time()
        time_expired = False
        # Get drone position
        [t, x0, y0, z0] = self.drone.get_position_data()
        iteration = 0
        while not time_expired:
            iteration += 1
            self.drone.send_absolute_position(x0, y0, z0, 0.1, 0, 0)
            # Get drone position after command issued, and save to output file
            [t, x, y, z] = self.drone.get_position_data()
            if self.write_datafile:
                now = datetime.datetime.now()
                date_time_str = (f'{now.month:02d}/{now.day:02d}/{now.year} {now.hour:02d}:{now.minute:02d}'
                                 f':{now.second:02d}.{now.microsecond:06d}')
                if write_note:
                    self.file_writer.writerow([pattern, date_time_str, t, x, y, z, 'Notes -- hover', iteration, x0, y0, z0])
                else:
                    self.file_writer.writerow([pattern, date_time_str, t, x, y, z])
            # Check whether execution time exceeds minimum delay time AND whether intended
            # drone displacement is substantially complete.  If not, continue in while loop.
            time_expired = (time.time() - start_time >= t_hover + t_sleep)

    def output_position(self, pattern='Movement'):
        #
        # Output current position of drone to datafile
        #
        if self.write_datafile:
            now = datetime.datetime.now()
            date_time_str = (f'{now.month:02d}/{now.day:02d}/{now.year} {now.hour:02d}:{now.minute:02d}'
                             f':{now.second:02d}.{now.microsecond:06d}')
            [t, x, y, z] = self.drone.get_position_data()
            self.file_writer.writerow([pattern, date_time_str, t, x, y, z])


# Main code
if __name__ == '__main__':
    main()

#
# drone_flight_patt1.py -- Collection of functions which implement complex drone flight patterns.
# Description -- Functions use attributes and methods of the DroneCalibrated class.
#
# by Christopher Abel
# Revision History
# ----------------
# 03/17/2024 -- Original
#
# -------------------------------------------------
import datetime
import sys
import csv
import math
from drone_flight_options import DroneCalibrated
from codrone_edu.drone import *


def main():
    if '-f' in sys.argv:
        print('Saving data to output file ...')
    if '-r' in sys.argv:
        print('Random flight path')
    else:
        print('Simple YZ flight path')
    # If command-line options include '-f', open output csv file.
    [write_datafile, file_out, writer] = open_datafile(sys.argv, 'drone_path')

    # Create drone object and DroneCalibrated object. Pair drone, and then takeoff.
    write_notes = False
    drone = Drone()
    drone_cal = DroneCalibrated(drone, write_datafile, writer)
    drone_cal.drone_pair()
    drone_cal.drone_takeoff(0.15)

    # Execute pattern; if '-r' option present in command-line, execute random pattern.
    # Else, execute simple YZ pattern.
    if '-r' in sys.argv:
        limits = [-0.1, 0.1, -1.5, 1.5, -0.1, 0.1]
        # move_random_limits(drone_cal, 0.5, limits, 0.5, 2.0, 10,
        #                    write_note=write_notes)
        max_xyz = [0.0, 1.0, 0.0]
        vel = 0.5
        delay = 2.0
        num_segs = 20
        move_random_xyz(drone_cal, max_xyz, limits, vel, delay, num_segs, write_note=write_notes)
    else:
        move_yz_simple(drone_cal, 0.75, 0.5, 0.5, 2.0, write_note=write_notes)
        # move_y_simple(drone_cal, 0.75, 0.5, 2.0)
        # move_z_simple(drone_cal, 0.5, 0.2, 2.0)
        # move_x_simple(drone_cal, 0.75, 0.5, 2.0)

    # Drone landing. If command-line option '-f' was present, close output datafile.
    drone_cal.drone_land()
    if write_datafile:
        file_out.close()


def move_x_simple(drone_cal, del_x, vel, delay=0.0, repeats=2):
    # Simple movement of drone in X (forward / backward) directions
    # using send_abs_pos_verif() method.
    pattern = 'Simple_X'
    t_sleep = 1.0
    t_hover = 2.0
    [t, x0, y0, z0] = drone_cal.drone.get_position_data()
    for i in range(0, repeats):
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0 + del_x, y0, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0 + del_x, y0, z0, x0, y0, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0 - del_x, y0, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0 - del_x, y0, z0, x0, y0, z0, vel, delay, pattern=pattern)


def move_z_simple(drone_cal, del_z, vel, delay=0.0, repeats=2):
    # Simple movement of drone in Y (left / right) directions
    # using send_abs_pos_verif() method.
    pattern = 'Simple_Z'
    t_sleep = 1.0
    t_hover = 2.0
    [t, x0, y0, z0] = drone_cal.drone.get_position_data()
    # Reduce del_z if current drone height is insufficient
    delta_z = del_z if del_z < 0.75 * z0 else 0.75 * z0
    for i in range(0, repeats):
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0, z0 + delta_z, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0, y0, z0 + delta_z, x0, y0, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0, z0 - delta_z, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0, y0, z0 - delta_z, x0, y0, z0, vel, delay, pattern=pattern)


def move_y_simple(drone_cal, del_y, vel, delay=0.0, repeats=2):
    # Simple movement of drone in Y (left / right) directions
    # using send_abs_pos_verif() method.
    pattern = 'Simple_Y'
    t_sleep = 1.0
    t_hover = 2.0
    [t, x0, y0, z0] = drone_cal.drone.get_position_data()
    for i in range(0, repeats):
        # Move left / right
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0 + del_y, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0, y0 + del_y, z0, x0, y0, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0 - del_y, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)
        drone_cal.send_abs_pos_verif(x0, y0 - del_y, z0, x0, y0, z0, vel, delay, pattern=pattern)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern)


def move_yz_simple(drone_cal, del_y, del_z, vel, delay=0.0, repeats=2, write_note=False):
    # Simple movement of drone in Y (left / right) and Z (up / down) directions
    # using send_abs_pos_verif() method.
    pattern = 'Simple_YZ'
    t_sleep = 1.0
    t_hover = 2.0
    [t, x0, y0, z0] = drone_cal.drone.get_position_data()
    # Reduce del_z if current drone height is insufficient
    delta_z = del_z if del_z < 0.75 * z0 else 0.75 * z0
    for i in range(0, repeats):
        # Move left / right
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0 + del_y, z0, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern, write_note=write_note)
        drone_cal.send_abs_pos_verif(x0, y0 + del_y, z0, x0, y0, z0, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern, write_note=write_note)
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0 - del_y, z0, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern, write_note=write_note)
        drone_cal.send_abs_pos_verif(x0, y0 - del_y, z0, x0, y0, z0, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern, write_note=write_note)
        # Move up / down
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0, z0 + delta_z, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern, write_note=write_note)
        drone_cal.send_abs_pos_verif(x0, y0, z0 + delta_z, x0, y0, z0, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern, write_note=write_note)
        drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0, z0 - delta_z, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(t_hover, t_sleep, pattern, write_note=write_note)
        drone_cal.send_abs_pos_verif(x0, y0, z0 - delta_z, x0, y0, z0, vel, delay,
                                     pattern=pattern, write_note=write_note)


def move_random_limits(drone_cal, max_r, limits, vel, delay=0.0, segments=10, write_note=False):
    """
    Random movement of drone in 3-dimensional space. Maximum displacement in each segment
    will not exceed max_r. Limitations on X, Y, Z displacement contained in list limits.
    If random segment will take drone outside the limits, then the sign of the displacement
    in that segment is inverted.
    :param drone_cal: Pointer to object of type DroneCalibrated.
    :param max_r:
    :param limits:
    :param vel:
    :param delay:
    :param segments:
    :param write_note:
    :return:
    """
    pattern = 'Random_Limits'
    t_sleep = 1.0
    [t, x0, y0, z0] = drone_cal.drone.get_position_data()
    [x_neg, x_pos, y_neg, y_pos, z_neg, z_pos] = limits

    # Perform initial calibration by moving to the left, then to the right, then back to center (Y axis)
    drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0 + y_pos, z0, vel, delay,
                                 pattern=pattern, write_note=write_note)
    drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)
    drone_cal.send_abs_pos_verif(x0, y0 + y_pos, z0, x0, y0 + y_neg, z0, vel, delay,
                                 pattern=pattern, write_note=write_note)
    drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)
    drone_cal.send_abs_pos_verif(x0, y0 + y_neg, z0, x0, y0, z0, vel, delay,
                                 pattern=pattern, write_note=write_note)
    drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)

    # Move in segments number of random segments
    for i in range(0, segments):
        # Get current position; calculate random magnitude and direction of displacement, and magnitude of velocity.
        [t, x, y, z] = drone_cal.drone.get_position_data()
        r = random.uniform(0.5 * max_r, max_r)
        theta = random.uniform(0, math.pi)
        phi = random.uniform(0, 2 * math.pi)
        velocity = random.uniform(0.5 * vel, vel)
        # Calculate displacement values in each dimension. If displacement will result in movement beyond
        # current limits, invert sign of displacement in that dimension.
        del_z = r * math.cos(theta)
        del_y = r * math.sin(theta) * math.cos(phi)
        del_x = r * math.sin(theta) * math.sin(phi)
        if x + del_x < x0 + x_neg:
            del_x = math.fabs(del_x)
        elif x + del_x > x0 + x_pos:
            del_x = -math.fabs(del_x)
        if y + del_y < y0 + y_neg:
            del_y = math.fabs(del_y)
        elif y + del_y > y0 + y_pos:
            del_y = -math.fabs(del_y)
        if z + del_z < z0 + z_neg:
            del_z = math.fabs(del_z)
        elif z + del_z > z0 + z_pos:
            del_z = -math.fabs(del_z)
        # Move to position defined by current segment
        drone_cal.send_abs_pos_verif(x, y, z, x + del_x, y + del_y, z + del_z, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)
        # time.sleep(t_sleep)


def move_random_xyz(drone_cal, max_xyz, limits, vel, delay=0.0, num_segs=10, write_note=False):
    """
    Move drone in a random pattern in 3-dimensional space, with limits in the X, Y, and Z dimensions.
    If (x0, y0, z0) is the initial position of the drone when the function is called, the drone
    should not move outside the range:
        [x0 - limits[0], x0 + limits[1]] in the x dimension.
        [y0 - limits[2], y0 + limits[3]] in the y dimension.
        [z0 - limits[4], z0 + limits[5]] in tye z dimension.
    The drone movement is split into num_segs (default=10) segments. In each segment, the drone
    will move by an amount [delta_x, delta_y, delta_z] where:
        delta_x is a uniformly distributed value within the range [-max_xyz[0], max_xyz[0]]
        delta_y is a uniformly distributed value within the range [-max_xyz[1], max_xyz[1]]
        delta_z is a uniformly distributed value within the range [-max_xyz[2], max_xyz[2]]
    The velocity of movement in each dimension is set by the value of the vel parameter.

    :param drone_cal: Pointer to object of type DroneCalibrated.
    :param max_xyz: List [delta_x_max, delta_y_max, delta_z_max] which the define the
            x, y, z limits of movement in a single segment.  For example, the movement along
            the x axis in a single segment is in the range [-delta_x_max, delta_x_max].
    :param limits: List[x_neg, x_pos, y_neg, y_pos, z_neg, z_pos] which define the limits of the X, Y, Z
            "box" in which the drone movement is constrained.
    :param vel: Constant velocity of the drone in each segment, in (m/s)
    :param delay:
    :param segments:
    :param write_note:
    :return:
    """
    pattern = 'Random_Limits'
    t_sleep = 1.0
    [t, x0, y0, z0] = drone_cal.drone.get_position_data()
    [x_neg, x_pos, y_neg, y_pos, z_neg, z_pos] = limits

    # Perform initial calibration by moving to the left, then to the right, then back to center (Y axis)
    drone_cal.send_abs_pos_verif(x0, y0, z0, x0, y0 + y_pos, z0, vel, delay,
                                 pattern=pattern, write_note=write_note)
    drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)
    drone_cal.send_abs_pos_verif(x0, y0 + y_pos, z0, x0, y0 + y_neg, z0, vel, delay,
                                 pattern=pattern, write_note=write_note)
    drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)
    drone_cal.send_abs_pos_verif(x0, y0 + y_neg, z0, x0, y0, z0, vel, delay,
                                 pattern=pattern, write_note=write_note)
    drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)

    # Move in segments number of random segments
    for i in range(0, num_segs):
        # Get current position; calculate random magnitude and direction of displacement, and magnitude of velocity.
        [t, x, y, z] = drone_cal.drone.get_position_data()
        # Calculate random displacement values in each dimension. If displacement will result in movement beyond
        # current limits, invert sign of displacement in that dimension.
        delta_x = random.uniform(-max_xyz[0], max_xyz[0])
        delta_y = random.uniform(-max_xyz[1], max_xyz[1])
        delta_z = random.uniform(-max_xyz[2], max_xyz[2])
        if x + delta_x < x0 + x_neg:
            delta_x = math.fabs(delta_x)
        elif x + delta_x > x0 + x_pos:
            delta_x = -math.fabs(delta_x)
        if y + delta_y < y0 + y_neg:
            delta_y = math.fabs(delta_y)
        elif y + delta_y > y0 + y_pos:
            delta_y = -math.fabs(delta_y)
        if z + delta_z < z0 + z_neg:
            delta_z = math.fabs(delta_z)
        elif z + delta_z > z0 + z_pos:
            delta_z = -math.fabs(delta_z)

        # Move to position defined by current segment
        drone_cal.send_abs_pos_verif(x, y, z, x + delta_x, y + delta_y, z + delta_z, vel, delay,
                                     pattern=pattern, write_note=write_note)
        drone_cal.hover_w_del(1.0, t_sleep, pattern=pattern, write_note=write_note)


def open_datafile(argv, root_name='drone_path'):
    #
    # Simple function to open a csv file to write drone flight status and
    # movements. The file is only opened if the command-line arguments include
    # the '-f' option.
    #   Parameters: root_name -- String that is used to form the beginning of the
    #                   csv filename.
    #               argv -- List containing command-line arguments
    #   Returns: List containing
    #               write_datafile = Boolean value; true when '-f' argument is present in argv
    #               file_out = Pointer to open output file.  Must be closed by calling program.
    #               writer = Pointer to csv writer object.
    #
    if '-f' in argv:
        write_datafile = True
        filename = root_name
        now = datetime.datetime.now()
        date_time_str = f"{now.month:02d}{now.day:02d}{now.year}_{now.hour:02d}{now.minute:02d}{now.second:02d}"
        file_out = open(filename + date_time_str + '.csv', 'w', newline='')
        writer = csv.writer(file_out)
        writer.writerow(['CoDrone Location vs Time', date_time_str])
    else:
        write_datafile = False
        file_out = None
        writer = None
    return [write_datafile, file_out, writer]


# Main code
if __name__ == '__main__':
    main()

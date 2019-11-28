import cv2
import numpy as np
import math
from time import time, sleep

# import matplotlib.pyplot as plt

lower = [0, 0, 0]
higher = [50, 50, 50]
lower_rgb = np.array(lower)
higher_rgb = np.array(higher)
size = width, height = 320, 240
center = width // 2, height // 2
width_min, width_max = 100, 220
height_min, height_max = 0, 150
pi_na_dva = math.pi / 2
last_yaw_point = (center[0], 0)

average_time_dict = {
    't': 0,
    's': 0
}

sigmaEta = 3.5  # sensor disperson
sigmaPsi = 1  # main dispersion

prev_error = sigmaEta ** 2
x_opt_prev = 0


def find_nearest(array, value):
    array = np.asarray(array)
    idx = np.abs(array - value).argmin()
    return array[idx]


def find_nearest_white(img, target):
    nonzero = cv2.findNonZero(img)
    distances = np.sqrt((nonzero[:, :, 0] - target[0]) ** 2 + (nonzero[:, :, 1] - target[1]) ** 2)
    nearest_index = np.argmin(distances)
    return nonzero[nearest_index]


def kalman_easy(sensor_value):
    global prev_error, x_opt_prev
    sensor_value = sensor_value * 100

    def middle_error(prev_error):
        optimal_error = math.pow(
            (sigmaEta ** 2 * (prev_error ** 2 + sigmaPsi ** 2)) / (prev_error ** 2 + sigmaPsi ** 2 + sigmaEta ** 2),
            0.5)
        return optimal_error

    def kalman_value_next(prev_error):
        middle_prev_error = middle_error(prev_error)
        kalman_value = middle_prev_error ** 2 / sigmaEta ** 2
        return kalman_value, middle_prev_error

    def x_opt_next(x_opt_prev, kalman_value, sensor_value):
        x_opt1 = kalman_value * sensor_value + (1 - kalman_value) * x_opt_prev
        return x_opt1

    kalman_value, prev_error = kalman_value_next(prev_error)
    x_opt_prev = x_opt_next(x_opt_prev, kalman_value, sensor_value)

    return x_opt_prev


def get_yaw(frame):
    global_mask = cv2.inRange(frame, lower_rgb, higher_rgb)
    mask = global_mask[height_min:height_max, width_min:width_max]
    # contours, _ = cv2.findContours(global_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(frame, contours, -1, (255, 150, 100), 2)
    dx = width_min
    max_y = 0

    global time_from_start, times

    try:
        while mask[max_y].sum() == 0:
            max_y += 1
    except:
        mask = global_mask[height_min:height_max, :]
        dx = 0
        max_y = 0
        try:
            while mask[max_y].sum() == 0:
                max_y += 1
        except:
            mask = global_mask
            try:
                while mask[max_y].sum() == 0:
                    max_y += 1
            except:
                return frame, None, None

    # print max_y
    # cv2.imshow('mask', global_mask)
    nearest_x = find_nearest(np.where(mask[max_y] == 255)[0], center[0])

    try:
        yaw = math.atan2(max_y - center[1], dx + nearest_x - center[0]) + pi_na_dva
    except:
        yaw = None

    no_filtered = yaw
    yaw = kalman_easy(yaw)
    yaw = yaw / 100
    print("yaw on tik: norm -- {}, filtered -- {}".format(no_filtered, yaw))

    long_mask = global_mask[:, width_min:width_max]
    y_i = 0
    _dx = width_min

    while True:
        try:
            if long_mask[center[1] + y_i].sum() != 0:
                break
            elif long_mask[center[1] - y_i].sum() != 0:
                y_i = -y_i
                break
            else:
                y_i += 1
        except:
            long_mask = global_mask
            y_i = 0
            _dx = 0

    x_i = find_nearest(np.where(long_mask[center[1] + y_i] == 255)[0], center[0])
    cv2.circle(frame, (x_i + _dx, center[1] + y_i), 4, (0, 0, 255), cv2.FILLED)

    cv2.circle(frame, (dx + nearest_x, max_y), 5, (0, 255, 255), cv2.FILLED)
    cv2.line(frame, center, (dx + nearest_x, max_y), (0, 255, 255), 3)

    # cv2.imshow('mask', global_mask)
    return frame, yaw, (x_i + dx - center[0])


def get_better_yaw(frame):
    global last_yaw_point
    global_mask = get_mask(frame)

    yaw_coord_from_center = find_nearest_white(global_mask, (center[0], 0))[0]
    yaw_coords = find_nearest_white(global_mask, last_yaw_point)
    yaw_coord = yaw_coords[0]

    for y in yaw_coords:
        if y[1] > yaw_coord[1]:
            yaw_coord = y

    yaw_coord = (yaw_coord + yaw_coord_from_center) // 2
    last_yaw_point = yaw_coord

    y_coords = find_nearest_white(global_mask, center)[0]

    cv2.circle(frame, tuple(yaw_coord), 5, (255, 0, 255), cv2.FILLED)
    cv2.circle(frame, tuple(y_coords), 5, (0, 255, 255), cv2.FILLED)
    cv2.line(frame, center, tuple(y_coords), (0, 255, 255), 3)
    cv2.line(frame, center, tuple(yaw_coord), (255, 0, 255), 3)

    yaw = math.atan2(yaw_coord[1] - center[1], yaw_coord[0] - center[0]) + pi_na_dva

    if yaw > math.pi:
        yaw -= 2 * math.pi

    return frame, yaw, y_coords[0] - center[0]


def get_mask(frame):
    mask = cv2.inRange(frame, lower_rgb, higher_rgb)
    return mask


if __name__ == '__main__':
    cap = cv2.VideoCapture('assets/flow_line_testv2.mp4')
    out = cv2.VideoWriter('output/output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 60.0, (width, height))

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (width, height))
            t = time()
            try:
                frame, yaw, y = get_better_yaw(frame)
            except TypeError:
                print('No line detected')
            average_time_dict['t'] += time() - t
            average_time_dict['s'] += 1
            cv2.imshow('frame', frame)
            out.write(frame)
        else:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    while not cv2.waitKey(1) & 0xFF == ord('q'):
        sleep(0.01)
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    # print(average_time_dict['t'] / average_time_dict['s'])
    # print(average_time_dict['s'] / average_time_dict['t'])

# 0.00111650835135 - time taken to execute get_yaw with drawing contours
# 0.000837510282343 - time taken to execute get_yaw without drawing contours
# 1.33312793274 times longer

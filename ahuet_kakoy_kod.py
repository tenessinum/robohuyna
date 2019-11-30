import cv2
import numpy as np
import math
from time import time, sleep

# Function handlers
on_clever = False
enable_grayscale = True
enable_kalman_filter = True
enable_mask_filter = True
mask_debug = True

lower_mask_value = 180
higher_mask_value = 255
size = width, height = 320, 240
center = width // 2, height // 2
# width_min, width_max = 100, 220
# height_min, height_max = 0, 150
pi_na_dva = math.pi / 2
last_yaw_point = (center[0], 0)
last_yaw = 0
last_y = 0

average_time_dict = {
    't': 0,
    's': 0
}

sigmaEta = 3.1  # sensor disperson
sigmaPsi = 1  # main dispersion
prev_error = sigmaEta ** 2
x_opt_prev = 0

kernel_edge = 3
first_erode_iterations = 10
second_dilate_iterations = 14  # should be about 2 times larger then first_erode_iterations
delta_morphological_iterations = 17  # used to separate working wrea
kernel = np.ones((kernel_edge, kernel_edge), np.uint8)


def find_nearest(array, value):
    array = np.asarray(array)
    idx = np.abs(array - value).argmin()
    return array[idx]


def find_nearest_white(img, target):
    nonzero = cv2.findNonZero(img)
    distances = np.sqrt((nonzero[:, :, 0] - target[0]) ** 2 + (nonzero[:, :, 1] - target[1]) ** 2)
    nearest_index = np.argmin(distances)
    if distances[nearest_index] > 60:
        raise NameError
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


def get_filtered_area(g_mask):
    """
    The function that selects the main working area for subsequent processing

    :param image: binary main image
    :return:
    g_mask:
    contours: array contaiting the main contour of working area
    """
    g_mask = cv2.bitwise_not(g_mask)
    filter_mask = cv2.erode(g_mask, kernel, iterations=first_erode_iterations)
    filter_mask = cv2.dilate(filter_mask, kernel, iterations=second_dilate_iterations)
    filter_mask = cv2.erode(filter_mask, kernel, iterations=second_dilate_iterations-first_erode_iterations+delta_morphological_iterations)
    if on_clever:
        _, contours, _ = cv2.findContours(filter_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    else:
        contours, hierarchy = cv2.findContours(filter_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    filter_mask = cv2.bitwise_not(filter_mask)
    g_mask = cv2.bitwise_or(g_mask, filter_mask)
    g_mask = cv2.bitwise_not(g_mask)
    return g_mask, contours


def get_mask(frame, reverse=0):
    if not reverse:
        if enable_grayscale:
            mask = cv2.inRange(frame, lower_mask_value, higher_mask_value)
        else:
            mask = cv2.inRange(frame, np.full(3, lower_mask_value), np.full(3, higher_mask_value))
    else:
        if enable_grayscale:
            mask = cv2.inRange(frame, math.fabs(higher_mask_value - 255), math.fabs(lower_mask_value - 255))
        else:
            mask = cv2.inRange(frame,
                               np.full(3, math.fabs(higher_mask_value - 255)),
                               np.full(3, math.fabs(lower_mask_value - 255)))
    return mask


# def get_yaw(frame):
#    global_mask = cv2.inRange(frame, lower_rgb, higher_rgb)
#    mask = global_mask[height_min:height_max, width_min:width_max]
#    contours, _ = cv2.findContours(global_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#    cv2.drawContours(frame, contours, -1, (255, 150, 100), 2)
#    dx = width_min
#    max_y = 0

#    try:
#        while mask[max_y].sum() == 0:
#            max_y += 1
#    except:
#        mask = global_mask[height_min:height_max, :]
#        dx = 0
#        max_y = 0
#        try:
#            while mask[max_y].sum() == 0:
#                max_y += 1
#        except:
#            mask = global_mask
#            try:
#                while mask[max_y].sum() == 0:
#                    max_y += 1
#            except:
#                return frame, None, None

#    print max_y
#    cv2.imshow('mask', global_mask)
#    nearest_x = find_nearest(np.where(mask[max_y] == 255)[0], center[0])

#    try:
#        yaw = math.atan2(max_y - center[1], dx + nearest_x - center[0]) + pi_na_dva
#    except:
#        yaw = None

#    no_filtered = yaw
#    yaw = kalman_easy(yaw)
#    yaw = yaw / 100
#    print("yaw on tik: norm -- {}, filtered -- {}".format(no_filtered, yaw))

#    long_mask = global_mask[:, width_min:width_max]
#    y_i = 0
#    _dx = width_min

#    while True:
#        try:
#            if long_mask[center[1] + y_i].sum() != 0:
#                break
#            elif long_mask[center[1] - y_i].sum() != 0:
#                y_i = -y_i
#                break
#            else:
#                y_i += 1
#        except:
#            long_mask = global_mask
#            y_i = 0
#            _dx = 0

#    x_i = find_nearest(np.where(long_mask[center[1] + y_i] == 255)[0], center[0])
#    cv2.circle(frame, (x_i + _dx, center[1] + y_i), 4, (0, 0, 255), cv2.FILLED)

#    cv2.circle(frame, (dx + nearest_x, max_y), 5, (0, 255, 255), cv2.FILLED)
#    cv2.line(frame, center, (dx + nearest_x, max_y), (0, 255, 255), 3)

#    cv2.imshow('mask', global_mask)
#    return frame, yaw, (x_i + dx - center[0])


def get_better_yaw(frame):
    global last_yaw_point, last_yaw, last_y

    if enable_grayscale:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if enable_mask_filter:
        if enable_grayscale:
            global_mask = get_mask(gray)
        else:
            global_mask = get_mask(frame)

        global_mask, contours = get_filtered_area(global_mask)
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)
    else:
        if enable_grayscale:
            global_mask = get_mask(gray)
        else:
            global_mask = get_mask(frame)


    global_mask = cv2.erode(global_mask, kernel, iterations=1)
    global_mask = cv2.dilate(global_mask, kernel, iterations=1)

    try:
        yaw_coord_from_center = find_nearest_white(global_mask, (center[0], 0))[0]
        yaw_coords = find_nearest_white(global_mask, last_yaw_point)
    except TypeError:
        if mask_debug:
            return frame, global_mask, 0, 0
        else:
            return frame, 0, 0
    except NameError:
        if mask_debug:
            return frame, global_mask, last_yaw, last_y
        else:
            return frame, last_yaw, last_y

    yaw_coord = yaw_coords[0]

    for y in yaw_coords:
        if y[1] > yaw_coord[1]:
            yaw_coord = y

    yaw_coord = (yaw_coord + yaw_coord_from_center) // 2
    last_yaw_point = yaw_coord

    try:
        y_coords = find_nearest_white(global_mask, center)[0]
    except NameError:
        if mask_debug:
            return frame, global_mask, last_yaw, last_y
        else:
            return frame, last_yaw, last_y

    yaw = math.atan2(yaw_coord[1] - center[1], yaw_coord[0] - center[0]) + pi_na_dva

    if yaw > math.pi:
        yaw -= 2 * math.pi

    if enable_kalman_filter:
        yaw = kalman_easy(yaw) / 100

    cv2.circle(frame, tuple(yaw_coord), 5, (255, 0, 255), cv2.FILLED)
    cv2.circle(frame, tuple(y_coords), 5, (0, 255, 255), cv2.FILLED)
    cv2.line(frame, center, tuple(y_coords), (0, 255, 255), 3)
    cv2.line(frame, center, (int(center[0] + math.tan(yaw) * center[1]), 0), (255, 0, 255), 3)

    last_yaw, last_y = yaw, y_coords[0] - center[0]

    if mask_debug:
        return frame, global_mask, yaw, y_coords[0] - center[0]
    else:
        return frame, yaw, y_coords[0] - center[0]


if __name__ == '__main__':
    cap = cv2.VideoCapture('assets/untitled.mp4')
    out = cv2.VideoWriter('output/output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 60.0, (width, height))

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (width, height))
            t = time()
            if mask_debug:
                frame, mask, yaw, y = get_better_yaw(frame)
                cv2.imshow('global_mask', mask)
            else:
                frame, yaw, y = get_better_yaw(frame)
            cv2.imshow('frame', frame)
            average_time_dict['t'] += time() - t
            average_time_dict['s'] += 1
            out.write(frame)
            # sleep(0.05)
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

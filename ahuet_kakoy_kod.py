import numpy as np
import cv2
import math
from time import time, sleep

lower = [0, 0, 0]
higher = [50] * 3
lower_rgb = np.array(lower)
higher_rgb = np.array(higher)
size = width, height = 320, 240
center = width // 2, height // 2
width_min, width_max = 100, 220
height_min, height_max = 0, 150
pi_na_dva = math.pi / 2

max_velocity = 1

average_time_dict = {
    't': 0,
    's': 0
}


def get_velocity(_yaw):
    global max_velocity
    return max_velocity * math.cos(_yaw)


def find_nearest(array, value):
    array = np.asarray(array)
    idx = np.abs(array - value).argmin()
    return array[idx]


def find_nearest_white(img, target):
    nonzero = cv2.findNonZero(img)
    distances = np.sqrt((nonzero[:, :, 0] - target[0]) ** 2 + (nonzero[:, :, 1] - target[1]) ** 2)
    nearest_index = np.argmin(distances)
    return nonzero[nearest_index]


def get_yaw(frame, draw_rects=False, draw_contours=False, show_mask=False):
    global_mask = cv2.inRange(frame, lower_rgb, higher_rgb)

    vertical_size = height // 30
    vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, vertical_size))
    global_mask = cv2.erode(global_mask, vertical_structure)
    global_mask = cv2.dilate(global_mask, vertical_structure)

    mask = global_mask[height_min:height_max, width_min:width_max]
    dx = width_min
    max_y = 0

    if mask.sum() == 0:
        mask = global_mask[height_min:height_max, :]
        dx = 0
    if mask.sum() == 0:
        mask = global_mask

    try:
        while mask[max_y].sum() == 0:
            max_y += 1
    except:
        return frame, None, None

    nearest_x = find_nearest(np.where(mask[max_y] == 255)[0], center[0])

    try:
        yaw = math.atan2(max_y - center[1], dx + nearest_x - center[0]) + pi_na_dva
        if yaw > math.pi:
            yaw -= math.pi * 2
    except:
        yaw = None

    nearest_y = tuple(find_nearest_white(global_mask, center)[0])

    if draw_contours or draw_rects:
        contours, _ = cv2.findContours(global_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if draw_contours:
            cv2.drawContours(frame, contours, -1, (0, 150, 255), 1)
        if draw_rects:
            for contour in contours:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 1)

    cv2.circle(frame, nearest_y, 3, (255, 0, 255), cv2.FILLED)
    cv2.circle(frame, (dx + nearest_x, max_y), 4, (0, 255, 255), cv2.FILLED)
    cv2.line(frame, center, (dx + nearest_x, max_y), (0, 255, 255), 2)
    if show_mask:
        cv2.imshow('mask', mask)
    return frame, yaw, nearest_y[0] - center[0]


if __name__ == '__main__':
    cap = cv2.VideoCapture('assets/cool_vid2.mp4')
    out = cv2.VideoWriter('output/output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 60.0, (width, height))
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (width, height))
            t = time()
            frame, yaw, y = get_yaw(frame)
            average_time_dict['t'] += time() - t
            average_time_dict['s'] += 1
            cv2.imshow('frame', frame)
            out.write(frame)
        else:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.waitKey(1) & 0xFF == ord('p'):
            while cv2.waitKey(1) & 0xFF != ord('c'):
                sleep(1)
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(average_time_dict['s'] / average_time_dict['t'], 'Hz')
    print(average_time_dict['t'] / average_time_dict['s'], 'SPF')

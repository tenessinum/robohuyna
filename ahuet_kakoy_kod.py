import numpy as np
import cv2
import math

lower = [0, 0, 0]
higher = [40, 40, 40]
lower_rgb = np.array(lower)
higher_rgb = np.array(higher)
size = width, height = 320, 240
center = width // 2, height // 2
width_min, width_max = 100, 220
height_min, height_max = 0, 150
pi_na_dva = math.pi / 2

max_velocity = 1


def get_velocity(yaw):
    global max_velocity
    return max_velocity * (1 - abs(yaw) / math.pi)


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]


def get_yaw(frame):
    global_mask = cv2.inRange(frame, lower_rgb, higher_rgb)
    mask = global_mask[height_min:height_max, width_min:width_max]
    # contours, _ = cv2.findContours(global_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(frame, contours, -1, (255, 150, 100), 2)
    dx = width_min
    max_y = 0

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

    # cv2.imshow('mask', global_mask)
    nearest_x = find_nearest(np.where(mask[max_y] == 255)[0], center[0])

    try:
        yaw = math.atan2(max_y - center[1], dx + nearest_x - center[0]) + pi_na_dva
    except:
        yaw = None

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

    try:
        x_i = find_nearest(np.where(long_mask[center[1] + y_i] == 255)[0], center[0])
        cv2.circle(frame, (x_i + _dx, center[1] + y_i), 4, (0, 0, 255), cv2.FILLED)
    except:
        pass

    cv2.circle(frame, (dx + nearest_x, max_y), 5, (0, 255, 255), cv2.FILLED)
    cv2.line(frame, center, (dx + nearest_x, max_y), (0, 255, 255), 3)

    return frame, yaw,  (x_i + dx - center[0])


if __name__ == '__main__':
    cap = cv2.VideoCapture('assets/cool_vid.mp4')
    out = cv2.VideoWriter('output/output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 60.0, (width, height))
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (width, height))
            frame, yaw = get_yaw(frame)
            cv2.imshow('frame', frame)
            out.write(frame)
        else:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

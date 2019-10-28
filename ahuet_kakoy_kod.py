import numpy as np
import cv2
import math

lower = [0, 0, 0]
higher = [50, 50, 50]
lower_rgb = np.array(lower)
higher_rgb = np.array(higher)
size = width, height = 320, 240
center = width // 2, height // 2
width_min, width_max = 100, 220
height_min, height_max = 0, 150
pi_na_dva = math.pi / 2


def get_yaw(frame):
    mask = cv2.inRange(frame[height_min:height_max, width_min:width_max], lower_rgb, higher_rgb)
    dx = width_min
    max_y = 0
    try:
        while mask[max_y].sum() == 0:
            max_y += 1
    except:
        mask = cv2.inRange(frame[height_min:height_max, :], lower_rgb, higher_rgb)
        dx = 0
        max_y = 0
        while mask[max_y].sum() == 0:
            max_y += 1

    cv2.imshow('mask', mask)

    cv2.circle(frame, (dx + int(np.where(mask[max_y] == 255)[0].mean()), max_y), 6, (0, 255, 255), cv2.FILLED)
    cv2.line(frame, center, (dx + int(np.where(mask[max_y] == 255)[0].mean()), max_y), (0, 255, 255), 3)

    try:
        yaw = math.degrees(
            math.atan2(max_y - center[1], (dx + int(np.where(mask[max_y] == 255)[0].mean()) - center[0])) + pi_na_dva)
    except:
        yaw = None

    '''found = False
    radius = 0
    mask = cv2.inRange(frame, lower_rgb, higher_rgb)
    while not found:
        radius += 1
        for _x in range(-radius, radius, 2):
            _y = int((radius ** 2 - _x ** 2) ** 0.5)
            if mask[center[1] - _y][center[0] + _x] != 0:
                # cv2.line(frame, center, (center[0] + _x, center[1] - _y), (0, 255, 255))
                # cv2.circle(frame, (center[0] + _x, center[1] - _y), 6, (0, 0, 255), cv2.FILLED)
                cv2.line(frame, center, (center[0] + _x, center[1]), (255, 255, 0), 3)
                found = True
                radius = _x
                break'''

    return frame, yaw  # , radius


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

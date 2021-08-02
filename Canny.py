# import cv2
# import numpy as np
#
# image = cv2.imread('simulator.png')
#
# lane_image = np.copy(image)
#
#
# def canny(image):
#     gray = cv2.cvtColor(lane_image, cv2.COLOR_BGR2GRAY)
#     blur = cv2.GaussianBlur(gray, (13, 13), 0)
#     canny = cv2.Canny(blur, 50, 150)
#     return canny
#
#
# def display_lines(image, lines):
#     line_image = np.zeros_like(image)
#     if lines is not None:
#         for line in lines:
#             x1, y1, x2, y2 = line.reshape(4)
#
#             cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 10)
#     return line_image
#
#
# def region_of_interst(image):
#     height = image.shape[0]
#     polygons = np.array([
#         [(200, height), (1100, height), (550, 250)]
#     ])
#     mask = np.zeros_like(image)
#     cv2.fillPoly(mask, polygons, 255)
#     masked_image = cv2.bitwise_and(image, mask)
#     return masked_image
#
#
# # HSVlow = (181, 255, 178)
# HSVlow = (144, 149, 0)
# HSVHigh = (179, 0, 255)
# # HSVHigh = (255, 255, 255)
# ROI = region_of_interst(image)
#
# filter_mask_White = cv2.inRange(ROI, HSVlow, HSVHigh)
# contours_White, hierarchy_white = cv2.findContours(filter_mask_White.copy(), cv2.RETR_EXTERNAL,
#                                                    cv2.CHAIN_APPROX_SIMPLE)
#
# if len(contours_White) > 0:
#     c_white = max(contours_White, key=cv2.contourArea)
#     M_white = cv2.moments(c_white)
#
#     if M_white['m00'] != 0:
#         cx_white = int(M_white['m10'] / M_white['m00'])
#         cy_white = int(M_white['m01'] / M_white['m00'])
#     else:
#         cx_white = 0
#         cy_white = 0
# a = cv2.drawContours(ROI, contours_White, -1, (255, 0, 0), 3)
#
# canny = canny(lane_image)
# lane_image = np.copy(image)
# cropped_image = region_of_interst(canny)
# lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 290, 100, np.array([]), minLineLength=40, maxLineGap=500)
# line_image = display_lines(lane_image, lines)
# combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
# cv2.imshow("canny", ROI)
# cv2.imshow('result', combo_image)
# cv2.waitKey(0)
import time

import cv2
import numpy as np

import AVISEngine

# Calling the class
car = AVISEngine.car()

# connecting to the server (Simulator)
car.connect("127.0.0.1", 25001)

time.sleep(3)

performance_mode = False
is_avoiding_obstacle = False
HSVlow = (0, 135, 0)
HSVHigh = (126, 255, 255)

HSVlow_white = (181, 255, 178)
HSVHigh_white = (255, 255, 255)

counter = 0

temp_vehicle_offset = 0


def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    print(rightMin + (valueScaled * rightSpan), "  t   ra  ns  la   te  ")
    return rightMin + (valueScaled * rightSpan)


cx_white_left_line = 0
cx_white_right_line = 0


def obs():
    global is_avoiding_obstacle
    if sensors[1] < 1300 or sensors[2] < 800:

        is_avoiding_obstacle = True


    else:
        is_avoiding_obstacle = False

    # if sensors[1] < 1499 or sensors[2] < 1499:
    #     # is_avoiding_obstacle = True
    #     pass
    # TODO: avoiding obstacle part in obstacle def
    if (sensors[1] <= 1200 and sensors[1] >= 1000) or sensors[2] <= 900:
        if cx_white_right_line < 190:
            is_avoiding_obstacle = True
            steering_value = -90
            car.setSteering(steering_value)
            time.sleep(0.02)

    if sensors[1] <= 950:
        if cx_white_right_line < 190:
            is_avoiding_obstacle = True
            steering_value = -100
            car.setSteering(steering_value)
            time.sleep(0.01)

    if cx_white_left_line > 150:
        is_avoiding_obstacle = True
        # car.setSpeed(50)
        steering_value = 80
        if ((sensors[1] <= 1200 and sensors[1] >= 1000) or sensors[2] <= 900) and carSpeed <= 20:
            time.sleep(0.1)
            car.setSteering(steering_value)

        else:
            car.setSteering(steering_value)

    if cx_white_right_line < 0:
        is_avoiding_obstacle = True
        car.setSteering(-70)
        time.sleep(0.01)

    if cx_white_left_line >= 210:
        is_avoiding_obstacle = True
        car.setSteering(70)
        time.sleep(0.05)


def speed_adjustment(V_off):
    V_off = V_off - 120
    # TODO : speed adj
    adjusted_speed = translate(abs(V_off), 0, 70, 30, 100)
    return adjusted_speed


def canny(hsvImage):
    can = cv2.Canny(hsvImage, 100, 200)
    # bitwise = cv2.bitwise_and(can, hsvImage, filter_mask_white_left_line)
    hogh = cv2.HoughLinesP(can, 10, np.pi / 180, 100, (0, 0, 255), maxLineGap=250)
    print(hogh, "+++++++")
    # cv2.imshow('bit', bitwise)
    return hogh


Sline = 0


def display_lines(image, lines):
    global Sline

    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            lineT = ((x1 + x2) / (y1 + y2))
            Sline = lineT
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            print(lineT, "-----------------")
    return line_image


try:
    while True:
        counter = counter + 1
        # car.setSpeed(100)
        # car.setSteering(0)

        car.getData()
        if counter > 8:
            frame = car.getImage()
            sensors = car.getSensors()
            carSpeed = car.getSpeed()
            # cv2.imread(frame)
            right_line_region = frame[120:200, 150:200]
            left_line_region = frame[120:200, 50:100]

            cv2.imshow("real_time_image1", right_line_region)
            cv2.imshow("real_time_image2", left_line_region)

            filter_mask_white_right_line = cv2.inRange(right_line_region, HSVlow_white, HSVHigh_white)
            filter_mask_white_left_line = cv2.inRange(left_line_region, HSVlow_white, HSVHigh_white)

            contours_White_right_line, hierarchy_white_right_line = cv2.findContours(
                filter_mask_white_right_line.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)

            contours_White_left_line, hierarchy_white_left_line = cv2.findContours(filter_mask_white_left_line.copy(),
                                                                                   cv2.RETR_EXTERNAL,
                                                                                   cv2.CHAIN_APPROX_SIMPLE)

            if len(contours_White_right_line) > 0:
                c_white_right_line = max(contours_White_right_line, key=cv2.contourArea)
                M_white_right_line = cv2.moments(c_white_right_line)

                if M_white_right_line['m00'] != 0:
                    cx_white_right_line = int(M_white_right_line['m10'] / M_white_right_line['m00'])
                    cy_white_right_line = int(M_white_right_line['m01'] / M_white_right_line['m00'])
                else:
                    cx_white_right_line = 0
                    cy_white_right_line = 0

            if len(contours_White_left_line) > 0:
                c_white_left_line = max(contours_White_left_line, key=cv2.contourArea)
                M_white_left_line = cv2.moments(c_white_left_line)

                if M_white_left_line['m00'] != 0:
                    cx_white_left_line = int(M_white_left_line['m10'] / M_white_left_line['m00'])
                    cy_white_left_line = int(M_white_left_line['m01'] / M_white_left_line['m00'])
                else:
                    cx_white_left_line = 0
                    cy_white_left_line = 0

                if not performance_mode:
                    cv2.drawContours(right_line_region, contours_White_right_line, -1, (0, 0, 255), 3)
                    cv2.drawContours(left_line_region, contours_White_left_line, -1, (0, 255, 0), 3)

            C_left = cx_white_left_line + 200
            print(Sline, "ssssssssssssssss")
            vehicle_offset = (cx_white_right_line + C_left) / 2
            print(cx_white_right_line, "cx_white_right_line")
            # vehicle_offset = 0

            # print(f"{cx_white_right_line} right ")
            # print(f"{cx_white_left_line}  left")
            print(f"{vehicle_offset}  vehicle_offset")
            # cx = vehicle_offset #ch
            if not performance_mode:
                cv2.circle(frame, (int(vehicle_offset), 25), 10, (255, 0, 0), 2)

            # delta_vehicle_offset = vehicle_offset - temp_vehicle_offset

            # temp_vehicle_offset = vehicle_offset
            if is_avoiding_obstacle == False:
                # TODO : final steering value
                steering_value = translate(vehicle_offset, 100, 155, -45, 45)

                car.setSteering(steering_value)

            # print(sensors[1])
            # obs()
            canny0 = canny(filter_mask_white_left_line)
            canny1 = canny(filter_mask_white_right_line)
            canny01 = canny(frame[100:200,50:200])

            cv2.imshow('main_frame',frame[100:200,50:200])

            line_image = display_lines(frame, canny0)
            line_image_1 = display_lines(frame, canny1)
            line_image01 = display_lines(frame , canny01)
            cv2.imshow('hsvTOcanny', line_image)
            cv2.imshow('hsvTOcannyRirht', line_image_1)
            cv2.imshow('hsvTOcannyfull', line_image01)

            # print("Status ", is_avoiding_obstacle)

            vehicle_final_speed = speed_adjustment(vehicle_offset)

            # print("Final Speed           ", vehicle_final_speed)
            # TODO : final car speed
            car.setSpeed(vehicle_final_speed)

            if not performance_mode:
                cv2.imshow('main', frame)
        else:
            car.setSpeed(10)
        # CHANGED
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    print('done')
    car.stop()

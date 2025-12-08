#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

IMG_PATH = "traffic_frame.png"

img_bgr = cv2.imread(IMG_PATH)
if img_bgr is None:
    raise RuntimeError(f"Failed to load image: {IMG_PATH}")

img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

win_name = "inspect_hsv"
cv2.namedWindow(win_name)

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        b, g, r = img_bgr[y, x]
        h, s, v = img_hsv[y, x]
        # 打印当前像素的 BGR + HSV
        print(f"(x={x}, y={y})  BGR=({b}, {g}, {r})  HSV=({h}, {s}, {v})")

cv2.setMouseCallback(win_name, on_mouse)

while True:
    cv2.imshow(win_name, img_bgr)
    key = cv2.waitKey(20) & 0xFF
    if key == 27:  # ESC 退出
        break

cv2.destroyAllWindows()

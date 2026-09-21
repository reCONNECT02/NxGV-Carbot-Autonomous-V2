#!/usr/bin/env python3
"""Calibration images for hb_mapper: N training images -> 1x3x640x640 uint8 RGB
CHW .bin files (same format as the base repo's Colab cell 7).

    python make_calibration.py datasets/unified14/images/train calibration_data --count 100
Use images from OUR cameras at the venue lighting if possible, all classes.
"""
import argparse
import glob
import os
import random

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('images')
    ap.add_argument('out')
    ap.add_argument('--count', type=int, default=100)
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(a.images, '*')))
    random.Random(42).shuffle(files)
    os.makedirs(a.out, exist_ok=True)
    n = 0
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        rgb = cv2.cvtColor(cv2.resize(img, (640, 640)), cv2.COLOR_BGR2RGB)
        np.transpose(rgb, (2, 0, 1)).astype(np.uint8).tofile(os.path.join(a.out, f'calib_{n:03d}.bin'))
        n += 1
        if n >= a.count:
            break
    print(f'{n} calibration files in {a.out}')


if __name__ == '__main__':
    main()

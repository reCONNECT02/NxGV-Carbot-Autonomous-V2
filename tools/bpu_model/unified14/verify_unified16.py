#!/usr/bin/env python3
"""Smoke-test the unified14 YOLO11n model on an RDK X5 board without ROS."""

import argparse
import time

import cv2
import numpy as np


NAMES = [
    'end_of_tunnel_sign', 'hill_sign', 'obstacle_sign', 'parallel_parking_sign',
    'perpendicular_parking_sign', 'roundabout_sign', 'speed_bump_sign',
    'traffic_signals_ahead_sign', 'tunnel_sign', 'traffic_red',
    'traffic_yellow', 'traffic_green', 'boom_closed', 'boom_open',
]
STRIDES = (8, 16, 32)


def bgr_to_nv12(image):
    yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420)
    y, u, v = yuv[0:640, :], yuv[640:800, :], yuv[800:960, :]
    uv = np.stack([u.ravel(), v.ravel()], axis=1).ravel().reshape(320, 640)
    return np.vstack((y, uv))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--image')
    parser.add_argument('--confidence', type=float, default=0.25)
    args = parser.parse_args()

    from hbm_runtime import HB_HBMRuntime

    started = time.time()
    runtime = HB_HBMRuntime(args.model)
    print(f'load OK in {(time.time() - started) * 1000:.0f} ms')
    model_name = runtime.model_names[0]
    print('inputs:', {key: value for key, value in zip(runtime.input_names[model_name], runtime.input_shapes[model_name])})
    print('outputs:', len(runtime.output_names[model_name]))

    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            raise RuntimeError(f'Could not read {args.image}')
        input_data = bgr_to_nv12(cv2.resize(image, (640, 640)))
    else:
        input_data = np.zeros((960, 640), np.uint8)

    input_name = list(runtime.input_names[model_name])[0]
    started = time.time()
    output_map = runtime.run({model_name: {input_name: input_data}})[model_name]
    print(f'forward {(time.time() - started) * 1000:.0f} ms')

    output_names = list(runtime.output_names[model_name])
    if len(output_names) != 6:
        raise RuntimeError(f'Expected six outputs, found {len(output_names)}')
    for level, stride in enumerate(STRIDES):
        logits = np.asarray(output_map[output_names[level * 2]], dtype=np.float32).squeeze()
        if logits.shape[-1] != len(NAMES) and logits.shape[0] == len(NAMES):
            logits = np.transpose(logits, (1, 2, 0))
        scores = 1.0 / (1.0 + np.exp(-np.clip(logits, -30, 30)))
        flat = scores.reshape(-1, len(NAMES))
        location = int(flat.max(axis=1).argmax())
        class_id = int(flat[location].argmax())
        score = float(flat[location, class_id])
        label = NAMES[class_id] if score >= args.confidence else '-'
        print(f'stride {stride}: best {score:.3f} class={label}')
    print('SMOKE PASS: unified14 model ran end-to-end on the BPU')


if __name__ == '__main__':
    main()

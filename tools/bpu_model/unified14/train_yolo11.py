#!/usr/bin/env python3
"""Train YOLO11n on the unified14 dataset (GPU: Colab T4 or a PC).

    pip install ultralytics==8.3.*        # the version family the team model came from
    python train_yolo11.py --data dataset.yaml --epochs 150
    # optional Roboflow download (key from the ENVIRONMENT, never in a file):
    ROBOFLOW_API_KEY=... python train_yolo11.py --roboflow WORKSPACE/PROJECT/VERSION

Output: runs/detect/unified14/weights/best.pt -> export_rdk_onnx.py.
"""
import argparse
import os


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', default='dataset.yaml')
    ap.add_argument('--roboflow', default='', help='WORKSPACE/PROJECT/VERSION to download (YOLOv8 format)')
    ap.add_argument('--epochs', type=int, default=150)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--weights', default='yolo11n.pt')
    a = ap.parse_args()
    data = a.data
    if a.roboflow:
        key = os.environ.get('ROBOFLOW_API_KEY', '')
        if not key:
            raise SystemExit('set ROBOFLOW_API_KEY in the environment (never commit it)')
        from roboflow import Roboflow
        ws, proj, ver = a.roboflow.split('/')
        ds = Roboflow(api_key=key).workspace(ws).project(proj).version(int(ver)).download('yolov8')
        data = os.path.join(ds.location, 'data.yaml')
        print(f'downloaded to {ds.location}: CHECK its class order against dataset.yaml')
    from ultralytics import YOLO
    YOLO(a.weights).train(data=data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, patience=30,
                          name='unified14', exist_ok=True)


if __name__ == '__main__':
    main()

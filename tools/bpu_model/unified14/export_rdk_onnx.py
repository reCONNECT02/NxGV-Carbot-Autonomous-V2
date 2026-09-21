#!/usr/bin/env python3
"""best.pt -> best_rdk.onnx with the 6-output head bpu_detector decodes.

The stock Ultralytics export fuses the head (concat + DFL + sigmoid) into one
output. For the RDK X5 BPU the head is cut before that (D-Robotics RDK Model
Zoo method): per stride 8 / 16 / 32 the raw class logits (cv3) and the raw DFL
box regressions (cv2), NHWC. Sigmoid, DFL and NMS run on the CPU in
carbot_detectors/detector_core.py. Output order: cls8, box8, cls16, box16,
cls32, box32 (bpu_detector identifies them by shape anyway).

    python export_rdk_onnx.py runs/detect/unified14/weights/best.pt
NOT run in CI: check the printed output shapes (14 and 64 channels, grids
80 / 40 / 20), then compile with compile_model.sh and verify on the board.
"""
import sys


def main(pt: str):
    from ultralytics import YOLO
    from ultralytics.nn.modules.head import Detect

    def forward(self, x):
        out = []
        for i in range(self.nl):
            out.append(self.cv3[i](x[i]).permute(0, 2, 3, 1).contiguous())   # class logits
            out.append(self.cv2[i](x[i]).permute(0, 2, 3, 1).contiguous())   # DFL box
        return out

    Detect.forward = forward
    model = YOLO(pt)
    path = model.export(format='onnx', imgsz=640, opset=11, simplify=True)
    import onnx
    m = onnx.load(path)
    for o in m.graph.output:
        print(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim])
    target = path.replace('.onnx', '_rdk.onnx')
    onnx.save(m, target)
    print('wrote', target)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'best.pt')

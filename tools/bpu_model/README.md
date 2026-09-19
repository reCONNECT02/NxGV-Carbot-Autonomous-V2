# RISAbot YOLOv5s BPU Model Training & Compilation Guide

This guide covers training your object detection model on Google Colab, compiling it for the RDK X5 BPU on Windows, and launching the ROS2 inference node.

---

## 📅 Step 1: Google Colab Training
1. Open Google Colab and set the runtime to **GPU (T4)**.
2. Open the script [colab_training_script.py](file:///c:/Users/Victus/RISAbot/risabotcar_ws/tools/bpu_model/colab_training_script.py).
3. Copy the code from **Cell 1 to Cell 8** into separate code cells in your Colab notebook.
4. Replace `"YOUR_ROBOFLOW_API_KEY_HERE"` in Cell 2 with your actual Roboflow API key.
5. Run the cells in order.
6. Download the generated `/content/bpu_package.zip` package when the notebook completes.

---

## ⚙️ Step 2: BPU Compilation on Windows
1. Unzip the downloaded `bpu_package.zip` file.
2. Place the following files directly inside this folder (`tools/bpu_model/`):
   *   `best.onnx`
   *   `calibration_data/` (the folder containing the 50 `.bin` images)
3. Ensure **Docker Desktop** is running on your Windows PC.
4. Double-click [compile_model.bat](file:///c:/Users/Victus/RISAbot/risabotcar_ws/tools/bpu_model/compile_model.bat).
5. Once complete, your compiled model will be created at:
   *   `tools/bpu_model/model_output/risabot_signs_640x640_nv12.bin`

---

## 🚀 Step 3: Deployment to RDK X5
1. Secure copy (SCP) or transfer the compiled `.bin` model to the RDK X5 board:
   ```bash
   scp tools/bpu_model/model_output/risabot_signs_640x640_nv12.bin sunrise@<RDK_IP>:/home/sunrise/
   ```
2. Transfer your updated ROS2 workspace code to the RDK X5 board.
3. On the RDK X5, navigate to the workspace and build:
   ```bash
   cd ~/risabotcar_ws
   colcon build --packages-select risabot_automode
   source install/setup.bash
   ```

---

## 🏃 Step 4: Execution & Verification
1. Launch the system:
   ```bash
   ros2 launch risabot_automode bringup.launch.py
   ```
2. Verify that the node starts up and loads the BPU model.
3. You can enable debug visuals by editing `params.yaml` or setting the parameter at runtime:
   ```bash
   ros2 param set /signage_detector show_debug true
   ```
4. View the debug image stream on the dashboard at `http://<RDK_IP>:8080` or subscribe to `/camera/debug/signage`.

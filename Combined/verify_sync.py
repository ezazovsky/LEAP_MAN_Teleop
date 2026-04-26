import argparse
import os
import sys

try:
    import h5py
    import numpy as np
    import cv2
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Please run: pip install h5py numpy opencv-python matplotlib")
    sys.exit(1)


def visualize(hdf5_path, playback_speed=1.0):
    if not os.path.exists(hdf5_path):
        print(f"\n[ERROR] File {hdf5_path} not found.")
        return

    print(f"\nLoading data from: {hdf5_path}...")
    
    try:
        f = h5py.File(hdf5_path, 'r')
    except Exception as e:
        print(f"[ERROR] Could not open HDF5 file. Was it closed properly? {e}")
        return

    if 'camera/color' not in f or 'arm/smoothed_pose' not in f:
        print("[ERROR] Missing required datasets in HDF5 file. Ensure camera was enabled.")
        f.close()
        return

    # Load telemetry into memory (Camera frames stay on disk to save RAM)
    times = f['time/monotonic_s'][:]
    poses = f['arm/smoothed_pose'][:]  # [X, Y, Z, Rx, Ry, Rz]
    colors = f['camera/color']
    
    num_samples = len(times)
    t_rel = times - times[0]  # Start at 0 seconds

    print(f"Loaded {num_samples} samples ({t_rel[-1]:.2f} seconds of data).")
    print("Generating trajectory graph...")

    # --- 1. Create the base trajectory plot ---
    fig, ax = plt.subplots(figsize=(6, 4.8), dpi=100)
    fig.patch.set_facecolor('#f0f0f0')
    
    ax.plot(t_rel, poses[:, 0], label='X Position', color='#d62728', linewidth=2)
    ax.plot(t_rel, poses[:, 1], label='Y Position', color='#2ca02c', linewidth=2)
    ax.plot(t_rel, poses[:, 2], label='Z Position', color='#1f77b4', linewidth=2)
    
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('End-Effector Position (m)')
    ax.set_title('Robot Trajectory & Video Sync Verification')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend(loc='upper right')
    
    fig.canvas.draw()
    
    # Convert plot to a static OpenCV image
    base_plot_img = np.array(fig.canvas.buffer_rgba())
    base_plot_img = cv2.cvtColor(base_plot_img, cv2.COLOR_RGBA2BGR)
    plot_h, plot_w = base_plot_img.shape[:2]

    # --- 2. Playback Loop ---
    print("\n[PLAYBACK STARTED]")
    print("Controls: [Space] to Pause, [Q] to Quit, [Left/Right] arrows to scrub when paused.")

    i = 0
    is_paused = False
    
    while i < num_samples:
        if i < 0: i = 0
        
        # 1. Grab the camera frame
        frame = colors[i].copy() 
        cam_h, cam_w = frame.shape[:2]

        # 2. Resize plot image to exactly match the camera height for a seamless side-by-side
        if plot_h != cam_h:
            scale = cam_h / plot_h
            base_plot_img = cv2.resize(base_plot_img, (int(plot_w * scale), cam_h))
            plot_w = base_plot_img.shape[1]

        # 3. Calculate cursor X-position dynamically based on time
        # We use matplotlib's coordinate transform to find exactly where the line belongs
        cursor_x_float = ax.transData.transform((t_rel[i], 0))[0]
        # Adjust for any resizing we just did
        cursor_x = int(cursor_x_float * (cam_h / plot_h)) 
        
        # 4. Draw the playhead cursor on a fresh copy of the base plot
        current_plot = base_plot_img.copy()
        cv2.line(current_plot, (cursor_x, 0), (cursor_x, cam_h), (0, 0, 0), 2)
        cv2.circle(current_plot, (cursor_x, int(cam_h/2)), 4, (0, 0, 255), -1)

        # 5. Overlay text stats on the video feed
        cv2.putText(frame, f"Time: {t_rel[i]:.2f}s", (15, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Idx: {i}/{num_samples}", (15, 75), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Combine Video + Graph
        combined_view = np.hstack((frame, current_plot))
        cv2.imshow("Data Sync Verifier", combined_view)

        # --- Playback Logic & Controls ---
        if not is_paused:
            # Calculate dynamic delay to match original recording speed
            dt = t_rel[i+1] - t_rel[i] if i < num_samples - 1 else 0.008
            delay_ms = max(1, int((dt / playback_speed) * 1000))
            key = cv2.waitKey(delay_ms) & 0xFF
            i += 1
        else:
            key = cv2.waitKey(0) & 0xFF

        # Handle Keyboard Inputs
        if key == ord('q') or key == 27:  # Q or Esc
            break
        elif key == ord(' '):  # Spacebar toggles pause
            is_paused = not is_paused
        elif key == 81 or key == ord('a'):  # Left Arrow / A (Scrub back)
            i = max(0, i - 10)
        elif key == 83 or key == ord('d'):  # Right Arrow / D (Scrub forward)
            i = min(num_samples - 1, i + 10)

    f.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Verify synchronization of HDF5 Teleop Data.")
    parser.add_argument("hdf5_file", type=str, help="Path to the .hdf5 log file to visualize.")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (e.g., 2.0 for 2x speed).")
    args = parser.parse_args()

    visualize(args.hdf5_file, args.speed)
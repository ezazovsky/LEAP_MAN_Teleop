import sys
import time
import argparse
import numpy as np

from track import ViveTrackerModule
from render_argparse import get_render_args
from vive_visualizer import ViveTrackerViewer
from fairmotion_vis import camera
from fairmotion_ops import conversions, math as fairmotion_math


class ViveTrackerUpdater():
    def __init__(self):
        self.vive_tracker_module = ViveTrackerModule()
        self.vive_tracker_module.print_discovered_objects()

        self.fps = 30
        self.device_key = "tracker"
        self.tracking_devices = self.vive_tracker_module.return_selected_devices(self.device_key)
        self.tracking_result = []

        # Base station origin configuration
        self.base_station_origin = conversions.p2T(np.array([3.0, -2.8, -3.0]))
        self.origin_inv = fairmotion_math.invertT(self.base_station_origin)

    def update(self, verbose=True):
        self.tracking_result = []
        output_string = ""

        for key, device in self.tracking_devices.items():
            # Get the matrix and apply the origin offset
            T_matrix = self.origin_inv @ device.get_T()
            self.tracking_result.append(T_matrix)

            if verbose:
                # Extract the translation (X, Y, Z) from the 4x4 transformation matrix
                x, y, z = T_matrix[0, 3], T_matrix[1, 3], T_matrix[2, 3]
                
                # Format to the 4th decimal point
                output_string += f"[{key}] X: {x:.4f} Y: {y:.4f} Z: {z:.4f}   "

        if verbose and output_string:
            # ljust(80) ensures we overwrite residual characters from the previous frame
            # flush=True forces the terminal to output immediately
            print("\r" + output_string.ljust(80), end="", flush=True)

        # The viewer needs the updated matrices to render the 3D objects
        return self.tracking_result


def main(args):
    cam = camera.Camera(
        pos=np.array(args.camera_position),
        origin=np.array(args.camera_origin),
        vup=np.array([0, 1, 0]),
        fov=45.0,
    )
    
    viewer = ViveTrackerViewer(
        v_track_updater=ViveTrackerUpdater(),
        play_speed=args.speed,
        scale=args.scale,
        thickness=args.thickness,
        render_overlay=args.render_overlay,
        hide_origin=args.hide_origin,
        title="Vive Viewer",
        cam=cam,
        size=(1920, 1280),
    )
    viewer.run()

if __name__ == "__main__":
    args = get_render_args().parse_args()
    main(args)
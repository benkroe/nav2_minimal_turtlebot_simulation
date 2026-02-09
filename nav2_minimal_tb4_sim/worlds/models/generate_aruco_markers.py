#!/usr/bin/env python3
"""
Generate ArUco marker PNG files for use in Gazebo simulation.
Saves all textures to the centralized textures/ directory.
"""

import cv2
import numpy as np
import os
from pathlib import Path

def generate_aruco_marker(marker_id, size=200, border_size=40, output_path=None):
    """
    Generate an ArUco marker with white quiet zone and save as PNG.
    
    Args:
        marker_id: ID of the marker (0-99 for DICT_4X4_100)
        size: Size of the inner marker in pixels
        border_size: Width of white border in pixels
        output_path: Path to save the PNG file
    
    Returns:
        numpy array of the final marker image
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_100
    )

    # Generate inner marker (black border included)
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)

    # Create white canvas
    final_size = size + 2 * border_size
    canvas = np.ones((final_size, final_size), dtype=np.uint8) * 255

    # Place marker in center
    canvas[
        border_size:border_size + size,
        border_size:border_size + size
    ] = marker

    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        cv2.imwrite(output_path, canvas)
        print(f"✓ Generated ArUco marker {marker_id} -> {output_path}")

    return canvas


def main():
    # Base path - textures go directly to textures/ folder
    base_path = Path(__file__).parent / "textures"
    
    # Define markers to generate
    # Marker ID -> output filename
    markers = {
        1: "aruco_mark_1.png",
        2: "aruco_mark_2.png",
        3: "aruco_mark_3.png",
        4: "aruco_mark_4.png",
    }
    
    print("Generating ArUco markers...")
    print("-" * 50)
    
    for marker_id, filename in markers.items():
        output_path = base_path / filename
        generate_aruco_marker(marker_id, size=256, output_path=str(output_path))
    
    print("-" * 50)
    print("Done! All markers generated successfully.")
    print(f"Textures saved to: {base_path}")


if __name__ == "__main__":
    main()


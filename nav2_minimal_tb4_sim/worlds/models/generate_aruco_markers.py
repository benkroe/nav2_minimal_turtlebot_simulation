#!/usr/bin/env python3
"""
Generate ArUco marker PNG files for use in Gazebo simulation.
Saves all textures to the centralized textures/ directory.
"""

import cv2
import numpy as np
import os
from pathlib import Path

def generate_aruco_marker(marker_id, size=200, output_path=None):
    """
    Generate an ArUco marker and save as PNG.
    
    Args:
        marker_id: ID of the marker (0-99 for DICT_5X5_100)
        size: Size of the generated marker in pixels (default 200)
        output_path: Path to save the PNG file
    
    Returns:
        numpy array of the marker image
    """
    # Use the 4x4 dictionary with 100 markers (DICT_4X4_100)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    
    # Generate the marker
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    
    # Save if output path provided
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        cv2.imwrite(output_path, marker_image)
        print(f"✓ Generated ArUco marker {marker_id} -> {output_path}")
    
    return marker_image


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


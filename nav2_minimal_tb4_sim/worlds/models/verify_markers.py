#!/usr/bin/env python3
"""
Verify which ArUco marker IDs are in the PNG texture files.
"""

import cv2
import numpy as np
from pathlib import Path

def check_marker_id(image_path):
    """Read PNG and detect what ArUco marker ID it contains."""
    try:
        img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None, "Failed to read image"
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict)
        corners, ids, rejected = aruco_detector.detectMarkers(img)
        
        if ids is not None and len(ids) > 0:
            return ids[0][0], "OK"
        else:
            return None, "No marker detected"
    except Exception as e:
        return None, str(e)

def main():
    base_path = Path(__file__).parent / "textures"
    
    print("=== ArUco Marker ID Verification ===")
    print()
    
    texture_files = sorted(base_path.glob("aruco_mark_*.png"))
    
    for texture_file in texture_files:
        marker_id, status = check_marker_id(texture_file)
        filename = texture_file.name
        
        if marker_id is not None:
            print(f"✓ {filename:20s} → Marker ID {marker_id} ({status})")
        else:
            print(f"✗ {filename:20s} → ERROR: {status}")
    
    print()
    print("Verify that your detection code searches for the correct marker ID!")

if __name__ == "__main__":
    main()

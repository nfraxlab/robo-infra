# Vision

Computer vision enables robots to perceive and interact with the world. This guide covers color detection, marker tracking, image processing, and camera calibration.

## Overview

The vision module provides:

- **Color Detection** - Find objects by color using HSV thresholding
- **Marker Detection** - ArUco, AprilTag, and QR code detection
- **Image Processing** - Edges, contours, transformations
- **Camera Utilities** - Calibration and coordinate transforms

```python
from robo_infra.vision import (
    ColorDetector,
    ArUcoDetector, AprilTagDetector,
    resize, crop, edge_detect, find_contours,
)
```

> **Note**: Vision features require OpenCV. Install with:
> ```bash
> pip install robo-infra[vision]
> ```

---

## Color Detection

### HSV Color Spaces

HSV (Hue, Saturation, Value) is more robust for color detection than RGB because it separates color (hue) from brightness (value).

```
H (Hue):        0-179 in OpenCV (0-360° mapped)
S (Saturation): 0-255 (gray to pure color)
V (Value):      0-255 (dark to bright)
```

| Color | Hue Range (OpenCV) |
|-------|-------------------|
| Red | 0-10 and 170-179 (wraps around) |
| Orange | 10-25 |
| Yellow | 25-35 |
| Green | 35-85 |
| Blue | 85-130 |
| Purple | 130-160 |

### ColorDetector

```python
from robo_infra.vision.color import ColorDetector, ColorRange

# Use preset detectors
red_detector = ColorDetector.red()
green_detector = ColorDetector.green()
blue_detector = ColorDetector.blue()
yellow_detector = ColorDetector.yellow()

# Custom color range
orange_detector = ColorDetector(
    lower_hsv=(5, 100, 100),
    upper_hsv=(25, 255, 255),
    name="orange",
)

# From ColorRange
custom = ColorDetector(
    color_range=ColorRange(
        lower=(100, 150, 50),
        upper=(130, 255, 255),
        name="blue",
    )
)
```

### Color Tracking

```python
from robo_infra.sensors.camera import USBCamera
from robo_infra.vision.color import ColorDetector

# Create detector for red objects
detector = ColorDetector.red()

with USBCamera() as camera:
    while True:
        frame = camera.capture()
        
        # Detect red blobs
        blobs = detector.detect(frame)
        
        if blobs:
            # Get largest blob
            largest = max(blobs, key=lambda b: b.area)
            print(f"Red object at {largest.center}, area={largest.area}")
            
            # Track with PanTilt
            pan_tilt.track(largest.center)
```

### ColorBlob Properties

```python
blobs = detector.detect(frame)

for blob in blobs:
    print(f"Center: {blob.center}")        # (x, y) pixel coordinates
    print(f"Area: {blob.area}")            # Area in pixels
    print(f"Bounding box: {blob.bbox}")    # (x, y, width, height)
    print(f"Aspect ratio: {blob.aspect_ratio}")
    print(f"Circularity: {blob.circularity}")  # 1.0 = perfect circle
    print(f"Contour: {blob.contour}")      # OpenCV contour points
```

### Object Isolation

Filter blobs by properties:

```python
# Detect all blobs
blobs = detector.detect(frame)

# Filter by minimum area
blobs = detector.detect(frame, min_area=500)

# Filter by maximum blobs
blobs = detector.detect(frame, max_blobs=5)

# Custom filtering
large_round_blobs = [
    b for b in blobs
    if b.area > 1000 and b.circularity > 0.7
]
```

### Color Calibration Tool

Find optimal HSV ranges for your lighting:

```python
from robo_infra.vision.color import ColorCalibrator

calibrator = ColorCalibrator()

with USBCamera() as camera:
    # Interactive calibration
    color_range = calibrator.calibrate(camera)
    
    print(f"Lower HSV: {color_range.lower}")
    print(f"Upper HSV: {color_range.upper}")
    
    # Save for later
    color_range.save("my_color.json")
    
    # Load saved range
    loaded = ColorRange.load("my_color.json")
```

---

## Marker Detection

Fiducial markers provide accurate identification and 6-DOF pose estimation.

### ArUco Markers

```python
from robo_infra.vision.markers import ArUcoDetector, ArUcoDictionary

# Create detector with dictionary
detector = ArUcoDetector(dictionary=ArUcoDictionary.DICT_4X4_50)

# Available dictionaries
# DICT_4X4_50     - 50 markers, 4x4 bits (small, simple)
# DICT_4X4_100    - 100 markers, 4x4 bits
# DICT_5X5_50     - 50 markers, 5x5 bits
# DICT_6X6_250    - 250 markers, 6x6 bits (robust)
# DICT_7X7_1000   - 1000 markers, 7x7 bits (most IDs)
# DICT_ARUCO_ORIGINAL - Original ArUco dictionary
```

### Detecting Markers

```python
with USBCamera() as camera:
    frame = camera.capture()
    
    # Detect markers
    markers = detector.detect(frame)
    
    for marker in markers:
        print(f"Marker ID: {marker.id}")
        print(f"Center: {marker.center}")
        print(f"Corners: {marker.corners}")  # 4 corner points
        print(f"Perimeter: {marker.perimeter}")
```

### Pose Estimation

Get 3D position and orientation of markers:

```python
from robo_infra.vision.markers import ArUcoDetector

detector = ArUcoDetector(dictionary="DICT_4X4_50")

# Get camera intrinsics (from calibration)
intrinsics = camera.get_intrinsics()

with USBCamera() as camera:
    frame = camera.capture()
    markers = detector.detect(frame)
    
    for marker in markers:
        # Estimate 6-DOF pose
        pose = detector.estimate_pose(
            marker,
            intrinsics,
            marker_size=0.05,  # 5cm marker
        )
        
        if pose:
            print(f"Marker {marker.id}:")
            print(f"  Translation: {pose.translation}")  # (x, y, z) in meters
            print(f"  Euler angles: {pose.euler_angles}")  # (roll, pitch, yaw) radians
            print(f"  Reprojection error: {pose.reprojection_error}")
```

### AprilTags

Similar interface, different marker family:

```python
from robo_infra.vision.markers import AprilTagDetector

# AprilTag families: "tag16h5", "tag25h9", "tag36h11", "tagCircle21h7"
detector = AprilTagDetector(family="tag36h11")

markers = detector.detect(frame)

for marker in markers:
    print(f"AprilTag {marker.id} at {marker.center}")
    
    # Pose estimation (if calibrated)
    pose = detector.estimate_pose(marker, intrinsics, marker_size=0.1)
```

### QR Codes

```python
from robo_infra.vision.markers import QRCodeDetector

detector = QRCodeDetector()

with USBCamera() as camera:
    frame = camera.capture()
    codes = detector.detect(frame)
    
    for code in codes:
        print(f"QR Code: {code.data}")
        print(f"Position: {code.center}")
        print(f"Corners: {code.corners}")
```

### Generating Markers

```python
from robo_infra.vision.markers import ArUcoDetector

detector = ArUcoDetector(dictionary="DICT_4X4_50")

# Generate marker image
marker_image = detector.generate_marker(
    marker_id=0,
    size=200,  # pixels
)

# Save for printing
import cv2
cv2.imwrite("marker_0.png", marker_image)

# Generate marker board
board_image = detector.generate_board(
    rows=4,
    cols=5,
    marker_size=100,  # pixels
    margin=20,
)
cv2.imwrite("marker_board.png", board_image)
```

---

## Image Processing

### Edge Detection

```python
from robo_infra.vision.processing import edge_detect

# Canny edge detection
edges = edge_detect(frame, method="canny")

# With thresholds
edges = edge_detect(frame, method="canny", threshold1=50, threshold2=150)

# Sobel edge detection
edges_sobel = edge_detect(frame, method="sobel")

# Laplacian
edges_lap = edge_detect(frame, method="laplacian")
```

### Contour Finding

```python
from robo_infra.vision.processing import find_contours, draw_contours

# Find contours
contours = find_contours(edges)

print(f"Found {len(contours)} contours")

for contour in contours:
    print(f"Area: {contour.area}")
    print(f"Perimeter: {contour.perimeter}")
    print(f"Center: {contour.centroid}")
    print(f"Bounding rect: {contour.bounding_rect}")
    print(f"Is convex: {contour.is_convex}")

# Draw contours on frame
annotated = draw_contours(frame, contours, color=(0, 255, 0))
```

### Object Measurement

```python
from robo_infra.vision.processing import find_contours, measure_distance

contours = find_contours(edges, min_area=100)

for contour in contours:
    # Get bounding box dimensions
    x, y, w, h = contour.bounding_rect
    print(f"Width: {w}px, Height: {h}px")
    
    # Fit ellipse for better measurement
    if contour.area > 100:
        ellipse = contour.fit_ellipse()
        print(f"Major axis: {ellipse.major_axis}px")
        print(f"Minor axis: {ellipse.minor_axis}px")
    
    # Approximate polygon
    approx = contour.approximate(epsilon=0.02)
    print(f"Polygon vertices: {len(approx)}")
```

### Basic Transformations

```python
from robo_infra.vision.processing import resize, crop, rotate, flip

# Resize
small = resize(frame, (320, 240))
large = resize(frame, (1920, 1080), interpolation="lanczos")

# Crop region of interest
roi = crop(frame, (100, 100, 200, 200))  # x, y, width, height

# Rotate
rotated = rotate(frame, 45.0)  # degrees
rotated_centered = rotate(frame, 45.0, center=(320, 240))

# Flip
flipped_h = flip(frame, "horizontal")
flipped_v = flip(frame, "vertical")
flipped_both = flip(frame, "both")
```

### Filtering

```python
from robo_infra.vision.processing import blur, sharpen, threshold

# Gaussian blur
blurred = blur(frame, kernel_size=5)
blurred_strong = blur(frame, kernel_size=11)

# Median blur (good for salt-and-pepper noise)
median = blur(frame, kernel_size=5, method="median")

# Bilateral filter (preserves edges)
bilateral = blur(frame, kernel_size=9, method="bilateral")

# Sharpen
sharpened = sharpen(frame)

# Threshold
binary = threshold(frame, value=128)
adaptive = threshold(frame, method="adaptive")
otsu = threshold(frame, method="otsu")
```

### Color Conversions

```python
from robo_infra.vision.processing import to_grayscale, to_hsv, to_rgb

# Convert to grayscale
gray = to_grayscale(frame)

# Convert to HSV
hsv = to_hsv(frame)

# Convert back to RGB/BGR
rgb = to_rgb(hsv_frame)
```

---

## Camera Calibration

### Intrinsic Parameters

Camera intrinsics describe the internal camera geometry:

```python
from robo_infra.vision.calibration import CameraCalibrator, CameraIntrinsics

# Create calibrator
calibrator = CameraCalibrator(
    board_size=(9, 6),      # Chessboard corners (inner)
    square_size=0.025,      # 25mm squares
)

# Capture calibration images
with USBCamera() as camera:
    images = []
    print("Press SPACE to capture, Q to finish")
    
    while len(images) < 20:
        frame = camera.capture()
        # Show frame with detected corners
        display = calibrator.draw_corners(frame)
        cv2.imshow("Calibration", display.data)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            if calibrator.find_corners(frame):
                images.append(frame)
                print(f"Captured {len(images)}/20")
        elif key == ord('q'):
            break
    
    # Calibrate
    intrinsics = calibrator.calibrate(images)
    
    print(f"Camera matrix:\n{intrinsics.camera_matrix}")
    print(f"Distortion coefficients: {intrinsics.dist_coeffs}")
    print(f"Reprojection error: {intrinsics.reprojection_error:.4f}")
    
    # Save calibration
    intrinsics.save("camera_calibration.json")
```

### Distortion Correction

```python
from robo_infra.vision.calibration import CameraIntrinsics

# Load calibration
intrinsics = CameraIntrinsics.load("camera_calibration.json")

# Undistort frames
with USBCamera() as camera:
    frame = camera.capture()
    undistorted = intrinsics.undistort(frame)
```

### Using Calibration

```python
# Load once at startup
intrinsics = CameraIntrinsics.load("camera_calibration.json")

# Use for marker pose estimation
detector = ArUcoDetector(dictionary="DICT_4X4_50")

with USBCamera() as camera:
    frame = camera.capture()
    undistorted = intrinsics.undistort(frame)
    
    markers = detector.detect(undistorted)
    for marker in markers:
        pose = detector.estimate_pose(marker, intrinsics, marker_size=0.05)
```

---

## Coordinate Transforms

### Pixel to World Coordinates

Convert 2D pixel coordinates to 3D world coordinates:

```python
from robo_infra.vision.calibration import pixel_to_world

# Known Z plane (e.g., table surface)
z_plane = 0.0  # meters

# Pixel coordinates
pixel_x, pixel_y = 320, 240

# Convert to world coordinates
world_x, world_y, world_z = pixel_to_world(
    pixel_x, pixel_y,
    intrinsics=intrinsics,
    z_plane=z_plane,
)

print(f"World position: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
```

### World to Pixel Coordinates

Project 3D points onto the image:

```python
from robo_infra.vision.calibration import world_to_pixel

# World coordinates
world_point = (0.1, 0.05, 0.5)  # meters

# Project to pixel
pixel_x, pixel_y = world_to_pixel(
    world_point,
    intrinsics=intrinsics,
    extrinsics=camera_pose,  # Camera position in world
)

print(f"Pixel position: ({pixel_x}, {pixel_y})")
```

### Camera-to-Robot Transform

Calibrate the transform from camera frame to robot frame:

```python
from robo_infra.vision.calibration import HandEyeCalibrator
from robo_infra.motion.transforms import Transform

# Hand-eye calibration
calibrator = HandEyeCalibrator()

# Collect data: move robot to known poses, capture marker poses
for i in range(20):
    # Move robot to pose i
    robot_pose = robot.get_pose()  # Robot end-effector pose
    
    # Capture marker pose
    frame = camera.capture()
    markers = detector.detect(frame)
    marker_pose = detector.estimate_pose(markers[0], intrinsics, 0.05)
    
    calibrator.add_sample(robot_pose, marker_pose)

# Compute camera-to-robot transform
camera_to_robot = calibrator.calibrate()

print(f"Camera to robot transform:\n{camera_to_robot}")

# Now convert marker positions to robot coordinates
marker_in_camera = detector.estimate_pose(marker, intrinsics, 0.05)
marker_in_robot = camera_to_robot @ marker_in_camera.as_transform()

print(f"Marker position in robot frame: {marker_in_robot.position}")
```

---

## Complete Vision Example

```python
from robo_infra.sensors.camera import USBCamera
from robo_infra.vision.color import ColorDetector
from robo_infra.vision.markers import ArUcoDetector
from robo_infra.vision.calibration import CameraIntrinsics
from robo_infra.controllers import PanTilt
import time

# Load camera calibration
intrinsics = CameraIntrinsics.load("camera_calibration.json")

# Create detectors
color_detector = ColorDetector.red()
marker_detector = ArUcoDetector(dictionary="DICT_4X4_50")

# Create pan-tilt for tracking
pan_tilt = PanTilt(
    pan_actuator=pan_servo,
    tilt_actuator=tilt_servo,
)

with USBCamera() as camera:
    pan_tilt.enable()
    pan_tilt.center()
    
    while True:
        frame = camera.capture()
        undistorted = intrinsics.undistort(frame)
        
        # Try marker detection first (more accurate)
        markers = marker_detector.detect(undistorted)
        
        if markers:
            # Track marker
            target = markers[0]
            pan_tilt.track(target.center)
            
            # Get 3D pose
            pose = marker_detector.estimate_pose(
                target, intrinsics, marker_size=0.05
            )
            if pose:
                print(f"Marker {target.id} at {pose.translation}")
        else:
            # Fall back to color detection
            blobs = color_detector.detect(undistorted, min_area=500)
            
            if blobs:
                largest = max(blobs, key=lambda b: b.area)
                pan_tilt.track(largest.center)
                print(f"Color blob at {largest.center}")
        
        time.sleep(0.033)  # ~30 FPS

    pan_tilt.disable()
```

---

## Next Steps

- [Sensors](sensors.md) - Camera sensors and configuration
- [Controllers](controllers.md) - PanTilt for visual tracking
- [Safety](safety.md) - Vision-based safety systems
- [Kinematics](kinematics.md) - Camera-robot calibration

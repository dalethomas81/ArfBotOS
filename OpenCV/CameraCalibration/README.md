# CalibrateCamera.py

`CalibrateCamera.py` performs a single-image chessboard calibration for the ArfBotOS vision pipeline. It captures or loads an image of a checkerboard, finds the internal corners, estimates the camera matrix and distortion coefficients, computes image-to-machine alignment values, writes `cal.yaml`, saves an annotated result image, and prints a short `CAL` status line.

## What The Script Does

At a high level the script runs this pipeline:

1. Parse command line arguments for output image, square size, checkerboard dimensions, debug mode, and capture resolution.
2. Capture an image:
   - In debug mode it loads `Input.jpg`.
   - In normal mode it captures a frame using `Picamera2`.
3. Convert the image to grayscale.
4. Detect checkerboard corners with `cv2.findChessboardCorners()`.
5. Refine the detected corners with `cv2.cornerSubPix()`.
6. Build the 3D checkerboard reference points using the supplied checkerboard size and physical square size.
7. Run `cv2.calibrateCamera()` to compute the intrinsic matrix, distortion coefficients, and reprojection error.
8. Derive additional vision alignment values from the detected checkerboard:
   - `pixelratio`
   - `rotation_offset`
   - `origin`
   - `top_left`
   - `bot_right`
9. Save all of that data into `cal.yaml`.
10. Draw the detected checkerboard corners and orientation arrows onto the image.
11. Undistort the result image using the newly computed calibration.
12. Save the annotated image and print a `CAL` summary line.

If the checkerboard cannot be found, the script returns `-1` values for the computed outputs and does not generate a useful calibration result.

## Inputs

The script needs:

- a visible checkerboard in the camera image
- the checkerboard internal corner count
- the real-world square size

The checkerboard size is passed as:

- `checkerboard_x`: number of inner corners across
- `checkerboard_y`: number of inner corners down

The square size is passed as the physical distance between adjacent checkerboard corners, in the user unit you want to calibrate against. The default in this script is `25`, and the comments indicate millimeters are intended.

## What Gets Saved To `cal.yaml`

`save_cal_data()` writes these values:

- `checkerboard`
- `squaresize`
- `matrix`
- `distortion`
- `pixelratio`
- `rotation_offset`
- `top_left`
- `bot_right`
- `origin`

These are later consumed by other scripts such as template matching and image capture.

## How The Derived Values Are Computed

### Pixel ratio

The script estimates the average distance between adjacent checkerboard corners in pixels with `calculate_average_square_size()`. It then computes:

```text
pixelratio = average_square_size_in_pixels / squaresize
```

That gives pixels per user unit.

### Rotation offset

`get_rotation_offset()` compares the first checkerboard corner with the corner at the bottom of the first column and converts that into an angle. This is the camera-to-board rotation used later to align vision coordinates to the calibrated frame.

### Origin and corners

The script uses:

- the checkerboard top-left detected corner as `origin`
- the same point again as `top_left` for backward compatibility
- the checkerboard bottom-right detected corner as `bot_right`

Downstream scripts use these saved points for coordinate conversion and cropping.

## Result Image

When calibration succeeds, the saved result image includes:

- the detected checkerboard corners
- a red X-axis arrow
- a green Y-axis arrow
- the undistorted output image

The orientation arrows are drawn from the detected origin and rotated by the computed calibration angle.

## Command Line Options

The script supports these arguments:

- `-r` or `--resultfile`: output image path, default `result.jpg`
- `-l` or `--squaresize`: physical checkerboard square size, default `25`
- `-x` or `--checkerboard_x`: checkerboard inner corners in X, default `6`
- `-y` or `--checkerboard_y`: checkerboard inner corners in Y, default `9`
- `-d` or `--debug`: boolean flag such as `true` or `false`
- `-w` or `--width`: capture width, default `3280`
- `-h` or `--height`: capture height, default `2464`

Example:

```bash
python CalibrateCamera.py -r cal_result.jpg -l 25 -x 6 -y 9 -d true -w 3280 -h 2464
```

## Output Format

After running, the script prints a summary line like:

```text
CAL rat:23.23 rot:23.23 err:23.23
```

The fields are:

- `rat`: computed `pixelratio`
- `rot`: computed `rotation_offset` in degrees
- `err`: reprojection error returned by `cv2.calibrateCamera()`

This is the response format expected by parts of the CODESYS side of the project.

## Debug Mode

Debug mode is intended for offline development and testing:

- it loads `Input.jpg` instead of using the camera
- it keeps the rest of the calibration flow the same

That makes it useful for tuning checkerboard detection on a desktop machine before moving to the live camera path.

## Relationship To Other Files

- `cal.yaml` is the important output of this script.
- `roi.yaml` exists in this folder, but this script does not currently write or use it during the active code path.
- The `patterns` folder is present for calibration-related assets, but the current script directly reads a captured image rather than iterating a set of pattern files.

## Current Notes

- This script calibrates from a single detected checkerboard image, not from a batch of multiple calibration views.
- `filter_image()` contains an optional crop path, but the active calibration flow uses undistortion without cropping.
- There are helper functions for rotating and cropping images that are not used in the current main path.
- On failure to find the checkerboard, the script reports `-1` values rather than throwing an exception.

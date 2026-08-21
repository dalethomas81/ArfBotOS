# TemplateMatch.py

`TemplateMatch.py` is a simple OpenCV-based part finder. It captures or loads an image, applies the saved camera calibration, searches for a template image inside the camera view, saves a marked-up result image, and prints the matched location.

## What The Script Does

At a high level the script runs this pipeline:

1. Parse command line arguments for output file, template file, thresholds, debug mode, and camera resolution.
2. Load camera calibration data from `cal.yaml` in the same folder.
3. Acquire an image:
   - In debug mode it loads `Input.jpg`.
   - In normal mode it captures a frame with `Picamera2`.
4. If not in debug mode, preprocess the captured image using the calibration data:
   - undistort the image using the camera matrix and distortion coefficients
   - rotate it by the saved `rotation_offset`
   - crop it to the saved region of interest defined by `top_left` and `bot_right`
5. Run OpenCV template matching against the provided template image.
6. Draw a green rectangle and crosshair on the detected match.
7. Save the annotated image to the requested output path.
8. Print the matched top-left pixel coordinate to stdout.

## Calibration Data

The script expects a `cal.yaml` file beside `TemplateMatch.py`. It reads:

- `matrix` and `distortion` for lens correction
- `rotation_offset` to align the image
- `top_left` and `bot_right` to crop the working area
- additional calibration values such as `checkerboard`, `squaresize`, and `pixelratio`

The calibration data comes from [`CalibrateCamera.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/CameraCalibration/CalibrateCamera.py) in `OpenCV/CameraCalibration/`. That script writes a `cal.yaml` file containing the camera matrix, distortion coefficients, pixel ratio, rotation offset, and detected checkerboard reference points.

This `TemplateMatching` folder does not currently include its own `cal.yaml` in the repo, so in practice the calibration file needs to be copied here from a calibration run or from the deployed vision directory after `CalibrateCamera.py` has been run for the active camera setup.

## Template Matching Details

The actual matching is done in `FindTemplate()`:

- The input image is converted to grayscale.
- The template image is loaded in grayscale.
- The script tries three OpenCV match modes:
  - `cv2.TM_CCOEFF`
  - `cv2.TM_CCORR`
  - `cv2.TM_SQDIFF`
- It keeps the result with the strongest score and converts that into the final top-left match position.
- The match box size comes directly from the template image dimensions.

After the match is chosen, the script draws:

- a green rectangle around the detected template
- a green crosshair at the center of that rectangle

## Command Line Options

The script supports these arguments:

- `-s` or `--savelocation`: output image path, default `image.jpg`
- `-t` or `--templatelocation`: template image path, default `template.jpg`
- `-i` or `--minthreshold`: default `0.9`
- `-j` or `--maxthreshold`: default `1.0`
- `-d` or `--debug`: boolean flag such as `true` or `false`
- `-w` or `--width`: capture width, default `3280`
- `-h` or `--height`: capture height, default `2464`

Example:

```bash
python TemplateMatch.py -s output.jpg -t Template.jpg -d true -w 2016 -h 2000
```

## Debug Mode

Debug mode is intended for development away from the Raspberry Pi camera stack. Instead of opening `Picamera2`, it loads `Input.jpg` from the current working directory and skips the calibration/rotation/cropping stage.

That makes debug mode useful for quick testing on a desktop machine, but it also means the result can differ from the live runtime path if the real system depends on calibration.

## Outputs

The script produces two outputs:

- an annotated image written to `savelocation`
- a printed `(x, y)` top-left pixel coordinate from the template match

## Current Notes

- `minthreshold` and `maxthreshold` are parsed but are not currently used inside `FindTemplate()`.
- `FindTemplate()` returns the top-left corner of the match, not the center point.
- There are two `rotate_image()` definitions in the file; the second one is the version that is actually used.

# FastTemplateMatching.py

`FastTemplateMatching.py` is the more capable vision locator used by ArfBotOS when a part may appear at different rotations and when the result needs to be converted into calibrated machine coordinates. It captures or loads a grayscale image, searches for one or more rotated instances of a template inside a configured ROI, prints `LOC` result lines, and saves an annotated result image.

## What The Script Does

At a high level the script runs this pipeline:

1. Parse command line arguments for output path, template path, capture size, number of matches, overlap limit, minimum score, angle tolerance, and debug mode.
2. Load camera calibration data from `cal.yaml`.
3. Load the search region from `roi.yaml`.
4. Acquire an image:
   - In debug mode it loads `Input.jpg` in grayscale.
   - In normal mode it captures a grayscale frame using `Picamera2` with `YUV420`.
5. If not in debug mode, undistort the captured image with the saved camera matrix and distortion coefficients.
6. Load the template image in grayscale.
7. Learn the template by building an image pyramid and precomputing statistics needed for fast normalized matching.
8. Crop the source image to the ROI and search that ROI across a range of rotation angles.
9. Refine the best candidates from the top pyramid layer down to full resolution.
10. Filter weak and overlapping matches.
11. Convert each match center from ROI pixels into calibrated coordinates using `origin`, `rotation_offset`, and `pixelratio`.
12. Print one `LOC` line per object and save an annotated image of the ROI.

## Required Files In This Folder

This script expects these companion files beside it:

- `cal.yaml`
- `roi.yaml`
- a template image such as `Template.jpg`

This folder already contains sample inputs including:

- `Input.jpg`
- `outputimage.bmp`
- multiple `Test*` folders with reference images and calibration files

## Calibration And ROI Data

`cal.yaml` provides the camera and coordinate conversion data:

- `matrix` and `distortion` for lens correction
- `pixelratio` to convert pixels into user units
- `rotation_offset` to align image coordinates to the calibrated frame
- `origin` as the coordinate reference point
- `top_left` and `bot_right` from calibration output

The calibration data comes from [`CalibrateCamera.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/CameraCalibration/CalibrateCamera.py) in `OpenCV/CameraCalibration/`. Running that script produces a `cal.yaml` file with the camera matrix, distortion coefficients, pixel ratio, rotation offset, and reference points used here for undistortion and coordinate conversion.

`roi.yaml` provides the active search window:

- `top_left`
- `bot_right`

The search happens only inside the ROI from `roi.yaml`, not across the whole captured image. That keeps the search faster and limits false positives.

In this folder, the checked-in `cal.yaml` is an example calibration output. If the camera, lens, mounting angle, or working distance changes, `CalibrateCamera.py` should be run again and the resulting `cal.yaml` should replace the old one for accurate results.

## How The Matching Works

The main matcher is substantially more advanced than the simple template matcher.

### 1. Template learning

`learn_pattern()` builds a pyramid of the template image and precomputes per-layer values such as:

- mean intensity
- normalization terms
- inverse area
- border color used when rotating source images

These values are later reused during matching so the full template statistics do not need to be recalculated for every candidate.

### 2. Coarse rotation search

At the top pyramid layer, the script:

- builds a matching pyramid for the ROI
- generates candidate angles from `-dToleranceAngle` to `+dToleranceAngle`
- rotates the ROI to each candidate angle
- runs template matching on the rotated ROI
- stores the best candidate locations, scores, and angles

Matching uses `cv2.matchTemplate(..., cv2.TM_CCORR)` followed by a custom normalization step in `CCOEFF_Denominator()` so the score behaves more like a normalized correlation score.

### 3. Coarse-to-fine refinement

After the first pass, each candidate is refined one pyramid level at a time:

- the search angle is narrowed around the current best angle
- a rotated local ROI is extracted
- the candidate is re-matched at the next finer level
- the best position and angle are carried forward

This makes the search much faster than testing every rotation at full resolution from the start.

### 4. Score and overlap filtering

Once all candidates are refined:

- `filter_with_score()` removes anything below `dScore`
- `FilterWithRotatedRect()` removes duplicate detections whose rotated boxes overlap more than `dMaxOverlap`

That overlap filtering is important when the same object would otherwise be reported multiple times at nearby positions or angles.

## Output Format

For each accepted match the script prints a line like:

```text
LOC obj:0 cx:123.45 cy:678.90 a:55.94 s:0.95
```

The fields are:

- `obj`: zero-based object index
- `cx`: calibrated X coordinate
- `cy`: calibrated Y coordinate
- `a`: matched angle in degrees
- `s`: match score

The coordinate conversion works like this:

1. Take the match center inside the ROI.
2. Add the ROI offset back in.
3. Rotate that point around the calibrated `origin` using `rotation_offset`.
4. Subtract `origin`.
5. Divide by `pixelratio` to convert pixels into user units.

## Result Image

The script draws the detections with `RefreshSrcView()`:

- rotated bounding boxes
- a center crosshair
- corner indicators
- an index label for each match

Although the code can also overlay the ROI back onto the full source image, the final save currently writes only the marked ROI image to `savelocation`.

## Command Line Options

The script supports these arguments:

- `-s` or `--savelocation`: output image path
- `-t` or `--templatelocation`: template image path
- `-w` or `--width`: capture width, default `640`
- `-h` or `--height`: capture height, default `400`
- `-i` or `--iMaxPos`: maximum number of objects to report, default `2`
- `-j` or `--dMaxOverlap`: allowed overlap ratio before duplicate matches are removed, default `0.0`
- `-k` or `--dScore`: minimum score threshold, default `0.6`
- `-l` or `--dToleranceAngle`: rotation search range in degrees, default `90.0`
- `-d` or `--debug`: boolean flag such as `true` or `false`

Example:

```bash
python FastTemplateMatching.py -s outputimage.bmp -t Template.jpg -i 5 -j 0.0 -k 0.8 -l 90.0 -d true
```

## Debug Mode

Debug mode is useful for development on a desktop machine:

- it loads `Input.jpg`
- it skips the live camera path
- it also skips the undistortion step that runs in normal mode

Because of that, debug results are good for algorithm development, but final tuning should still be done with live calibrated images.

## Current Notes

- The source image and template are treated as grayscale.
- The ROI search comes from `roi.yaml`, while calibrated coordinate conversion comes from `cal.yaml`.
- The final reported angle is the detected template angle; the commented code shows there was once consideration to also fold in `rotation_offset`.
- Several branches are marked in comments as not yet tested, including some tolerance-range, block-max, and sub-pixel paths.
- The script builds a full annotated image with ROI overlay internally, but currently saves `markedRoi` rather than the full overlaid image.

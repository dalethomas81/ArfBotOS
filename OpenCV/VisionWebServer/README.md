# VisionWebServer

`OpenCV/VisionWebServer` is a small Flask application that provides a browser-based interface for vision template management and for viewing the latest vision result image. It lets an operator capture a camera image, drag out a crop to create a template, save that crop into the deployed templates directory, browse or delete saved templates, and serve the latest processed output image back to the HMI.

## High-Level Structure

The folder is organized like this:

- [`vision.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/vision.py): top-level Flask entrypoint
- [`config.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/config.py): Flask config object and default SQLite URI
- [`app/__init__.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/app/__init__.py): creates the Flask app and loads routes
- [`app/routes.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/app/routes.py): main application behavior
- `app/templates/`: Jinja templates and client-side JavaScript

The actual working logic is almost entirely in `app/routes.py` and `app/templates/template.html`.

## What The Web Server Does

There are two main purposes:

1. Template management for the vision system.
2. HTTP serving of the latest vision output image for browser or HMI use.

In practice this gives the project a lightweight maintenance UI for creating and managing the template files used by the template-matching scripts.

## App Initialization

The Flask app is created in [`app/__init__.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/app/__init__.py):

- `app = Flask(__name__)`
- config is loaded from [`config.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/config.py)
- routes are imported from `app.routes`

`config.py` defines:

- `SECRET_KEY`
- `SQLALCHEMY_DATABASE_URI`
- `SQLALCHEMY_TRACK_MODIFICATIONS`

The SQLAlchemy setup is currently commented out, so the app behaves as a simple Flask app without an active database layer.

## Important Paths

The web server is wired to the deployed vision runtime directories, not just files inside this repo.

### Template storage

Uploaded and saved templates go to:

```text
/var/opt/codesys/PlcLogic/Application/Vision/Templates
```

This matches the template directory referenced elsewhere in the project and in the CODESYS configuration.

### Vision result image

The latest processed output image is served from:

```text
/var/opt/codesys/PlcLogic/visu/outputimage.jpg
```

That is the image path the HMI can reference when it wants to show the most recent vision result.

## Main Routes

### `/` and `/template`

These routes render [`template.html`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/app/templates/template.html), which is the main UI for capturing and cropping templates.

The page includes:

- a live still-image capture area
- a canvas overlay for click-and-drag rectangle selection
- width and height inputs for the requested capture size
- a template name field
- a cropped preview area
- a save button

This is the main operator workflow for making a new vision template.

### `/captured_image`

This route accepts optional query parameters:

- `width`
- `height`

It calls `gen_frames(width, height)`, which:

- opens `Picamera2`
- configures a preview capture in `RGB888`
- captures one frame
- stores that frame in the global `capArray`
- JPEG-encodes it
- returns it as a multipart HTTP response

Despite the name `gen_frames`, the current implementation captures and yields one still image per request rather than maintaining a continuous live stream.

### `/cropped`

This route accepts:

- `top_left`
- `bot_right`

Those are passed as comma-separated pixel coordinates from the browser JavaScript. The route calls `crop_template()`, which:

- parses the coordinates
- crops the previously captured global `capArray`
- stores the crop in the global `croppedImage`
- JPEG-encodes the cropped image
- returns it to the browser

This is how the UI previews the user-selected template region before saving it.

### `/save_template/<filename>`

This route writes the last cropped image to disk in the templates directory using:

```python
cv2.imwrite(os.path.join(app.config['UPLOAD_FOLDER'], filename), croppedImage)
```

It returns JSON-like data:

```text
{'status': 'saved'}
```

The browser calls this after the user enters a template name and clicks Save.

### `/delete_template/<filename>`

This route removes a saved template file from the templates directory and returns:

```text
{'status': 'deleted'}
```

### `/files/`

This route lists all files in the template directory and renders [`saved_templates.html`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/app/templates/saved_templates.html).

The page shows:

- a thumbnail of each saved template
- the filename
- a delete button

### `/files/<filename>`

This serves the requested template image from the templates directory.

### `/upload`

This is a simple upload endpoint that accepts `png`, `jpg`, and `jpeg` files and stores them in the templates directory. It uses `secure_filename()` and creates the folder if needed.

### `/output`

This directly serves `/var/opt/codesys/PlcLogic/visu/outputimage.jpg`.

### `/output_sized`

This route accepts:

- `width`
- `height`

It reads `outputimage.jpg`, resizes it with OpenCV, encodes it as JPEG, and returns it via `send_file()`.

This route is referenced from the CODESYS side of the project for displaying the vision output image at HMI-friendly dimensions.

## Browser Workflow For Creating A Template

The main page in [`template.html`](c:/Users/dalet/Github/ArfBotOS/OpenCV/VisionWebServer/app/templates/template.html) works like this:

1. The user enters a desired capture width and height.
2. Clicking `Capture Image` reloads the `/captured_image` URL with those dimensions.
3. The browser displays the captured image with a canvas over it.
4. The user clicks and drags on the canvas to define a rectangle.
5. On mouse release, the page computes normalized top-left and bottom-right coordinates regardless of drag direction.
6. The page requests `/cropped?...` to preview the selected region.
7. The user enters a template filename.
8. Clicking `Save` calls `/save_template/<name>.jpg`.

The JavaScript also appends a timestamp query string to image URLs so the browser does not show a stale cached image.

## How It Connects To The Rest Of ArfBotOS

- Templates saved here are intended for the template-matching scripts in `OpenCV/TemplateMatching/` and `OpenCV/FastTemplateMatching/`.
- The template directory matches the runtime paths used by the socket server and by CODESYS command strings.
- `/output_sized` is referenced in `Codesys/ArfBot.xml` to display the latest vision result image in the HMI.

## Current Notes

- `vision.py` only imports `app`; there is no `app.run()` in this repo, so the server is intended to be started through Flask tooling or another WSGI launcher.
- Database-backed models and login flows appear to come from an earlier Flask tutorial and are currently unused.
- `TemplateForm` exists but the main template page is driven by custom HTML and JavaScript rather than Flask-WTF form submission.
- The app relies on global variables such as `capArray` and `croppedImage` to pass image data between requests.
- Because of those globals, the current implementation is best suited to simple single-user or low-concurrency use.
- Error handling is minimal; several exceptions are caught and ignored without returning structured error responses.
- Deleting a template does not currently check whether the file exists before calling `os.remove()`.

from flask import render_template, flash, redirect, url_for, Response, Flask, request, abort, send_file
from app import app
from app.forms import TemplateForm
#from app.forms import LoginForm, TemplateForm

from flask import request

import cv2
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import time
from array import array

# https://flask.palletsprojects.com/en/2.3.x/patterns/fileuploads/
import os
from werkzeug.utils import secure_filename
#UPLOAD_FOLDER = os.path.abspath(os.path.dirname(__file__)) + '/uploads' #'/srv'
UPLOAD_FOLDER = '/var/opt/codesys/PlcLogic/Application/Vision/Templates'
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}
app.config['MAX_CONTENT_LENGTH'] = 16 * 1000 * 1000
#app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/upload', methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        # check if the post request has the file part
        if 'file' not in request.files:
            flash('No file part')
            return redirect(request.url)
        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if file.filename == '':
            flash('No selected file')
            return redirect(request.url)
        if file and allowed_file(file.filename):
            if not os.path.exists(app.config['UPLOAD_FOLDER']):
                os.makedirs(app.config['UPLOAD_FOLDER'])
            filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
            return redirect(url_for('download_file', name=filename))
    return '''
    <!doctype html>
    <title>Upload new File</title>
    <h1>Upload new File</h1>
    <form method=post enctype=multipart/form-data>
      <input type=file name=file>
      <input type=submit value=Upload>
    </form>
    '''
from flask import send_from_directory
@app.route('/uploads/<name>')
def download_file(name):
    return send_from_directory(app.config["UPLOAD_FOLDER"], name)
app.add_url_rule(
    "/uploads/<name>", endpoint="download_file", build_only=True
)


# save template
@app.route('/save_template')
@app.route('/save_template/<filename>')
def save_template(filename=None):
    global croppedImage
    if not os.path.exists(app.config['UPLOAD_FOLDER']):
        os.makedirs(app.config['UPLOAD_FOLDER'])
    cv2.imwrite(os.path.join(app.config['UPLOAD_FOLDER'], filename), croppedImage)
    #flash('Template saved')
    data = {'status': 'saved'}
    return data, 200


# delete template
@app.route('/delete_template')
@app.route('/delete_template/<filename>')
def delete_template(filename=None):
    os.remove(os.path.join(app.config['UPLOAD_FOLDER'], filename))
    #flash('Template deleted')
    data = {'status': 'deleted'}
    return data, 200


# show files
# https://stackoverflow.com/questions/67373819/python-flask-app-which-returns-list-of-files
@app.route('/files/')
def files():
    filenames = os.listdir(app.config['UPLOAD_FOLDER'])
    return render_template('saved_templates.html', files=filenames)
@app.route('/files/<path:filename>')
def file(filename):
    return send_from_directory(
        app.config['UPLOAD_FOLDER'],
        filename,
        as_attachment=False # when True you can click on link and download
    )
    
@app.route('/output')
def output():
    return send_from_directory(
        '/var/opt/codesys/PlcLogic/visu/',
        'outputimage.jpg',
        as_attachment=False # when True you can click on link and download
    )

# index
'''
@app.route('/')
@app.route('/index')
def index():
    user = {'username': 'Dale'}
    posts = [
        {
            'author': {'username': 'John'},
            'body': 'Beautiful day in Portland!'
        },
        {
            'author': {'username': 'Susan'},
            'body': 'The Avengers movie was so cool!'
        }
    ]
    return render_template('index.html', title='Home', user=user, posts=posts)


# login
@app.route('/login', methods=['GET', 'POST'])
def login():
    form = LoginForm()
    if form.validate_on_submit():
        flash('Login requested for user {}, remember_me={}'.format(
            form.username.data, form.remember_me.data))
        return redirect(url_for('index'))
    return render_template('login.html', title='Sign In', form=form)
'''

# template
@app.route('/')
@app.route('/template')
def template():
    return render_template('template.html', title='template title')


# captured image
@app.route('/captured_image')
def captured_image():
    # https://stackoverflow.com/questions/24892035/how-can-i-get-the-named-parameters-from-a-url-using-flask
    width = request.args.get('width', default = 640, type = int)
    height = request.args.get('height', default = 400, type = int)
    #print(width)
    #print(height)
    
    return Response(gen_frames(width, height), mimetype='multipart/x-mixed-replace; boundary=frame')


# cropped
@app.route('/cropped')
def cropped():
    # https://stackoverflow.com/questions/24892035/how-can-i-get-the-named-parameters-from-a-url-using-flask
    # https://flask.palletsprojects.com/en/3.0.x/api/#flask.Request.args
    top_left = request.args.get('top_left')#, default = [0,0], type = None)
    bot_right = request.args.get('bot_right')#, default = [640,400], type = None)
    print(top_left)
    print(bot_right)

    return Response(crop_template(top_left, bot_right), mimetype='multipart/x-mixed-replace; boundary=frame')


# draw rectangle
####### not validated ############
def draw_rectangle(startPoint, endPoint):
    # the global varible holds the current image.
    global buffer

    # make a copy of the original
    imageCopy = buffer.copy()

    # set up visuals
    rectColor = (0, 255, 0)
    rectThickness = 2

    # create the rectangle. note the image is passed by reference
    cv2.rectangle(imageCopy, startPoint, endPoint, rectColor, rectThickness)

    #
    return imageCopy


# crop template
def crop_template(top_left=None, bot_right=None):
    global croppedImage, buffer, frame, capArray
    try:
        _tl = top_left.split(",")
        _br = bot_right.split(",")

        _tla = list(map(int, _tl)) # convert ['0','0'] -> [0,0]
        _bra = list(map(int, _br)) # convert ['0','0'] -> [0,0]

        #_tla = array("i",_tl) # the 'i' is the typecode for integer
        #_bra = array("i",_br) # the 'i' is the typecode for integer

        # crop the image and return
        croppedImage = crop_image(capArray, _tla, _bra)
    
        ret, buffercropped = cv2.imencode('.jpg', croppedImage)
        frame = buffercropped.tobytes()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
    except Exception as e:
        pass


# crop image
def crop_image(image, top_left, bot_right):
    y = int(top_left[1])
    x = int(top_left[0])
    h = int(bot_right[1] - top_left[1])
    w = int(bot_right[0] - top_left[0])
    cropped = image[y:y+h, x:x+w].copy() # image slicing creates a pointer. So use copy()
    return cropped


# gen frames
def gen_frames(width, height):  # generate frame by frame from camera
    global out, capture, rec_frame, buffer, frame, capArray
    
    # https://github.com/raspberrypi/picamera2/discussions/702#discussioncomment-5965577
    with Picamera2() as camera: 
        config = camera.create_preview_configuration(main={"size": (int(width), int(height)), "format": "RGB888"}) # 640, 400
        camera.configure(config)
        camera.start()
        time.sleep(0.1)
        capArray = camera.capture_array()
        camera.stop()
        camera.close()

        try:
            ret, buffer = cv2.imencode('.jpg', capArray)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        except Exception as e:
            pass
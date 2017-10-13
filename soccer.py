from flask import Flask, make_response, request, abort, send_from_directory, send_file, Response
from threading import Thread, Event
from vision import ImageAnalyser
from webcam_stream import WebcamStream
from PIL import Image
import yaml
from io import BytesIO
import json

class Soccer:
    def __init__(self):
        self.config = {}
        self.load_config()
        self.run_event = Event()
        self.frame = None
        self.stop_flag = False
        self.analyser = ImageAnalyser(self.config)

    def save_config(self):
        with open('config.yaml', 'w') as yamlfile:
            yaml.dump(self.config, yamlfile)

    def load_config(self):
        with open('config.yaml') as yamlfile:
            self.config.update(yaml.load(yamlfile))

    def do_config(self, path, val=None):
        cur = self.config
        while len(path) > 0:
            if type(cur) == list:
                try:
                    ind = int(path[0])
                except (ValueError):
                    raise KeyError
            else:
                ind = path[0]
            path = path[1:]

            try:
                temp = cur[ind]
                if len(path) == 0 and val is not None:
                    cur[ind] = val
                    return
                cur = temp
            except (IndexError, KeyError):
                raise KeyError

        if type(cur) == dict:
            return list(cur.keys())
        elif type(cur) == list:
            return list(range(len(cur)))
        else:
            return cur

    def run(self):
        with WebcamStream() as cam:
            for frame in cam.frames():
                if self.stop_flag:
                    return
                self.analyser.analyse(frame)
                self.run_event.set()


    def start(self):
        self.stop_flag = False
        self.run_thread = Thread(target=self.run)
        self.run_thread.start()

    def stop(self):
        self.stop_flag = True
        self.run_thread.join()

    def jpeg_video(self):
        while not self.stop_flag:
            self.run_event.wait()
            self.run_event.clear()
            io = BytesIO()
            image = Image.fromarray(self.analyser.wallDisp)
            image.save(io, format="JPEG")
            io.seek(0)
            yield io.getvalue()


app = Flask(__name__)
app.debug = True
soccer = Soccer()
soccer.start()

def json_out(val):
    return make_response((json.dumps(val), 200, {'Content-Type': 'application/json'}))

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route("/video")
def video():
    pass


@app.route("/video/<video_id>")
def video_id():
    def multipart_jpeg():
        for jpeg in soccer.jpeg_video():
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
    return Response(multipart_jpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route('/config/')
@app.route('/config/<path:path>', methods=['GET', 'POST'])
def config(path=''):
    if len(path) > 0:
        path = path.split("/")
    path = [val for val in path if val != '']

    try:
        out = soccer.do_config(path)
    except KeyError:
        abort(404)

    if request.method == 'POST':
        soccer.do_config(path, val=request.get_json(force=True))
        return ''
    else:
        return json_out(out)

@app.route('/save')
def save():
    soccer.save_config()

@app.route('/load')
def load():
    soccer.load_config()

if __name__ == '__main__':
    app.run()
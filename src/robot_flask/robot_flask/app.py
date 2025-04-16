import json
import os
import time
from flask import Flask, Response, jsonify, render_template, request
import numpy as np
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading
import cv2
from cv_bridge import CvBridge
import whisper
import requests

RASP_HOST = '192.168.98.109'
RASP_PORT = '5000'

# 移动消息数组
movement = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
liner_speed = 0.2
angular_speed = 0.2

# 图像
IMAGE_HEIGHT = 600
IMAGE_WIDTH = 480
FREQUENCY = 40
camera_frame = ''

# 语音识别模型
model = whisper.load_model("base")

ollma_url = "http://192.168.98.152:11434/api/generate"

app = Flask(__name__, template_folder='/home/ros_system/ros_workspace/src/robot_flask/views')

@app.route("/")
def home_view():
    return render_template('home.html')

# 移动控制页面
@app.route("/ROV")
def rov_view():
    return render_template('rov.html')

# 移动控制接口
@app.route("/move_control", methods=['POST'])
def move_control():
    global movement
    data = request.get_json()
    mov = data['number']

    # 遥控按键布局
    # --------------------
    # ↖: 1 | ↑: 2 | ↗: 3
    # --------------------
    # ← : 4 | ○: 5 | → : 6
    # --------------------
    # ↙: 7 | ↓: 8 | ↘: 9
    # --------------------

    if mov == 2:
        # 前进
        movement[0] = liner_speed
        movement[5] = 0.0
    elif mov == 5:
        # 停止
        movement[0] = 0.0
        movement[5] = 0.0
    elif mov == 8:
        # 后退
        movement[0] = -liner_speed
        movement[5] = 0.0
    elif mov == 4:
        # 左转
        movement[0] = 0.0
        movement[5] = angular_speed
    elif mov == 6:
        # 右转
        movement[0] = 0.0
        movement[5] = -angular_speed

    return 'success'

# 前进
def movGoahead(data):
    mov_speed = 0.2
    mov_time = data / mov_speed
    movement[0] = mov_speed
    movement[5] = 0.0
    time.sleep(mov_time)
    movement[0] = 0.0
    movement[5] = 0.0
    return

# 后退
def movGoback(data):
    mov_speed = 0.2
    mov_time = data / mov_speed
    movement[0] = -mov_speed
    movement[5] = 0.0
    time.sleep(mov_time)
    movement[0] = 0.0
    movement[5] = 0.0
    return

# 左转
def movTurnleft(data):
    mov_turn_speed = 0.2
    mov_time = data * 3.1415 / 180 / mov_turn_speed
    movement[0] = 0.0
    movement[5] = mov_turn_speed
    time.sleep(mov_time - 2)
    movement[0] = 0.0
    movement[5] = 0.0
    return

# 右转
def movTurnright(data):
    mov_turn_speed = 0.2
    mov_time = data * 3.1415 / 180 / mov_turn_speed
    movement[0] = 0.0
    movement[5] = -mov_turn_speed
    time.sleep(mov_time - 2)
    movement[0] = 0.0
    movement[5] = 0.0
    return

# 停止
def movStop(data):
    movement[0] = 0.0
    movement[5] = 0.0
    return

# 移动控制话题发布结点
class MoveControlPublisher(Node):

    def __init__(self):
        super().__init__('move_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = movement[0]
        msg.linear.y = movement[1]
        msg.linear.z = movement[2]
        msg.angular.x = movement[3]
        msg.angular.y = movement[4]
        msg.angular.z = movement[5]
        self.publisher_.publish(msg)

# 相机话题发布结点
class CameraImagePublisher(Node):

    def __init__(self):
        super().__init__('camera')
        self.publisher_ = self.create_publisher(Image, 'sensor_msgs/msg/Image', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Image()
        if camera_frame == '':
            return
        else:
            msg = CvBridge.cv2_to_imgmsg(np.array(cv2.flip(camera_frame, 1)), encoding="bgr8")
            self.publisher_.publish(msg)


def nodeBuilder(Node):
    rclpy.init()
    node = Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 视频流
def generate_camera():
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
    while True:
        return_value, frame = camera.read()
        camera_frame = frame
        image = cv2.imencode('.jpg', frame)[1].tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + image + b'\r\n')
        
@app.route('/get_camera')
def get_camera():
    return Response(generate_camera(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 视频页面
@app.route("/CAM")
def cam_view():
    return render_template('camera.html')

# 语音页面
@app.route("/ASR")
def asr_view():
    return render_template('voice.html')

# 语音识别函数
def speechRecognizer(src):
    result = model.transcribe(audio=src, fp16=False, verbose=True, initial_prompt="以下是普通话的句子，含有数字", language="zh")
    r = requests.post(url=ollma_url, json={
        "model": "wangshenzhi/llama3-8b-chinese-chat-ollama-q4",
        "prompt": result["text"],
        "context": [
            198,
            128006,
            9125,
            128007,
            271,
            198,
            2675,
            527,
            264,
            11190,
            18328,
            13,
            198,
            128009,
            128006,
            882,
            128007,
            198,
            198,
            30177,
            113931,
            15225,
            57668,
            60979,
            105318,
            18904,
            59464,
            37046,
            1811,
            105318,
            121589,
            5232,
            37046,
            38093,
            70141,
            57668,
            73117,
            1,
            25580,
            42399,
            20,
            73361,
            1,
            5486,
            1,
            34547,
            56906,
            18,
            73361,
            1,
            5486,
            1,
            122434,
            1,
            50667,
            22238,
            127448,
            65305,
            3922,
            57668,
            86206,
            46281,
            25580,
            42399,
            1,
            3427,
            43887,
            1,
            5486,
            34547,
            56906,
            1,
            70,
            677,
            474,
            1,
            5486,
            78659,
            47770,
            1,
            413,
            2414,
            1,
            5486,
            65917,
            47770,
            1,
            413,
            1315,
            1,
            5486,
            122434,
            1,
            9684,
            1,
            76208,
            19483,
            33904,
            16325,
            51504,
            48044,
            113925,
            37046,
            1811,
            78657,
            3922,
            37046,
            73117,
            1,
            25580,
            42399,
            16,
            73361,
            1,
            3922,
            57668,
            18904,
            59464,
            1,
            5018,
            1337,
            3332,
            3427,
            43887,
            2247,
            695,
            794,
            16,
            10064,
            5486,
            1,
            78659,
            47770,
            966,
            27479,
            1,
            3922,
            1,
            5018,
            1337,
            3332,
            413,
            2414,
            2247,
            695,
            794,
            966,
            10064,
            5486,
            1,
            95475,
            113931,
            1,
            3922,
            1,
            5018,
            1337,
            3332,
            9684,
            2247,
            695,
            794,
            15,
            10064,
            1811,
            15225,
            61633,
            3922,
            113473,
            18904,
            59464,
            109545,
            43292,
            126189,
            9554,
            39404,
            18476,
            3922,
            57106,
            30624,
            72234,
            23039,
            39404,
            6447,
            15225,
            61633,
            3922,
            16325,
            106885,
            18476,
            48915,
            113473,
            111935,
            109545,
            119292,
            6447,
            1811,
            128009,
            128006,
            78191,
            128007,
            198,
            198,
            110085,
            3922,
            37046,
            120222,
            35287,
            1811,
            15225,
            73117,
            110310,
            64467,
            103507,
            1811
        ],
        "stream": False
    }, timeout=30)
    response = json.loads(r.json()["response"])
    print(response)
    if response["type"] == "goahead":
        movGoahead(response["data"])
    elif response["type"] == "goback":
        movGoback(response["data"])
    elif response["type"] == "turnleft":
        movTurnleft(response["data"])
    elif response["type"] == "turnright":
        movTurnright(response["data"])
    elif response["type"] == "stop":
        movStop(response["data"])
    return response

# 语音上传
@app.route('/upload_speech', methods=['POST'])
def upload_speech():
    if 'file' not in request.files:
        return jsonify({"error": "未接收到文件"}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "未选择文件"}), 400

    # 保存上传的文件
    file_path = "speech_t.wav"
    file.save(file_path)

    # 进行语音识别
    recognized_text = speechRecognizer(file_path)

    # 删除临时文件
    os.remove(file_path)

    # 返回识别结果
    return jsonify({"text": recognized_text})

def main(args=None):
    move_node = threading.Thread(target=nodeBuilder, args=(MoveControlPublisher,))
    move_node.start()
    # camera_node = threading.Thread(target=nodeBuilder, args=(CameraImagePublisher,))
    # camera_node.start()
    app.run(host=RASP_HOST, port=RASP_PORT)

if __name__ == '__main__':
    main()

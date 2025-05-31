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
import nav2_inter
import ollama_lib
from nav2_simple_commander.robot_navigator import BasicNavigator
import pyaudio
from vosk import Model, KaldiRecognizer
from fuzzywuzzy import fuzz, process
from playsound import playsound

RASP_HOST = '192.168.240.109'
RASP_PORT = '5000'
ollma_url = ollama_lib.OLLAMA_URL + "/api/generate"
vosk_model_path = "/home/ros_system/ros_workspace/src/robot_flask/models/vosk-model-small-cn-0.22"
wake_word = "机器人"
stream_sample_rate = 48000
stream_chunk_size = 1024

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
model = whisper.load_model("tiny")

# # vosk语音模型
# vosk_model = Model(vosk_model_path)
# vosk_rec = KaldiRecognizer(vosk_model, stream_sample_rate)

# # 初始化 PyAudio
# p = pyaudio.PyAudio()
# stream = p.open(format=pyaudio.paInt16,
#                 channels=1,
#                 rate=stream_sample_rate,
#                 input=True,
#                 frames_per_buffer=stream_chunk_size)

# 模糊匹配
def fuzzy_match(target, choices):
    best_match = process.extractOne(target, choices)
    return best_match

# 匹配序列
choices_list = [
    "前往A点", 
    "前往B点", 
    "前往C点"
    ]

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

# 导航到点
def nav2pose(data):
    pose = ''
    if data == '1': 
        pose = {
            "x": 3.0,
            "y": 0.0,
            "w": 1.0
        }
    elif data == '2': 
        pose = {
            "x": -1.0,
            "y": 3.0,
            "w": 1.0
        }
    else: 
        return 0
    rclpy.init()
    navigator = BasicNavigator()
    # 等待导航启动完成
    navigator.waitUntilNav2Active()
    nav2_inter.navToPoseOnce(navigator, pose, overtime=600.0)
    return 1

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

def selectFunction(response):
    rt = response["type"]
    rd = response["data"]

    print(rt + "(" + str(rd) + ")")

    if rt == "goahead":
        movGoahead(rd)
    elif rt == "goback":
        movGoback(rd)
    elif rt == "turnleft":
        movTurnleft(rd)
    elif rt == "turnright":
        movTurnright(rd)
    elif rt == "stop":
        movStop(rd)
    elif rt == "navToPose":
        nav2pose(rd)
    elif rt == "feedback":
        print("检测到非法指令，请重试！")
    return 0

# 语音识别函数
def speechRecognizer(src):
    result = model.transcribe(audio=src, fp16=False, verbose=False, initial_prompt="含有中文，可能包含字母或数字", language="zh")

    print(result["text"])
    print("### 开始指令识别 ###")

    r = requests.post(url=ollma_url, json={
        "model": ollama_lib.OLLAMA_MODEL,
        "prompt": result["text"],
        # "prompt": "前进到点A",
        "context": ollama_lib.CONTEXT_ALL_THE_THING,
        "stream": False
    }, timeout=90)
    response = json.loads(r.json()["response"])

    print(response)

    for resp in response:
        selectFunction(resp)
    return response

# 文本上传
def postOllama(text):
    print("开始解析")
    r = requests.post(url=ollma_url, json={
        "model": ollama_lib.OLLAMA_MODEL,
        "prompt": text,
        # "prompt": "前进到点A",
        "context": ollama_lib.CONTEXT_ALL_THE_THING,
        "stream": False
    }, timeout=90)
    response = json.loads(r.json()["response"])

    print(response)

    for resp in response:
        selectFunction(resp)
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
    
    print("### 开始语音识别 ###")

    # 进行语音识别
    recognized_text = speechRecognizer(file_path)

    # 删除临时文件
    os.remove(file_path)

    # 返回识别结果
    return jsonify({"text": recognized_text})

# 实时语音识别
def voskWeak():
    global stream, vosk_rec
    while True:
        data = stream.read(stream_chunk_size)
        if len(data) == 0:
            break
        if vosk_rec.AcceptWaveform(data):
            result = vosk_rec.Result()
            text = eval(result)["text"]
            # print(text)
            if wake_word in text:
                print("唤醒词已检测到！")
                playsound("/home/ros_system/ros_workspace/src/robot_flask/resources/imhere.mp3")
                print("开始录音")
                full_text = []
                last_voice_time = time.time()
                
                while True:
                    data = stream.read(stream_chunk_size)
                    
                    # 检测是否有语音输入
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    current_rms = np.max(audio_data)
                    silence_threshold = 1500  # 静音阈值，根据实际情况调整
                    # print(current_rms)
                    if current_rms > silence_threshold:
                        last_voice_time = time.time()
                        if vosk_rec.AcceptWaveform(data):
                            partial_result = eval(vosk_rec.Result())["text"]
                            # if partial_result:
                            #     print(f"识别中: {partial_result}", end='\r')
                    else:
                        # 超过2秒静音则结束录音
                        if time.time() - last_voice_time > 2:
                            break
                
                # 获取最终识别结果
                final_result = eval(vosk_rec.FinalResult())["text"].replace('我 在', '')
                final_result = "  请  前 往  第 一 站 "
                if final_result:
                    playsound("/home/ros_system/ros_workspace/src/robot_flask/resources/yesir.mp3")
                    print(f"\n语音识别结果: {final_result}")
                    postOllama(final_result)
                    # #模糊匹配
                    # choice_result = fuzzy_match(final_result, choices_list)
                    # if choice_result[1] > 80:
                    #     print(f"\n模糊匹配结果: {choice_result[0]}")
                    # else:
                    #     print("无模糊匹配结果")
                else:
                    print("\n没有检测到有效语音")

def main(args=None):
    # move_node = threading.Thread(target=nodeBuilder, args=(MoveControlPublisher,))
    # move_node.start()
    # camera_node = threading.Thread(target=nodeBuilder, args=(CameraImagePublisher,))
    # camera_node.start()
    # global stream
    # stream = p.open(format=pyaudio.paInt16,
    #             channels=1,
    #             rate=stream_sample_rate,
    #             input=True,
    #             frames_per_buffer=stream_chunk_size)
    # vosk_node = threading.Thread(target=voskWeak, args=())
    # vosk_node.start()
    app.run(host=RASP_HOST, port=RASP_PORT)

if __name__ == '__main__':
    main()

import os
import pyaudio
from vosk import Model, KaldiRecognizer
import time
import numpy as np
from fuzzywuzzy import fuzz, process
from playsound import playsound
import rclpy
import requests
import nav2_inter
from nav2_simple_commander.robot_navigator import BasicNavigator
import ollama_lib
import json

# 配置参数
model_path = "/home/ros_system/ros_workspace/src/robot_flask/models/vosk-model-small-cn-0.22"  # 替换为你下载的模型路径
sample_rate = 48000
chunk_size = 128

# 检查模型是否存在
if not os.path.exists(model_path):
    print("请下载模型并解压到指定路径。")
    exit(1)

# 初始化 PyAudio
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=sample_rate,
                input=True,
                frames_per_buffer=chunk_size)
stream.stop_stream()

# 初始化模型和识别器
model = Model(model_path)
rec = KaldiRecognizer(model, sample_rate)

# 模糊匹配
def fuzzy_match(target, choices):
    if not target:
        return ["", 0]
    best_match = process.extractOne(target, choices)
    return best_match

# 指令序列 
command_list = [
    "前进到位置一，然后移动到位置二"
]

# 唤醒序列

wake_list = [
    "机器人"
]

# 单点导航
def nav2pose(data):
    pose = ''
    if data == '一': 
        pose = {
            "x": 2.0,
            "y": 1.0,
            "w": 1.0
        }
    elif data == '二': 
        pose = {
            "x": -1.0,
            "y": 1.0,
            "w": 1.0
        }
    else: 
        return 0
    rclpy.init()
    navigator = BasicNavigator()
    # 等待导航启动完成
    print("### 等待导航服务器 ###")
    navigator.waitUntilNav2Active()
    nav2_inter.navToPoseOnce(navigator, pose, overtime=600.0)
    rclpy.shutdown()
    return 1

def selectFunction(response):
    rt = response["type"]
    rd = response["data"]

    print(rt + "(" + str(rd) + ")")

    if rt == "navToPose":
        nav2pose(rd)
    elif rt == "feedback":
        print("检测到非法指令，请重试！")
    return 0

try:
    print("### 开始监听唤醒词 ###")
    stream.start_stream()
    while True:
        data = stream.read(chunk_size)
        if np.max(np.frombuffer(data, dtype=np.int16)) < 1500:
            continue
        if rec.AcceptWaveform(data):
            result = rec.Result()
            text = eval(result)["text"]
            if text:
                print("识别中")
            # print(text)
            # if wake_word in text:
            if fuzzy_match(text, wake_list)[1] > 20:
                print("唤醒词已检测到！")
                playsound("/home/ros_system/ros_workspace/src/robot_flask/resources/imhere.mp3")
                # 在这里添加后续语音识别，并输出识别结果
                print("### 开始录音 ###")
                full_text = []
                last_voice_time = time.time()
                limit_time = last_voice_time
                while True:
                    # 最长输入时间
                    if time.time() - limit_time > 8:
                        break
                    data = stream.read(chunk_size)
                    # 检测是否有语音输入
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    current_rms = np.max(audio_data)
                    silence_threshold = 1500  # 静音阈值，根据实际情况调整
                    # print(current_rms)
                    if current_rms > silence_threshold:
                        last_voice_time = time.time()
                        if rec.AcceptWaveform(data):
                            partial_result = eval(rec.Result())["text"]
                            # if partial_result:
                            #     print(f"识别中: {partial_result}", end='\r')
                    else:
                        # 超过2秒静音则结束录音
                        if time.time() - last_voice_time > 2:
                            break
                # 获取最终识别结果
                final_result = eval(rec.FinalResult())["text"].replace('我 在', '')
                if final_result:
                    playsound("/home/ros_system/ros_workspace/src/robot_flask/resources/yesir.mp3")
                    final_result = fuzzy_match(final_result, command_list)[0]
                    print(f"\n语音识别结果: {final_result}")
                    print("### 开始解析 ### ")
                    stream.stop_stream()
                    req = requests.post(url=ollama_lib.OLLAMA_URL + "/api/generate", json={
                        "model": ollama_lib.OLLAMA_MODEL,
                        "prompt": final_result,
                        # "prompt": "前进到点A",
                        "context": ollama_lib.CONTEXT_ALL_THE_THING,
                        "stream": False,
                        "keep_alive": 10
                    }, timeout=90)
                    print(req)
                    response = json.loads(req.json()["response"])
                    print(response)
                    for resp in response:
                        selectFunction(resp)
                    stream.start_stream()
                else:
                    print("\n没有检测到有效语音")

except KeyboardInterrupt:
    print("### 停止监听 ### ")
finally:
    # 清理资源
    stream.stop_stream()
    stream.close()
    p.terminate()
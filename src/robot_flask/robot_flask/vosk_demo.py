import wave
import sys
import json

from vosk import Model, KaldiRecognizer, SetLogLevel

# You can set log level to -1 to disable debug messages
SetLogLevel(-1)

wf = wave.open(sys.argv[1], "rb")
if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
    print("Audio file must be WAV format mono PCM.")

    sys.exit(1)

# model = Model(lang="en-us")
# You can also init model by name or with a folder path
# model = Model(model_name="vosk-model-en-us-0.21")
# 设置模型所在路径，刚刚4.1中解压出来的路径   《《《《
# model = Model("model") 
model = Model("model-small")

rec = KaldiRecognizer(model, wf.getframerate())
rec.SetWords(True)
# rec.SetPartialWords(True)   # 注释这行   《《《《

str_ret = ""

while True:
    data = wf.readframes(4000)
    if len(data) == 0:
        break
    if rec.AcceptWaveform(data):
        result = rec.Result()
        # print(result)

        result = json.loads(result)
        if 'text' in result:
            str_ret += result['text'] + ' '
    # else:
    #     print(rec.PartialResult())

# print(rec.FinalResult())
result = json.loads(rec.FinalResult())
if 'text' in result:
    str_ret += result['text']

print(str_ret)

import requests
import ollama_lib
import json

# req = requests.post(url="http://192.168.240.152:11434/api/generate", json={
#     "model": "wangshenzhi/llama3-8b-chinese-chat-ollama-q4",
#     "prompt": "接下来请你按照要求回复我，要求如下：我会向你发送\"机器人，向前方前进2米\"、\"机器人，向左转30度\"、\"机器人，请前往位置一\"等类似的消息，你需要使用\"goahead\"，\"goback\"，\"turnleft\"，\"turenright\"，\"navToPose\"，\"stop\"，\"feedback\"其中的函数回答我。例如，我发送\"机器人，向前方前进2米\"，你回复\"[{\"T\": 0,\"type\":\"goahead\",\"data\": 2}]\";我发送\"机器人，向左转30度\"，你回复\"[{\"T\": 0,\"type\":\"turnleft\",\"data\": 30}]\";我发送\"机器人，你先去位置一，再移动到位置二\"，你回复\"[{\"T\": 0,\"type\":\"navToPose\",\"data\": \"一\"}, {\"T\": 1,\"type\":\"navToPose\",\"data\": \"二\"}]\";我发送\"机器人，你先去位置三，再后退3米，最后停下\"，你回复\"[{\"T\": 0,\"type\":\"navToPose\",\"data\": \"三\"}, {\"T\": 1,\"type\":\"goback\",\"data\": 3}, {\"T\": 2,\"type\":\"stop\",\"data\": 0}]\";请严格按照我给你的例子回复！括号外不能出现任何东西！。如果你不知道回复什么，那就回复\"[{\"T\": 0,\"type\":\"feedback\",\"data\": \"非法指令\"}]\"",
#     "stream": False,
#     "keep_alive": 10
# }, timeout=90)
# print(req.json()["context"])

# req = requests.post(url=ollama_lib.OLLAMA_URL + "/api/generate", json={
#     "model": ollama_lib.OLLAMA_MODEL,
#     # "prompt": final_result,
#     "prompt": "前进到位置一",
#     "context": ollama_lib.CONTEXT_ALL_THE_THING,
#     "stream": False,
#     "keep_alive": 10
# }, timeout=90)
print(req.json())
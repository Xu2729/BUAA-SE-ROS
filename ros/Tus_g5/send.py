import requests
import json

url = 'http://39.105.118.198:8500/api/detect/upload'
filename = "/home/robot/tmp.jpg"
files = {"file": (filename, open(filename, "rb"), "image/jpg", {})}
res = requests.post(url=url, files=files)
print(res.text)
res = json.loads(res.text)
if res["fire"]:
    result = 1
elif res["smoke"]:
    result = 2
elif res["stranger"]:
    result = 3
else:
    result = 0
with open("/home/robot/post_result.txt", "w") as f:
    f.write(chr(48 + result))
    
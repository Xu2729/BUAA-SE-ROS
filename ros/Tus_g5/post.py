import requests
import yaml
import sys
name = str(sys.argv[1])
with open('/home/robot/maps/' + name + '.yaml', 'r', encoding='utf8') as file:
    f = yaml.safe_load(file)
ff = f
url = 'http://39.105.118.198:8500/api/mapping/origin'
body = {'x': ff['origin'][0], 'y': ff['origin'][1], 'name': str(sys.argv[1])}
req = requests.post(url, json=body)
print(body)
print(req.text)

import time
from PIL import Image
import torch
from torch import nn
from torchvision import transforms
from efficientnet_pytorch import FireSmokeEfficientNet
import collections

NOTHING_DETECT = 0
FIRE_DETECT = 1
SMOKE_DETECT = 2

model_para = collections.OrderedDict()
model = FireSmokeEfficientNet.from_arch('efficientnet-b0')
model._fc = nn.Linear(1280, 3)
modelpara = torch.load('./checkpoint.pth.tar', map_location='cpu')
for key in modelpara['state_dict'].keys():
    model_para[key[7:]] = modelpara['state_dict'][key]
model.load_state_dict(model_para)
# Preprocess image
tfms = transforms.Compose([transforms.Resize(224), transforms.ToTensor(),
                           transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]), ])

# Load ImageNet class names
labels_map = ["fire", "negative", "smoke"]
# Classify
model.eval()


def predict_fire_and_smoke(file, debug):
    with torch.no_grad():
        st = time.time()
        image = Image.open(file)
        img = tfms(image).unsqueeze(0)
        outputs = model(img)
        prob_fire = torch.softmax(outputs, dim=1)[0, 0].item()
        prob_smoke = torch.softmax(outputs, dim=1)[0, 2].item()
        if debug:
            print("Detect fire and smoke use: %.2fms" % (1000 * (time.time() - st)))
            print("fire: %.2f, smoke: %.2f, nothing: %.2f" % (prob_fire, prob_smoke, 1 - prob_smoke - prob_fire))
        if prob_fire > 0.5:
            return FIRE_DETECT
        if prob_smoke > 0.5:
            return SMOKE_DETECT
        return NOTHING_DETECT

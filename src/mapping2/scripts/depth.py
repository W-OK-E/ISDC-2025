import cv2
import torch
import os
import numpy as np
import matplotlib.pyplot as plt

import numpy as np
from scipy.ndimage import gaussian_filter

from depth_anything_v2.dpt import DepthAnythingV2

def get_depth(img):
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }

    encoder = 'vitb' # or 'vits', 'vitb', 'vitg'

    model = DepthAnythingV2(**model_configs[encoder])
    model.load_state_dict(torch.load(f'/media/summer/ubuntu 2tb/ISDC_MAIN/MONO_2/src/depth/checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
    model = model.to(DEVICE).eval()
    return model.infer_image(img)

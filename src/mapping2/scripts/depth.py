import torch,cv2
import argparse,os
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


def main():
    parser = argparse.ArgumentParser(
        description="Script to convert Monocular to depth image"
    )
    parser.add_argument(
        "--im_path",
        type = str,
        required = True,
        help = 'Path of the Image whose depth needs to be calculated'
    )

    args = parser.parse_args()
    fname = args.im_path
    depth_image = get_depth(cv2.imread(fname))
    
    if(os.path.exists("results")):
        os.mkdir("results")
    
    cv2.imwrite("results/depth.png",depth_image)
if __name__ == "__main__":
    main()


import os
from tqdm import tqdm
import subprocess

file_name = "rabbit" 
input_dir = f"{os.getcwd()}/jpgs/{file_name}" 
output_format = "{:04d}.jpg"
fps = 60
jpg_files = sorted([
    f for f in os.listdir(input_dir)
    if f.lower().endswith('.jpg')
])

for idx, filename in enumerate(tqdm(jpg_files), start=1):
    src = os.path.join(input_dir, filename)
    dst = os.path.join(input_dir, output_format.format(idx))
    os.rename(src, dst)

cmd = [
    "ffmpeg",
    "-framerate", "60",
    "-i", "%04d.jpg",
    "-c:v", "hevc_nvenc",
    "-preset", "p5",
    "-pix_fmt", "yuv420p",
    f"{file_name}.mp4"
]

# ©I¥s ffmpeg ¨ÃÀË¬d¿ù»~
result = subprocess.run(cmd, capture_output=True, text=True)
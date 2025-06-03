
import os
from tqdm import tqdm
import subprocess
import re
import shutil

file_name = "luca"
input_dir = os.path.join(os.getcwd(), "jpgs", file_name)
subfolder = os.path.join(input_dir, "1000")
output_format = "{:04d}.jpg"  

if os.path.exists(subfolder):
    for f in os.listdir(subfolder):
        if f.lower().endswith(".jpg"):
            match = re.search(r"(\d+)", f)
            if match:
                number = int(match.group(1))
                new_number = number + 1000
                new_name = f"DSC_{output_format.format(new_number)}"
                
                src = os.path.join(subfolder, f)
                dst = os.path.join(input_dir, new_name)

                print(f"Moving: {src} -> {dst}")
                shutil.move(src, dst)
            else:
                print(f"Skipped file (no number found): {f}")

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
    "-i", f"{input_dir}/%04d.jpg",
    "-c:v", "hevc_nvenc",
    "-preset", "p5",
    "-pix_fmt", "yuv420p",
    f"{input_dir}/{file_name}.mp4"
]
subprocess.run(cmd, stdout=None, stderr=None)
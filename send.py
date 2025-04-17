import socket,time
import os,cv2
from tqdm import tqdm,trange
import numpy as np
import struct
import pyautogui
process_file = "avem_test3"
esp32_ip = "192.168.4.1"
port = 80
output_video = f"{os.getcwd()}\\output\\{process_file}_vid.txt"
output_audio = f"{os.getcwd()}\\output\\{process_file}_aud.txt"
def reverse_bits(n, bit_length=8):
    return int(f'{n:08b}'[::-1], 2) 
def read_video():
    _f = open(output_video,'r')
    _data = _f.read()
    _f.close()
    _output = [bytes([reverse_bits(int(_y)) for _y in _x.split("-")]) for _x in _data.split('+')]
    return _output
def read_audio():
    _f = open(output_audio,'r')
    _data = _f.read()
    _f.close()
    _output = [bytes([int(_y) for _y in _x.split("-")]) for _x in _data.split('+')]
    return _output
def local_show(_video_frame):
    image_data_view = []
    image_row = []
    counter = 0
    for _bytes in _video_frame:
        for bit in range(8):
            image_row.append(255 if (_bytes >> (bit)) & 1 else 0)
            counter += 1
            if counter>=128:
                counter = 0
                image_data_view.append(image_row)
                image_row = []
                    
    image_data = np.array(image_data_view, dtype=np.uint8)
    resized_image = cv2.resize(image_data, (image_data.shape[1]*10, image_data.shape[0]*10), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Video Frame", resized_image)
    cv2.waitKey(60)
    return None
def send_to_esp32(_video_data , _audio_data):
    for _idx in trange(len(_video_data)):
        _frame_data = _video_data[_idx] + _audio_data[_idx]
        #_data = _video_data[_idx]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((esp32_ip, port))
        while True:
            _data = 'NA'
            _data = s.recv(1024).decode()
            if _data == 'n':
                s.sendall(_frame_data)
                s.close()
                break
            else:
                time.sleep(0.03)
                continue
        #local_show(_video_frame)
        time.sleep(0.05)
    return None
def screenshot_TFT():
    def rgb888_to_rgb565(image):
        r = (image[:, :, 0] >> 3).astype(np.uint16)
        g = (image[:, :, 1] >> 2).astype(np.uint16)
        b = (image[:, :, 2] >> 3).astype(np.uint16)
        return (r << 11) | (g << 5) | b
    screenshot = pyautogui.screenshot()
    frame = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)
    target_width = 1920
    target_height = 1080
    h, w, _ = frame.shape
    left = (w - target_width) // 2
    top = (h - target_height) // 2
    cropped = frame[top:top+target_height, left:left+target_width]
    rotated = cv2.rotate(cropped, cv2.ROTATE_90_CLOCKWISE)

    resized = cv2.resize(rotated, (128, 160), interpolation=cv2.INTER_AREA)
    rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    rgb565_image = rgb888_to_rgb565(rgb_image)
    rgb565_flat = rgb565_image.flatten()
    return rgb565_flat
def screenshot_toesp32():
    while True:
        frame_data = screenshot_TFT()
        frame_bytes = frame_data.tobytes()  
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)  
            s.connect((esp32_ip, port))
            while True:
                try:
                    received = s.recv(1024).decode()
                    if received.strip() == 'n':
                        s.sendall(frame_bytes)
                        s.close()
                        print("graph sent")
                        break
                    else:
                        time.sleep(0.03)
                except socket.timeout:
                    print("Timeout ESP32")
                    s.close()
                    break
        except Exception as e:
            print(f"{e}")
        time.sleep(0.05)
    return None

if __name__ == "__main__":
    screenshot_toesp32()
    

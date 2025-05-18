import socket,time
import os,cv2
from tqdm import tqdm,trange
import numpy as np
import struct
import pyautogui

esp32_ip = "192.168.4.1"
port = 80

class sand_drawer:
    send_length = 2048
    motor_count = 2
    def __init__(self):
        self.data = [
            ([10, 20, -30, 40, -50, 60, -70, 80, -90, 100, 110],
             [10, 20, -30, 40, -50, 60, -70, 80, -90, 100, 110])
        ]
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        self.iterator = 0
    @staticmethod
    def to_send(axisdata: list[int], armdata: list[int]):
        axis = axisdata + [0] * (sand_drawer.send_length - len(axisdata))
        arm  = armdata + [0] * (sand_drawer.send_length  - len(armdata))
        combined = axis + arm
        return np.array(combined, dtype=np.int32).tobytes()

    def send(self):
        while self.iterator<len(self.data):
            frame_bytes = self.send_data[self.iterator] 
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3)  
                s.connect((esp32_ip, port))
                while True:
                    try:
                        received = s.recv(1024).decode()
                        if received.strip() == 'S':
                            s.sendall(frame_bytes)
                            s.close()
                            self.iterator += 1
                            print("byte sent")
                            break
                        else:
                            time.sleep(0.03)
                    except socket.timeout:
                        print("Timeout ESP32")
                        s.close()
                        break
            except Exception as e:
                print(f"{e}")
        return None

if __name__ == "__main__":
    graph = sand_drawer()
    graph.send()





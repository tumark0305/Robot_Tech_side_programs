import socket,time
import numpy as np
from tqdm import tqdm

esp32_ip = "192.168.4.1"
port = 80

send_length = 2048



def to_send(axisdata: list[int], armdata: list[int]):
    axis = list(axisdata) + [0] * (send_length - len(axisdata))
    arm  = list(armdata) + [0] * (send_length  - len(armdata))
    combined = axis + arm
    return np.array(combined, dtype=np.int32).tobytes()
def send():
    iterator = 0
    pbar = tqdm(total=len(send_data), desc="Sending")
    while iterator<len(send_data):
        frame_bytes = send_data[iterator] 
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(600)  
            s.connect((esp32_ip, port))
            while True:
                try:
                    received = s.recv(1024).decode()
                    if received.strip() == 'S':
                        s.sendall(frame_bytes)
                        s.close()
                        iterator += 1
                        pbar.update(1)
                        break
                    else:
                        time.sleep(1)
                except socket.timeout:
                    print("Timeout ESP32")
                    # s.close()
                    break
        except Exception as e:
            print(f"{e}")
            continue
    return None

data = [
            ([2000] + [20000]*2000,
             [2000] + [-20000]*2000)
        ]
send_data = [to_send(x,r) for x,r in data]

if __name__ == "__main__":
    send()



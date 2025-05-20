import socket,time
import os,cv2
from tqdm import tqdm,trange
import numpy as np
import struct
import pyautogui

esp32_ip = "192.168.4.1"
port = 80

class sand_drawer:
    arm_length = [90,90]
    max_step = 2000
    send_length = 2048
    motor_count = 2
    epsilon=1.0 # epsilon 越小越精細
    image_size = 1024
    def __init__(self , _image_path:str):
        self.image = cv2.resize(cv2.imread(f"{os.getcwd()}/input/{_image_path}"), (sand_drawer.image_size, sand_drawer.image_size), interpolation=cv2.INTER_NEAREST)
        self.circle_border()
        self.lines = []
        self.data = [
            ([10, 200, -30, 40, -50, 60, -70, 80, -90, 100, 110],
             [10, 200, -30, 40, -50, 60, -70, 80, -90, 100, 110])
        ]
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        self.iterator = 0
    @staticmethod
    def to_send(axisdata: list[int], armdata: list[int]):
        axis = axisdata + [0] * (sand_drawer.send_length - len(axisdata))
        arm  = armdata + [0] * (sand_drawer.send_length  - len(armdata))
        combined = axis + arm
        return np.array(combined, dtype=np.int32).tobytes()
    def test(self):
        self.image = np.ones((sand_drawer.image_size, sand_drawer.image_size, 3), dtype=np.uint8) * 255
        cv2.line(self.image, (sand_drawer.image_size // 2, 0), (sand_drawer.image_size // 2, sand_drawer.image_size), (0, 0, 0), 1)
        self.circle_border()
        return None
    def preview(self):
        preview = np.ones_like(self.image) * 255
        for _pack in self.lines:
            cv2.line(preview, tuple(_pack[0].astype(int)), tuple(_pack[1].astype(int)), (0, 0, 0), 1)
        height, width = preview.shape[:2]
        radius = int(sand_drawer.max_step/4)
        center = (int(width / 2), int(height / 2))
        cv2.circle(preview, center, radius, (0, 0, 255), 1)

        cv2.imshow("Line Preview", preview)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return None
    def get_line(self):
        self.lines = []
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, epsilon=sand_drawer.epsilon, closed=True)
            for i in range(len(approx)-1):
                x1, y1 = approx[i][0]
                x2, y2 = approx[(i + 1) % len(approx)][0]
                _pack = np.array([[x1, y1],[x2, y2]])
                self.lines.append(_pack)
        return None
    def circle_border(self):
        mask = np.zeros((sand_drawer.image_size, sand_drawer.image_size), dtype=np.uint8)
        center = (sand_drawer.image_size // 2, sand_drawer.image_size // 2)
        radius = int(sand_drawer.max_step / 4) - 5
        cv2.circle(mask, center, radius, 255, -1) 
        white_bg = np.ones_like(self.image, dtype=np.uint8) * 255
        self.image = np.where(mask[:, :, None] == 255, self.image, white_bg)
        return None
    def convert(self):
        self.data = []
        def inverse_kinematics(x, y):
            L1 = 1000
            L2 = 1000
            D = np.hypot(x, y)
    
            if D > (L1 + L2): return None  # 點超出範圍

            cos_angle2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
            angle2 = np.arccos(np.clip(cos_angle2, -1.0, 1.0))

            k1 = L1 + L2 * np.cos(angle2)
            k2 = L2 * np.sin(angle2)
            angle1 = np.arctan2(y, x) - np.arctan2(k2, k1)

            return np.array([angle1, angle2])
        for lines in self.lines:
            resolution = sand_drawer.send_length -1
            discrete_line = np.linspace(lines[0],lines[1],resolution)
            theta = []
            for ds in discrete_line:
                dtheta = inverse_kinematics(ds[0], ds[1])
                theta.append(np.round(dtheta* 1000 / np.pi) )
            theta_axis,theta_arm = [len(theta)],[len(theta)]
            current_point = theta[0]
            for next_point in theta:
                vector = next_point - current_point
                if vector[0] >=0:
                    theta_axis.append(int(vector[0]))
                else:
                    theta_axis.append(-int(vector[0]))
                if vector[1] >=0:
                    theta_arm.append(int(vector[1]))
                else:
                    theta_arm.append(-int(vector[1]))
                current_point = next_point
            self.data.append((theta_axis , theta_arm))
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        return None
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
    def find_path(self):
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        def get_sorted_path_by_linesorter(contours):
            def dist(p1, p2):
                return np.linalg.norm(np.array(p1) - np.array(p2))

            # 將輪廓轉為線段列表
            segments = []
            for cnt in contours:
                pts = [tuple(p[0]) for p in cnt]
                for i in range(1, len(pts)):
                    segments.append([pts[i - 1], pts[i]])

            visited = [False] * len(segments)
            path = []

            # 找到最靠近中心的線段作為起點
            image_center = np.array([self.image.shape[1] / 2, self.image.shape[0] / 2])
            min_dist = float('inf')
            current_idx = 0
            for i, seg in enumerate(segments):
                mid_point = (np.array(seg[0]) + np.array(seg[1])) / 2
                d = np.linalg.norm(mid_point - image_center)
                if d < min_dist:
                    min_dist = d
                    current_idx = i

            visited[current_idx] = True
            path.append(segments[current_idx])
            current_tail = segments[current_idx][1]

            progress = tqdm(total=len(segments) - 1, desc="Connecting segments")
            while not all(visited):
                best_idx = None
                best_reverse = False
                best_distance = float('inf')

                for i, seg in enumerate(segments):
                    if visited[i]:
                        continue
                    s, e = seg
                    d1 = dist(current_tail, s)
                    d2 = dist(current_tail, e)

                    if d1 < best_distance:
                        best_idx = i
                        best_reverse = False
                        best_distance = d1
                    if d2 < best_distance:
                        best_idx = i
                        best_reverse = True
                        best_distance = d2

                # 插入補線段
                next_seg = segments[best_idx]
                next_start = next_seg[1] if best_reverse else next_seg[0]
                if dist(current_tail, next_start) > 0:
                    path.append([current_tail, next_start])

                # 接上下一段
                if best_reverse:
                    path.append([next_seg[1], next_seg[0]])
                    current_tail = next_seg[0]
                else:
                    path.append([next_seg[0], next_seg[1]])
                    current_tail = next_seg[1]

                visited[best_idx] = True
                progress.update(1)

            return [np.array([p1, p2]) for p1, p2 in path]
        print("running...")
        self.lines = get_sorted_path_by_linesorter(contours)
        return None


if __name__ == "__main__":
    graph = sand_drawer("anon.jpg")
    graph.test()
    graph.get_line()
    #graph.find_path()
    graph.preview()
    graph.convert()
    
    graph.send()





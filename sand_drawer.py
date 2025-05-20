import socket,time
import os,cv2
from tqdm import tqdm,trange
import numpy as np
import struct
import pyautogui
import sympy as sp

esp32_ip = "192.168.4.1"
port = 80

class coordinate:
    arm_length = 1000
    axis_length = 1000
    def __init__(self):
        self.t = sp.symbols('t', real=True)
        self.x = 0
        self.y = 0
        self.sol1 = None
        self.sol2 = None

    def inverse_kinematics_all(x, y):
        L1 = 1000
        L2 = 1000
        D = np.hypot(x, y)

        if D > (L1 + L2):
            return None  # 點超出工作範圍

        cos_angle2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_angle2 = np.clip(cos_angle2, -1.0, 1.0)

        # === 第一組解（彎肘）
        angle2_1 = np.arccos(cos_angle2)
        k1 = L1 + L2 * np.cos(angle2_1)
        k2 = L2 * np.sin(angle2_1)
        angle1_1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        # === 第二組解（伸肘）
        angle2_2 = -angle2_1
        k1 = L1 + L2 * np.cos(angle2_2)
        k2 = L2 * np.sin(angle2_2)
        angle1_2 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return np.array([[angle1_1, angle2_1], [angle1_2, angle2_2]])
    def gen_function(self):
        L1 = self.axis_length
        L2 = self.arm_length
        x = self.x
        y = self.x
        D2 = x**2 + y**2
        cos_theta2 = (D2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2_down = sp.acos(cos_theta2)       # 彎肘
        theta2_up   = -sp.acos(cos_theta2)      # 伸肘
        k1_down = L1 + L2 * sp.cos(theta2_down)
        k2_down = L2 * sp.sin(theta2_down)
        theta1_down = sp.atan2(y, x) - sp.atan2(k2_down, k1_down)

        k1_up = L1 + L2 * sp.cos(theta2_up)
        k2_up = L2 * sp.sin(theta2_up)
        theta1_up = sp.atan2(y, x) - sp.atan2(k2_up, k1_up)

        self.sol1 = (sp.simplify(theta1_down),sp.simplify(theta2_down))
        self.sol2 = (sp.simplify(theta1_up),sp.simplify(theta2_up))

        sp.pprint(self.sol1[0], use_unicode=True)
        sp.pprint(self.sol1[1], use_unicode=True)
        sp.pprint(self.sol2[0],   use_unicode=True)
        sp.pprint(self.sol2[1],   use_unicode=True)

        return None
    def convert(self,two_pin):
        start_sol = coordinate.inverse_kinematics_all(two_pin[0][0],two_pin[0][1]) * 1000 / np.pi
        end_sol = coordinate.inverse_kinematics_all(two_pin[1][0],two_pin[1][1]) * 1000 / np.pi
        quene = [(start_sol[0],end_sol[0]),(start_sol[0],end_sol[1]),(start_sol[1],end_sol[0]),(start_sol[1],end_sol[1])]
        for start_point,end_point in quene:
            theta1_end = int(max(start_point[0] , end_point[0]))
            theta2_end = int(max(start_point[1] , end_point[1]))
            theta1_start = int(min(start_point[0] , end_point[0]))
            theta2_start = int(min(start_point[1] , end_point[1]))
            theta1_time_step = range(theta1_start , theta1_end)
            theta2_time_step = range(theta2_start , theta2_end)
            for time1 in range(theta1_start , theta1_end):
                eq = sp.Eq(self.sol1[0], time1)
                sol = sp.solve(eq, self.t)
                pass#解出每個theta12_time_step的時間戳存成序列


        return None

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
        calculator = coordinate()
        calculator.x = calculator.t
        calculator.gen_function()
        self.data = []
        def inverse_kinematics_all(x, y):
            L1 = 1000
            L2 = 1000
            D = np.hypot(x, y)

            if D > (L1 + L2):
                return None  # 點超出工作範圍

            cos_angle2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
            cos_angle2 = np.clip(cos_angle2, -1.0, 1.0)

            # === 第一組解（彎肘）
            angle2_1 = np.arccos(cos_angle2)
            k1 = L1 + L2 * np.cos(angle2_1)
            k2 = L2 * np.sin(angle2_1)
            angle1_1 = np.arctan2(y, x) - np.arctan2(k2, k1)

            # === 第二組解（伸肘）
            angle2_2 = -angle2_1
            k1 = L1 + L2 * np.cos(angle2_2)
            k2 = L2 * np.sin(angle2_2)
            angle1_2 = np.arctan2(y, x) - np.arctan2(k2, k1)

            return np.array([[angle1_1, angle2_1], [angle1_2, angle2_2]])
        for lines in self.lines:
            calculator.convert(lines)
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





import socket,time
import os,cv2,math
from tkinter import Y
from tqdm import tqdm,trange
import numpy as np
import struct
import pyautogui
import sympy as sp
from math import cos, sin
from scipy.optimize import minimize_scalar
from sympy import lambdify
from copy import deepcopy

esp32_ip = "192.168.4.1"
port = 80

LENGTH = 1000
STEP_PER_ROUND = 2000

class stepper_helper:
    length = LENGTH
    max_step = STEP_PER_ROUND
    def __init__(self , initial_pos):
        self.position = initial_pos
        self.run_time = 0
        self.output = []
        self.delay_time = 0
    def coordinate(self):
        theta = self.position /(self.max_step//2) * np.pi
        output = [self.length * cos(theta) , self.length * sin(theta)]
        return np.array(output)
    def __add__(self,other):
        if not isinstance(other, stepper_helper):
            return NotImplemented
        return self.coordinate() + other.coordinate()
    def delay(self,delay_time):
        self.delay_time += abs(delay_time)
        return None
    def move(self , direction:int ,time_step):
        if direction != 0:
            next_pulse = abs(time_step) + self.delay_time
            next_pulse = next_pulse * direction
            self.run_time += abs(next_pulse)
            self.output.append(next_pulse)
            self.position += direction
            self.delay_time = 0
        else:
            self.delay_time += abs(time_step)
        return None

class sand_drawer:
    pulse_range = (1000,50000)
    rander_pulse = 20000
    arm_length = [1000,1000]
    max_step = STEP_PER_ROUND
    send_length = 2048
    motor_count = 2
    epsilon=1.0 # epsilon �V�p�V���
    image_size = 4096   
    velocity = 10 #pixel/s
    def __init__(self , _image_path:str):
        self.image = cv2.resize(cv2.imread(f"{os.getcwd()}/input/{_image_path}"), (sand_drawer.image_size, sand_drawer.image_size), interpolation=cv2.INTER_NEAREST)
        self.circle_border()
        self.lines = []
        self.data = [
            ([2000] + [20000]*2000,
             [2000] + [-20000]*2000)
        ]
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        self.iterator = 0
    @staticmethod
    def to_send(axisdata: list[int], armdata: list[int]):
        axis = list(axisdata) + [0] * (sand_drawer.send_length - len(axisdata))
        arm  = list(armdata) + [0] * (sand_drawer.send_length  - len(armdata))
        combined = axis + arm
        return np.array(combined, dtype=np.int32).tobytes()
    def test(self):
        self.image = np.ones((sand_drawer.image_size, sand_drawer.image_size, 3), dtype=np.uint8) * 255
        cv2.line(self.image, (sand_drawer.image_size // 2, 500), (sand_drawer.image_size // 2, sand_drawer.image_size-500), (0, 0, 0), 1)
        self.circle_border()
        return None
    def preview(self):
        preview = np.ones_like(self.image) * 255
        for _pack in self.lines:
            cv2.line(preview, tuple(_pack[0].astype(int)), tuple(_pack[1].astype(int)), (0, 0, 0), 1)
        height, width = preview.shape[:2]
        radius = int(sand_drawer.max_step)
        center = (int(width / 2), int(height / 2))
        cv2.circle(preview, center, radius, (0, 0, 255), 2)
        out = cv2.resize(preview, (512, 512), interpolation=cv2.INTER_NEAREST)

        cv2.imshow("Line Preview", out)
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
        radius = int(sand_drawer.arm_length[0] + sand_drawer.arm_length[1]) - 5
        cv2.circle(mask, center, radius, 255, -1) 
        white_bg = np.ones_like(self.image, dtype=np.uint8) * 255
        self.image = np.where(mask[:, :, None] == 255, self.image, white_bg)
        return None
    def convert(self):
        def initial_turn():
            turn_direction = -1
            vector = self.lines[0][1] - self.lines[0][0]
            facing = int(np.arctan2(vector[1], vector[0]) *(STEP_PER_ROUND//2) / np.pi) +1
            self.data.append([[facing] + [ turn_direction * self.rander_pulse] * facing,[facing] + [turn_direction * self.rander_pulse] * facing])
            axis.position += facing * turn_direction
            arm.position += facing * turn_direction
            return None
        def clostest_time_n(_start , _end , x_p,y_p):
            distance = np.linalg.norm(_start - _end)
            x1 = (_end[0] - _start[0]) / (distance * self.velocity)
            x2 = _start[0]
            y1 = (_end[1] - _start[1]) / (distance * self.velocity)
            y2 = _start[1]
            t_min_u = x1 *x_p + y1 * y_p - x1 * x2 - y1 * y2
            t_min_d = x1 * x1 + y1 * y1
            d_min_u = abs(x1*y2 - x2*y1 - x1*y_p + x_p*y1)
            d_min_d = math.sqrt(t_min_d)
            t_min = t_min_u / t_min_d
            d_min = d_min_u / d_min_d
            return t_min, d_min#old
        def clostest_time(_start, _end, x_p, y_p):
            v = _end - _start
            # 單位速度向量
            v_unit = v * self.velocity / np.linalg.norm(v)
            x1, y1 = v_unit
            x2, y2 = _start

            # 計算最近時間點 (投影公式)
            t_min_numer = x1 * x_p + y1 * y_p - x1 * x2 - y1 * y2
            t_min_denom = x1 * x1 + y1 * y1
            t_min = t_min_numer / t_min_denom

            # 最短距離 (叉積法)
            dx = _end[0] - _start[0]
            dy = _end[1] - _start[1]
            px = x_p - _start[0]
            py = y_p - _start[1]
            cross = abs(dx * py - dy * px)
            d_min = cross / np.hypot(dx, dy)

            return t_min, d_min#gpt
        def kinematics(theta1,theta2):
            theta1 = theta1 * np.pi / int(sand_drawer.max_step //2 )
            theta2 = theta2 * np.pi / int(sand_drawer.max_step //2 )
            x = sand_drawer.arm_length[0] * cos(theta1) + sand_drawer.arm_length[1] * cos(theta2)
            y = sand_drawer.arm_length[0] * sin(theta1) + sand_drawer.arm_length[1] * sin(theta2)
            return np.array([x,y])
        axis = stepper_helper(0)
        arm = stepper_helper(STEP_PER_ROUND//2)
        self.data = []
        initial_turn()
        now_coordinate = axis + arm
        offset = sand_drawer.image_size//2
        if self.lines[0][0][0] != int(now_coordinate[0] + offset) or self.lines[0][0][1] != int(now_coordinate[1] + offset):
            raise ValueError(f"{self.lines[0][0][0]} != {int(now_coordinate[0] + offset)} or {self.lines[0][0][1]} != {int(now_coordinate[1] + offset)} : initial coordinate is different.")
        for i,lines in enumerate(self.lines):
            axis.output = []
            arm.output = []
            current_time = 0
            start_point = lines[0] - offset
            end_point = lines[1] - offset
            choose = [(0,1,1),(1,1,0),(2,1,-1),(3,0,1),(4,0,-1),(5,-1,1),(6,-1,0),(7,-1,-1)]
            while True:
                min_distance = np.inf
                chosen = 0
                time_at = 0
                data_pack = []
                for at,add1,add2 in choose:
                    axis_position = axis.position + add1
                    arm_position = arm.position + add2
                    coord = kinematics(axis_position , arm_position)
                    t_min , d_min = clostest_time(start_point , end_point , coord[0] , coord[1])
                    data_pack.append([at, t_min - current_time , d_min])
                    if d_min < min_distance and sand_drawer.pulse_range[1] >(t_min - current_time) > 0:
                        min_distance = d_min
                        chosen = at
                        time_at = t_min - current_time
                current_time += time_at
                time_at = time_at * 1e6
                if time_at < sand_drawer.pulse_range[0] or time_at > sand_drawer.pulse_range[1]:
                    time_at = sand_drawer.rander_pulse

                axis.move(choose[chosen][1] , time_at)
                arm.move(choose[chosen][2] , time_at)
                current_coordinate = axis + arm
                #current_coordinate = [int(current_coordinate[0]) , int(current_coordinate[1])]
                vec_target = end_point - start_point
                vec_now = current_coordinate - start_point
                if choose[chosen][1] == choose[chosen][2]:
                    pass
                if np.dot(vec_now, vec_target) >= np.dot(vec_target, vec_target):
                    break
            axis_data = axis.output.copy()
            arm_data = arm.output.copy()
            min_value = min(axis_data + arm_data , key=abs)
            max_value = max(axis_data + arm_data , key=abs)
            if abs(min_value)<sand_drawer.pulse_range[0] or abs(max_value)>sand_drawer.pulse_range[1]:
                print(f"{min_value = } | {max_value = } \n expect: {sand_drawer.pulse_range[0] = } | {sand_drawer.pulse_range[1] = }")
            axis_output = [int(_) for _ in axis_data]
            arm_output = [int(_) for _ in arm_data]
            if any([_ == 0 for _ in axis_output] + [_ == 0 for _ in arm_output]):
                raise ValueError(f"zero in pulse")
            axis_output.insert(0,len(axis_output))
            arm_output.insert(0,len(arm_output))
            self.data.append([axis_output , arm_output])
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        return None
    def send(self):
        while self.iterator<len(self.send_data):
            frame_bytes = self.send_data[self.iterator] 
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(240)  
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
            segments = []
            for cnt in contours:
                pts = [tuple(p[0]) for p in cnt]
                for i in range(1, len(pts)):
                    segments.append([pts[i - 1], pts[i]])

            visited = [False] * len(segments)
            path = []
            image_center = np.array([self.image.shape[1] / 2, self.image.shape[0] / 2])
            min_dist = float('inf')
            current_idx = 0
            for i, seg in enumerate(segments):
                mid_point = (np.array(seg[0]) + np.array(seg[1])) / 2
                d = np.linalg.norm(mid_point - image_center)
                if d < min_dist:
                    min_dist = d
                    current_idx = i

            seg = segments[current_idx]  # 拿到目前線段
            visited[current_idx] = True  # 標記造訪

            # 決定從哪個點出發：靠近中心那一邊
            if np.linalg.norm(np.array(seg[0]) - image_center) < np.linalg.norm(np.array(seg[1]) - image_center):
                path.append([(int(image_center[0]),int(image_center[1])), seg[0]])
                path.append([seg[0], seg[1]])  # 原方向
                current_tail = seg[1]
            else:
                path.append([(int(image_center[0]),int(image_center[1])), seg[1]])
                path.append([seg[1], seg[0]])  # 反轉方向
                current_tail = seg[0]

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

                next_seg = segments[best_idx]
                next_start = next_seg[1] if best_reverse else next_seg[0]
                if dist(current_tail, next_start) > 0:
                    path.append([current_tail, next_start])
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
    def save_pulse(self , fle_name:str):
        text = "@".join(["+".join(["/".join([str(_z) for _z in _y]) for _y in _x]) for _x in self.data])
        f = open(f"{os.getcwd()}/output/{fle_name}.txt" , "w")
        f.write(text)
        f.close()
        return None
    def load_pulse(self , fle_name:str):
        f = open(f"{os.getcwd()}/output/{fle_name}.txt" , "r")
        text = f.read()
        f.close()
        
        self.data = [[[int(_z) for _z in _y.split("/") if _z != ""] for _y in _x.split("+")] for _x in text.split("@")]
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        return None

New_graph = True
from sand_drawer_simulation import simulation
if __name__ == "__main__":
    graph = sand_drawer("anon.jpg")
    if New_graph:
        #graph.test()
        graph.get_line()
        graph.find_path()
        graph.preview()
        graph.convert()
        graph.save_pulse( "line" )
    else:
        graph.load_pulse( "line" )
    print("sending")
    sim = simulation("line")
    sim.data = graph.data
    sim.run()
    sim.print_result()
    sim.save()


    graph.send()





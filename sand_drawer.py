import socket,time
import os,cv2
from tkinter import Y
from tqdm import tqdm,trange
import numpy as np
import struct
import pyautogui
import sympy as sp
from math import cos, sin
from scipy.optimize import minimize_scalar
from sympy import lambdify

esp32_ip = "192.168.4.1"
port = 80

class coordinate:
    arm_length = 1000
    axis_length = 1000
    velocity = 100 #pixel/s
    def __init__(self , current_sol:int):
        self.current_sol = current_sol
        self.t = sp.symbols('t', real=True)
        self.x = 0
        self.y = 0
        self.sol1 = None
        self.sol2 = None
        self.tf = None
        self.output = None

    def inverse_kinematics_all(x, y):
        L1 = 1000
        L2 = 1000
        D = np.hypot(x, y)

        if D > (L1 + L2):
            return None  

        cos_angle2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_angle2 = np.clip(cos_angle2, -1.0, 1.0)
        angle2_1 = np.arccos(cos_angle2)
        k1 = L1 + L2 * np.cos(angle2_1)
        k2 = L2 * np.sin(angle2_1)
        angle1_1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        angle2_2 = -angle2_1
        k1 = L1 + L2 * np.cos(angle2_2)
        k2 = L2 * np.sin(angle2_2)
        angle1_2 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return np.array([[angle1_1, angle2_1], [angle1_2, angle2_2]])
    def kinematics(theta1,theta2):
        theta1 = theta1 * np.pi / 1000
        theta2 = theta2 * np.pi / 1000
        x = coordinate.axis_length * cos(theta1) + coordinate.arm_length * cos(theta2)
        y = coordinate.axis_length * sin(theta1) + coordinate.arm_length * sin(theta2)
        return np.array([x,y])
    def gen_function(self):
        L1 = self.axis_length
        L2 = self.arm_length
        x = self.x
        y = self.x
        D2 = x**2 + y**2
        cos_theta2 = (D2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2_down = sp.acos(cos_theta2)       
        theta2_up   = -sp.acos(cos_theta2)    
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
        p1 = np.array(two_pin[0])
        p2 = np.array(two_pin[1])
        distance = np.linalg.norm(p1 - p2)
        x1 = (two_pin[0][0] - two_pin[1][0]) / (distance * self.velocity)
        x2 = two_pin[0][0]
        y1 = (two_pin[0][1] - two_pin[1][1]) / (distance * self.velocity)
        y2 = two_pin[0][1]
        def clostest_time(pointx,pointy):
            t = sp.Symbol('t', real=True)
            x_t = self.t * x1 + x2
            y_t = self.t * y1 + y2
            D2 = (x_t - pointx)**2 + (y_t - pointy)**2
            dD_dt = sp.diff(D2, t)
            t_min_candidates = sp.solve(dD_dt, t)
            t_min_real = [tt.evalf() for tt in t_min_candidates if tt.is_real]
            min_dist_squared = min([D2.subs(t, tt).evalf() for tt in t_min_real])
            min_distance = sp.sqrt(min_dist_squared).evalf()
            return t_min_real[0],min_distance
        def clostest_time_faster(pointx,pointy):
            t = sp.Symbol('t', real=True)
            x_t = self.x
            y_t = self.y
            D2 = (x_t - pointx)**2 + (y_t - pointy)**2
            D2_func = lambdify(t, D2, modules='numpy')
            res = minimize_scalar(D2_func, bounds=(0, 1), method='bounded')
            t_best = res.x
            d_best = np.sqrt(res.fun)
            return t_best, d_best
        def gen_time_step(start_point,end_point):
            theta1_dir = 1 if end_point[0] >= start_point[0] else -1
            theta2_dir = 1 if end_point[1] >= start_point[1] else -1
            theta1_step = np.arange(
                int(start_point[0]),
                int(end_point[0]) + theta1_dir,
                theta1_dir
            )

            theta2_step = np.arange(
                int(start_point[1]),
                int(end_point[1]) + theta2_dir,
                theta2_dir
            )
            iter1 = 0
            iter2 = 0
            theta1_time_step = []
            theta2_time_step = []
            pbar = tqdm(total=len(theta1_step) + len(theta2_step), desc="Planning")
            while (iter1<len(theta1_step) or iter2<len(theta2_step))and not (iter1==len(theta1_step)-1 and iter2==len(theta2_step)-1):
                pbar.update(1)
                who_added = []
                possible_point = []
                if iter1 + 1<len(theta1_step) :
                    possible_point.append(coordinate.kinematics(theta1_step[iter1 + 1],theta2_step[iter2]))
                    who_added.append("theta1")
                if iter2 + 1<len(theta2_step) :
                    possible_point.append(coordinate.kinematics(theta1_step[iter1],theta2_step[iter2 + 1]))
                    who_added.append("theta2")
                if iter1 + 1<len(theta1_step) and iter2 + 1<len(theta2_step) :
                    possible_point.append(coordinate.kinematics(theta1_step[iter1 + 1],theta2_step[iter2 + 1]))
                    who_added.append("both")
                error_distance = []
                error_t = []
                for real_coordinate in possible_point:
                    error = clostest_time(real_coordinate[0],real_coordinate[1])
                    error_t.append(error[0])
                    error_distance.append(error[1])
                min_index = error_distance.index(min(error_distance))
                if who_added[min_index] == "theta1":
                    theta1_time_step.append(theta1_dir * abs(error_t[min_index]))
                    iter1 += 1
                    continue
                elif who_added[min_index] == "theta2":
                    theta2_time_step.append(theta2_dir * abs(error_t[min_index]))
                    iter2 += 1
                    continue
                elif who_added[min_index] == "both":
                    theta1_time_step.append(theta1_dir * abs(error_t[min_index]))
                    theta2_time_step.append(theta2_dir * abs(error_t[min_index]))
                    iter1 += 1
                    iter2 += 1
                    continue
            return np.array(theta1_time_step),np.array(theta2_time_step)
        start_sol = coordinate.inverse_kinematics_all(two_pin[0][0],two_pin[0][1])[self.current_sol] * 1000 / np.pi
        end_sol = coordinate.inverse_kinematics_all(two_pin[1][0],two_pin[1][1]) * 1000 / np.pi
        quene = [(start_sol,end_sol[0]),(start_sol,end_sol[1])]
        _all_step = []
        for start_point,end_point in quene:
            if abs(start_point[0] - end_point[0]) >= 2047 or abs(start_point[1] - end_point[1]) >= 2047:
                continue
            _all_step.append(gen_time_step(start_point,end_point))
        best_step = []
        min_length = np.inf
        best_index = 0
        for i,select in enumerate(_all_step):
            length = len(select[0]) + len(select[1])
            if length < min_length:
                min_length = length
                best_step = select
                best_index = i
        self.current_sol = best_index
        best_step = [list(_) for _ in best_step]
        [_.insert(0,len(_)) for _ in best_step]
        
        self.output = [[int(_y) for _y in _x] for _x in best_step]
        return None

class sand_drawer:
    arm_length = [90,90]
    max_step = 2000
    send_length = 2048
    motor_count = 2
    epsilon=1.0 # epsilon �V�p�V���
    image_size = 1024   
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
        calculator = coordinate(0)
        calculator.x = calculator.t
        calculator.gen_function()
        self.data = []
        def inverse_kinematics_all(x, y):
            L1 = 1000
            L2 = 1000
            D = np.hypot(x, y)

            if D > (L1 + L2):
                return None 

            cos_angle2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
            cos_angle2 = np.clip(cos_angle2, -1.0, 1.0)
            angle2_1 = np.arccos(cos_angle2)
            k1 = L1 + L2 * np.cos(angle2_1)
            k2 = L2 * np.sin(angle2_1)
            angle1_1 = np.arctan2(y, x) - np.arctan2(k2, k1)
            angle2_2 = -angle2_1
            k1 = L1 + L2 * np.cos(angle2_2)
            k2 = L2 * np.sin(angle2_2)
            angle1_2 = np.arctan2(y, x) - np.arctan2(k2, k1)

            return np.array([[angle1_1, angle2_1], [angle1_2, angle2_2]])
        for lines in self.lines:
            calculator.convert(lines)
            self.data.append(calculator.output)
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        return None
    def send(self):
        while self.iterator<len(self.send_data):
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
        text = "@".join(["+".join(["-".join([str(_z) for _z in _y]) for _y in _x]) for _x in self.data])
        f = open(f"{os.getcwd()}/output/{fle_name}.txt" , "w")
        f.write(text)
        f.close()
        return None
    def load_pulse(self , fle_name:str):
        f = open(f"{os.getcwd()}/output/{fle_name}.txt" , "r")
        text = f.read()
        f.close()
        
        self.data = [[[int(_z) for _z in _y.split("-") if _z != ""] for _y in _x.split("+")] for _x in text.split("@")]
        self.send_data = [sand_drawer.to_send(x,r) for x,r in self.data]
        return None

New_graph = False
if __name__ == "__main__":
    graph = sand_drawer("anon.jpg")
    if New_graph:
        graph.test()
        graph.get_line()
        graph.find_path()
        graph.preview()
        graph.convert()
        graph.save_pulse( "line" )
    else:
        graph.load_pulse( "line" )
    graph.send()





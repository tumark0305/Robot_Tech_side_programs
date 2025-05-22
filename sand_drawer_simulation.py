import os,cv2
import numpy as np
from math import cos, sin

class stepper:
    length = 250
    max_step = 2000
    def __init__(self , initial_pos:int):
        self.iterator = 0
        self.position = initial_pos
        self.data = None
        self.finish = False
        self.run_time = 0
        self.next_pulse_time = 0
    def move(self):
        if not self.finish:
            pulse = self.data[self.iterator]
            self.run_time += abs(pulse * 1e-6)
            if pulse<0:
                self.position += 1
            else:
                self.position -= 1
            self.iterator += 1
            if self.iterator  >= len(self.data):
                self.next_pulse_time = np.inf
                self.finish = True
            else:
                self.next_pulse_time = self.run_time + abs(self.data[self.iterator]) * 1e-6
        return None
    def feed_data(self,data):
        self.data = data
        self.iterator = 0
        self.finish = False
        self.next_pulse_time = abs(data[0]) * 1e-6
        self.run_time = 0
        return None
    def real_coordinate(self):
        theta = self.position * np.pi / (stepper.max_step//2)
        x = self.length * cos(theta)
        y = self.length * sin(theta)
        return np.array([x,y])

class simulation:
    arm_length = 250
    axis_length = 250
    image_size = 1024
    max_step = 2000
    def __init__(self,file_name:str):
        self.file_name = file_name
        self.input_path = f"{os.getcwd()}/output/{file_name}.txt"
        self.data = None
        self.load_pulse()
        self.image = simulation.image_init()
        self.frame = [self.image.copy()]
        self.frame_time = [0]

    @staticmethod
    def image_init():
        _image = np.zeros((simulation.image_size,simulation.image_size))
        center = (simulation.image_size // 2, simulation.image_size // 2)
        radius = simulation.axis_length + simulation.arm_length
        cv2.circle(_image, center, radius, 255, 1) 
        return _image
    def load_pulse(self):
        f = open(self.input_path , "r")
        text = f.read()
        f.close()
        self.data = [[[int(_z) for _z in _y.split("/") if _z != ""] for _y in _x.split("+")] for _x in text.split("@")]
        return None
    def read_framebyframe(self):
        index = 0
        total_frames = len(self.frame)

        while True:
            cv2.imshow("Frame Viewer", self.frame[index])
            key = cv2.waitKey(0) & 0xFF

            if key == 27:  # ESC
                break
            elif key == ord('d') or key == 83:  # ¡÷ Áä or 'd'
                index = min(index + 1, total_frames - 1)
            elif key == ord('a') or key == 81:  # ¡ö Áä or 'a'
                index = max(index - 1, 0)
        cv2.destroyAllWindows()
        return None
    def print_result(self):
        cv2.imshow("Frame Viewer", self.image)
        cv2.waitKey(0)
        return None
    def run(self):
        def convert(axis,arm):
            theta1 = axis * np.pi / int(simulation.max_step /2)
            theta2 = arm * np.pi / int(simulation.max_step /2) 
            x = simulation.axis_length * cos(theta1) + simulation.arm_length * cos(theta2) + int(simulation.image_size/2)
            y = simulation.axis_length * sin(theta1) + simulation.arm_length * sin(theta2) + int(simulation.image_size/2)
            return np.array([int(x),int(y)])
        axis = stepper(0)
        arm = stepper(simulation.max_step//2)
        for line in self.data:
            axis.feed_data(line[0][1:line[0][0]+1])
            arm.feed_data(line[1][1:line[1][0]+1])
            while not(axis.finish) or not(arm.finish):
                if axis.next_pulse_time < arm.next_pulse_time:
                    axis.move()
                elif axis.next_pulse_time > arm.next_pulse_time:
                    arm.move()
                elif axis.next_pulse_time == arm.next_pulse_time:
                    axis.move()
                    arm.move()
                #if 0 <= coordinate[0] < simulation.image_size and 0 <= coordinate[1] < simulation.image_size:
                coordinate = axis.real_coordinate() + arm.real_coordinate() + + int(simulation.image_size/2)
                self.image[int(coordinate[1]),int(coordinate[0])] = 255
                #self.frame.append(self.image.copy())
                self.frame_time.append(arm.run_time)
            self.frame.append(self.image.copy())

        return None
    def save(self):
        cv2.imwrite(f"{os.getcwd()}/output/{self.file_name}.jpg", self.image)
        return None

if __name__ == "__main__":
    graph = simulation("line")
    graph.run()
    graph.print_result()
    graph.save()
    graph.read_framebyframe()


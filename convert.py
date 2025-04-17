import cv2,os,ffmpeg,wave
from tqdm import tqdm
import numpy as np

process_file = "avem_test3"

input_video = f"{os.getcwd()}\\input\\{process_file}.mp4"
output_video = f"{os.getcwd()}\\output\\{process_file}_vid.txt"
output_audio = f"{os.getcwd()}\\output\\{process_file}_aud.txt"
video_data_view = []
class video:
    def __init__(self,process_file):
        return None
    def read_frame():
        print("reading frame ...")
        cap = cv2.VideoCapture(input_video)
        if not cap.isOpened():
            print("no reach file")
            exit()
        fps = cap.get(cv2.CAP_PROP_FPS)
        delay = int(1000 / fps) if fps > 0 else 30  
        _all = []
        _counter  = 0
        while True:
            _counter+= 1
            ret, frame = cap.read() 
            if not ret:
                print("read frame done~")
                break
            else:
                if _counter%2 ==0:
                    _all.append(frame.copy())
        cap.release()
        return _all
    def store_byte(image):
        output = []
        counter = 1
        buffer = ""
        for _y in range(64):
            for _x in range(128):
                if image[_y,_x] >= 128:
                    buffer += "1"
                else:
                    buffer += "0"
                if counter>=8:
                    num = int(buffer, 2) 
                    output.append(num)
                    buffer = ""
                    counter = 0
                counter += 1
        return output
    def write_file(_data):
        _f = open(output_video,'w')
        _f.write("+".join(["-".join([str(_y) for _y in _x]) for _x in _data ]))
        _f.close()
        return None
    def convert_method0(_all_frame):
        _output = []
        for frame in tqdm(_all_frame):
            gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)[:640,:]
            resized_frame = cv2.resize(gray_image, (128,64), interpolation=cv2.INTER_AREA)
            binary_image = cv2.adaptiveThreshold(resized_frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            normal_frame = video.store_byte(binary_image)
            _output.append(normal_frame)
            video_data_view.append(binary_image)
        return _output
    def convert_method1(_all_frame):
        _output = []
        for frame in tqdm(_all_frame):
            gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)[:640,:]
            gaussian_image = cv2.GaussianBlur(gray_image,(3,3),0)
            avg_blur = cv2.blur(gaussian_image,(3,3))
            binary_image = cv2.adaptiveThreshold(gaussian_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
            resized_frame = cv2.resize(binary_image, (128,64), interpolation=cv2.INTER_NEAREST)
            normal_frame = video.store_byte(resized_frame)
            _output.append(normal_frame)
            video_data_view.append(resized_frame)
        return _output
    def view():
        for _vid_frame in tqdm(video_data_view):
            resized_frame = cv2.resize(_vid_frame, (1280,640), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Video", resized_frame) 
            if cv2.waitKey(80) & 0xFF == ord('q'):
                break
        return None
class audio:
    package_length = 1764
    def __init__(self):
        return None
    def read_wav():
        _audio_buffer = f"{os.getcwd()}\\output\\{process_file}_aud.wav"
        ffmpeg.input(input_video).output(
            _audio_buffer,
            ar=22050,   # 22.05 kHz 取樣率
            ac=1,       # 單聲道
            sample_fmt="u8",  # 8-bit PCM
            format="wav",
            codec="pcm_u8" 
        ).run(overwrite_output=True)
        with wave.open(_audio_buffer, "rb") as wav_file:
            num_frames = wav_file.getnframes()  # 總取樣數
            audio_data = wav_file.readframes(num_frames)  # 讀取所有音訊數據
        _output = np.frombuffer(audio_data, dtype=np.uint8)
        print(f"Max:{max(_output)}  Min:{min(_output)}")
        return _output
    def cut_package(_audio):
        _data = []
        _counter = 0
        while True:
            _package = _audio[_counter * audio.package_length : (_counter+1) * audio.package_length]
            _data.append(_package)
            if (_counter+2) * audio.package_length > len(_audio):
                break
            else:
                _counter += 1
        return _data
    def write_file(_data):
        _f = open(output_audio,'w')
        _f.write("+".join(["-".join([str(_y) for _y in _x]) for _x in _data ]))
        _f.close()
        return None


if __name__ == "__main__":
    _audio_samples = audio.read_wav()
    _packaged_aud = audio.cut_package(_audio_samples)
    audio.write_file(_packaged_aud)
    _all_frame = video.read_frame()
    _pixel_frame = video.convert_method0(_all_frame)
    video.write_file(_pixel_frame)
    video.view()
    pass


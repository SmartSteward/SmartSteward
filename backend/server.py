import json
import time
from paho.mqtt import client as mqtt_client
import threading
from flask import Flask, render_template, Response
import email
from email.mime.text import MIMEText
import smtplib
import email.utils
import requests
from ultralytics import YOLO
import cv2
from collections import deque
import numpy as np

def deal_data(topic, data):
    if debug_mode:
        print(f"mqtt {topic}: {data}")
    global state_list
    if topic == "esp32/light_power":
        if data == "on":
            state_list["light"] = 1
        else:
            state_list["light"] = 0
        return
    if topic == "server/airConditioner":
        if data == "on":
            state_list["airConditioner"] = 1
        else:
            state_list["airConditioner"] = 0
        return
    if topic == "server/humidifier":
        if data == "on":
            state_list["humidifier"] = 1
        else:
            state_list["humidifier"] = 0
        return
    if topic == "esp32/smokeConcentration":
        if float(data) > 99999:
            return
        if float(data) < 5:
            return
        fd = float(data)
        if fd == None or fd == 0:
            return
        state_list["smokeConcentration"] = float(data)
    if topic == "wateradd":
        state_list["water"] += float(data)
        return
    if topic == "smokeAlarm":
        send_email("睿居管家服务提醒", "# 烟雾报警\n发生时间： " + time.strftime("%Y-%m-%d %H:%M:%S",
                                                                                time.localtime()) + "\n请及时查看实际情况！")
        print("触发烟雾报警：发送邮件提醒所有联系人")
        return
    if topic == "waterAlarm":
        send_email("睿居管家服务提醒", "# 无生活迹象报警\n发生时间： " + time.strftime("%Y-%m-%d %H:%M:%S",
                                                                                      time.localtime()) + "\n24小时内用水不超过0.5L！\n请及时查看实际情况！")
        print("触发无生活迹象温报警：发送邮件提醒所有联系人")
        return
    if topic == "temperatureAlarm":
        send_email("睿居管家服务提醒", "# 高温报警\n发生时间： " + time.strftime("%Y-%m-%d %H:%M:%S",
                                                                                time.localtime()) + "\n家里温度过高，当前温度 " +
                   str(state_list["temperature"]) + " 摄氏度！\n系统正在尝试自动打开空调...\n请及时查看实际情况！")
        print("触发高温报警：发送邮件提醒所有联系人，并尝试打开空调")
        try:
            publish(my_mqtt_client, "server/airConditioner", "on")
        except Exception as e:
            print(f"发送MQTT时出错: {e}")
        return
    if topic == "fallDown":
        send_email("睿居管家服务提醒", "# 摔倒报警\n发生时间： " + time.strftime("%Y-%m-%d %H:%M:%S",
                                                                                time.localtime()) + "\n请及时查看实际情况！")
        print("触发摔倒报警：发送邮件提醒所有联系人")
        return
    if topic == "xiaozhi_server/message":
        send_email("睿居管家服务提醒", "# 留言提醒\n发送时间： " + time.strftime("%Y-%m-%d %H:%M:%S",
                                                                                time.localtime()) + "\n消息内容：" + data)
        print("触发留言提醒：发送邮件提醒所有联系人，消息内容：", data)
        return
    try:
        if len(topic.split("/")) == 2:
            topic = topic.split("/")[1]
        state_list[topic] = float(data)
    except Exception as e:
        print(f"处理数据时出错: {e}")

def main_loop():
    global my_mqtt_client
    water_list = []
    t = 0
    while True:
        if t >= 360000:
            t = 0
        if t % 3600 == 0:
            if not no_get_API:
                get_weather()
                get_AQI()
            if len(water_list) < 24:
                water_list.append(state_list["water"])
            else:
                water_list = water_list[1:] + [state_list["water"]]
                if water_list[-1] - water_list[0] > 0.5:
                    try:
                        publish(my_mqtt_client, "waterAlarm", "!!!")
                    except Exception as e:
                        print(f"发送MQTT时出错: {e}")
        if  t % 60 == 0:
            get_time()
            if state_list["temperature"] >= 35.0:
                try:
                    publish(my_mqtt_client, "temperatureAlarm", "!!!")
                except Exception as e:
                    print(f"发送MQTT时出错: {e}")
        t += 1
        time.sleep(1)

def get_time():
    global my_mqtt_client
    try:
        publish(my_mqtt_client, "server/time", time.strftime("%H:%M", time.localtime()))
        publish(my_mqtt_client, "server/date", time.strftime("%Y %m %d", time.localtime()))
    except Exception as e:
        print(f"处理数据时出错: {e}")

def get_weather():
    global my_mqtt_client
    response = requests.get(f"https://restapi.amap.com/v3/weather/weatherInfo?city={city_number}&key={weather_key}")
    try:
        publish(my_mqtt_client, "server/weather", response.json()["lives"][0]["weather"])
        print("获取到天气：", response.json()["lives"][0]["weather"])
    except Exception as e:
        print(f"处理数据时出错: {e}")
        print("获取天气返回：", response.json())

def get_AQI():
    global my_mqtt_client
    response = requests.get('http://apis.juhe.cn/fapigw/air/live', params={'key': AQI_key,'cityId': city_id})
    try:
        publish(my_mqtt_client, "server/AQI", response.json()["result"]["AQI"] + " " + response.json()["result"]["Quality"])
        print("获取到空气质量：", response.json()["result"]["AQI"] + " " + response.json()["result"]["Quality"])
    except Exception as e:
        print(f"处理数据时出错: {e}")
        print("获取空气质量返回：", response.json())

def control_thread():
    """从控制台（命令行）获取命令输入到控制线程"""
    global no_get_API, state_list, debug_mode
    while True:
        x = input()
        if x == 'save':
            save_state()
        elif x == 'weather':
            get_weather()
        elif x == 'AQI':
            get_AQI()
        elif x == 'get_API':
            no_get_API = not no_get_API
            print("切换为 " + ("不" if no_get_API else "") + "获取天气和空气质量")
        elif x == 'debug':
            debug_mode = not debug_mode
            print("切换为 " + ("" if debug_mode else "不") + "调试模式")
        elif x == 'state':
            print(state_list)
        else:
            print("无效的命令")

def send_email(head_subject, main_data):
    global state_list
    message = MIMEText(main_data)
    message['To'] = email.utils.formataddr(('联系人', 'everyone'))
    message['From'] = email.utils.formataddr(('睿居管家-智能家居系统', from_email))
    message['Subject'] = head_subject
    server = smtplib.SMTP_SSL('smtp.qq.com', 465)
    server.login(from_email, email_authentication)
    server.set_debuglevel(True)
    try:
        server.sendmail(from_email, state_list["email_to"], msg=message.as_string())
    finally:
        server.quit()

def save_state():
    """保存状态到文件"""
    global state_list
    try:
        with open('state.json', 'w') as f:
            json.dump(state_list, f)
        print("状态已保存到 state.json")
    except Exception as e:
        print(f"保存状态时出错: {e}")

def load_state():
    """从文件中读取状态"""
    global state_list
    try:
        with open('state.json', 'r') as f:
            state_list.update(json.load(f))
        print("状态已从 state.json 加载")
    except FileNotFoundError:
        print("文件 state.json 未找到")
    except json.JSONDecodeError:
        print("文件 state.json 解析错误")
    except Exception as e:
        print(f"加载 state.json 时出错: {e}")

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("成功连接到MQTT服务器！")
        else:
            print(f"无法连接到MQTT服务器，错误代码： {rc}")
    client = mqtt_client.Client(client_id=client_id)
    client.username_pw_set(mqtt_username, mqtt_password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        # print(f"收到主题 `{msg.topic}` 的信息： `{msg.payload.decode()}`")
        deal_data(msg.topic, msg.payload.decode())
    for t in topic:
        client.subscribe(t)
    client.on_message = on_message

def publish(client, topic, msg):
    result = client.publish(topic, msg)
    return result[0]

def start_mqtt():
    global my_mqtt_client
    my_mqtt_client = connect_mqtt()
    subscribe(my_mqtt_client)

    loop_threading = threading.Thread(target=main_loop, daemon=True)
    loop_threading.start()

    my_mqtt_client.loop_forever()

class ModelInference:
    def __init__(self):
        # 加载摔倒检测模型
        self.model_ready = False
        self.weight_path = 'fall_detect.pt'
        self.model = YOLO(self.weight_path)
        self.objs_labels = self.model.names
        print("摔倒检测模型已加载，识别类别：" + str(self.objs_labels))
        # 初始化检测参数
        self.fall_count = 0
        self.start_time = time.time()
        self.last_alert_time = 0
        self.model_ready = True

    def predict(self, frame):
        global my_mqtt_client
        # 添加时间戳
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, timestamp, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 目标检测
        result = list(self.model(frame, conf=0.3, stream=True, verbose=False))[0]
        boxes = result.boxes.cpu().numpy()

        for box in boxes.data:
            # print(time.time())
            if box[5] == 0:  # 0是摔倒类别
                self.fall_count += 1

        # 报警逻辑
        current_time = time.time()
        if self.fall_count >= 10:
            if (current_time - self.start_time <= 3) and (current_time - self.last_alert_time > 60):
                print("警报：3秒内检测到10次摔倒")
                publish(my_mqtt_client, 'fallDown', 'fallDown')
                # 重置参数
                self.fall_count = 0
                self.start_time = current_time
                self.last_alert_time = current_time
            else:
                self.fall_count = 1
                self.start_time = current_time
        elif current_time - self.start_time > 3:
            self.fall_count = 0
            self.start_time = time.time()

        return frame

class CameraStream:
    def __init__(self, buffer_size=3000, cap_addr=0):
        self.cap_addr = cap_addr
        self.camera = cv2.VideoCapture(self.cap_addr)
        self.frame_buffer = deque(maxlen=buffer_size)
        self.buffer_lock = threading.Lock()
        self.is_running = True
        # 记录所有活跃的消费者
        self.consumers = set()
        self.consumers_lock = threading.Lock()

        # 初始化推理模型
        self.model = ModelInference()

        # 添加性能统计
        self.fps = 0
        self.inference_time = 0
        self.last_frame_time = time.time()

        # 启动生产者线程
        self.producer_thread = threading.Thread(
            target=self._producer_task,
            daemon=True
        )
        self.producer_thread.start()
        
        # 启动显示窗口线程
        self.display_thread = threading.Thread(
            target=self._display_window,
            daemon=True
        )
        self.display_thread.start()

    def _producer_task(self):
        """生产者任务：读取和处理帧"""
        frame_count = 0
        fps_update_interval = 30  # 每30帧更新一次FPS
        
        print(f"摄像头线程启动，尝试连接: {self.cap_addr}")

        while self.is_running:
            ret, frame = self.camera.read()
            if ret:
                # 记录推理开始时间
                inference_start = time.time()

                # 进行模型推理
                processed_frame = self._process_frame(frame)

                # 计算推理时间
                self.inference_time = time.time() - inference_start

                # 转换为JPEG格式
                ret, buffer = cv2.imencode('.jpg', processed_frame)
                if ret:
                    jpeg_frame = buffer.tobytes()
                    timestamp = time.time()

                    with self.buffer_lock:
                        self.frame_buffer.append({
                            'frame': jpeg_frame,
                            'timestamp': timestamp
                        })

                    # 更新FPS计算
                    frame_count += 1
                    if frame_count % fps_update_interval == 0:
                        current_time = time.time()
                        self.fps = fps_update_interval / (current_time - self.last_frame_time)
                        self.last_frame_time = current_time
                        print(f"视频流正常，FPS: {self.fps:.1f}, 缓冲区大小: {len(self.frame_buffer)}")
            else:
                print("无法读取摄像头帧，尝试重新连接...")
                self.camera = cv2.VideoCapture(self.cap_addr)
                time.sleep(1)  # 等待1秒后重试

            time.sleep(0.001)  # 小的延迟以防止CPU过载

    def _process_frame(self, frame):
        """使用模型处理帧"""
        if self.model.model_ready:
            return self.model.predict(frame)
        else:
            # 如果模型未就绪，显示等待信息
            cv2.putText(frame, "Model Loading...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return frame


    def _display_window(self):
        """显示摄像头画面窗口"""
        while self.is_running:
            with self.buffer_lock:
                if self.frame_buffer:
                    newest_frame = self.frame_buffer[-1]
                    # 解码JPEG数据
                    frame_data = np.frombuffer(newest_frame['frame'], np.uint8)
                    frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
                    if frame is not None:
                        # 添加时间水印（已经在model.predict中添加）
                        cv2.imshow('Camera Stream', frame)
            # 按ESC键退出窗口
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

    def register_consumer(self):
        """注册新的消费者"""
        consumer_id = id(threading.current_thread())
        with self.consumers_lock:
            self.consumers.add(consumer_id)
        return consumer_id

    def unregister_consumer(self, consumer_id):
        """注销消费者"""
        with self.consumers_lock:
            self.consumers.discard(consumer_id)

    def get_frame_generator(self):
        """为每个消费者创建独立的帧生成器"""
        consumer_id = self.register_consumer()
        last_frame_time = 0

        try:
            while self.is_running:
                frame_data = None
                with self.buffer_lock:
                    if self.frame_buffer:
                        newest_frame = self.frame_buffer[-1]
                        # 只有当有新帧时才发送
                        if newest_frame['timestamp'] > last_frame_time:
                            frame_data = newest_frame
                            last_frame_time = newest_frame['timestamp']

                if frame_data:
                    # 直接使用已经编码好的JPEG数据
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           frame_data['frame'] +
                           b'\r\n')

                time.sleep(1 / 30)  # 控制消费帧率
        finally:
            self.unregister_consumer(consumer_id)

    def get_stats(self):
        """获取当前状态"""
        with self.consumers_lock:
            consumer_count = len(self.consumers)
        with self.buffer_lock:
            buffer_size = len(self.frame_buffer)
        return {
            'active_consumers': consumer_count,
            'buffer_size': buffer_size,
            'fps': round(self.fps, 1),
            'inference_time': round(self.inference_time * 1000, 1),  # 转换为毫秒
            'model_ready': self.model.model_ready
        }

    def cleanup(self):
        """清理资源"""
        self.is_running = False
        self.producer_thread.join()
        self.camera.release()

def camera_init():
    global camera_stream, cap_addr
    camera_stream = CameraStream(cap_addr=cap_addr)



no_get_API = False
debug_mode = False
city_number = '440100'  # 获取天气的城市的编码
weather_key = "9d2b57ce7004e6ce4993ff79495bf675"  # 高德地图获取天气的API的key
city_id = "189"  # 获取空气质量的城市的ID
AQI_key = "059e310bcabb9e6da8ceca45673ba9d1"  # 聚合数据获取空气质量的API的key
from_email = "TDR_Group@foxmail.com"  # 发邮件用的邮箱
email_authentication = 'ghurjkukgtzebiia'  # 发邮件授权码
broker = 'znjj.piedaochuan.top'  # mqtt服务器地址
port = 1883  # mqtt服务器端口
client_id = 'main-server'  # mqtt客户端id
mqtt_username = "xiaozhi_anyizhijia"  # MQTT服务器用户名
mqtt_password = "123456"  #MQTT服务器密码
http_server_debug = False  # http服务器flask调试模式
http_server_host = '0.0.0.0'  # http服务器监听地址
http_server_port = 41235  # http服务器监听端口
state_list = {}  # 存放各种状态数据
my_mqtt_client = None
cap_addr = "http://192.168.3.159:8080/video"  # 摄像头地址（填0可以调用本地摄像头，或者填写网络摄像头地址如 "http://192.168.3.159:8080"）
camera_stream = None
topic = [  # mqtt订阅的主题topic
    "esp32/temperature",  # 温度
    "esp32/humidity",  # 湿度
    "water",  # 用水量
    "wateradd",  # 用水增量
    "esp32/smokeConcentration",  # 烟雾浓度
    "heartRate",  # 心率
    "oxygenLevel",  # 血氧
    "server/light_power",  # 灯开关
    "server/airConditioner",  # 空调开关
    "server/humidifier",  # 加湿器开关
    "fallDown",  # 摔倒警报
    "smokeAlarm",  # 烟雾警报
    "xiaozhi_server/message",  # 邮件发送消息
    "waterAlarm",  # 无生活迹象警报
    "temperatureAlarm",  # 高温警报
]


if __name__ == '__main__':
    load_state()

    camera_init_threading = threading.Thread(target=camera_init, daemon=True)
    camera_init_threading.start()

    main_threading = threading.Thread(target=start_mqtt, daemon=True)
    main_threading.start()

    control_threading = threading.Thread(target=control_thread, daemon=True)
    control_threading.start()

    app = Flask(__name__)
    @app.route("/")
    def index():
        return render_template('index.html')

    @app.route("/api/<string:r>", )
    def index_args(r):
        global state_list
        if r == "state":
            return state_list
        else:
            return "404"

    @app.route("/api/<string:r>/<string:r2>", )
    def index_argss(r, r2):
        publish(my_mqtt_client, r, r2)
        return f'已发送topic为 {r} , 信息为 {r2} 的消息'

    @app.route("/api/<string:r>/<string:r2>/<string:r3>", )
    def index_argsss(r, r2, r3):
        publish(my_mqtt_client, r+"/"+r2, r3)
        if r == "esp32s3":
            publish(my_mqtt_client, "esp32/" + r2, r3)
        return f'已发送topic为 {r+"/"+r2} , 信息为 {r3} 的消息'

    @app.route('/video_feed')
    def video_feed():
        if camera_stream is None:
            print("错误：camera_stream 未初始化")
            return Response("摄像头未初始化", mimetype='text/plain')
        print("开始提供视频流...")
        return Response(
            camera_stream.get_frame_generator(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    app.run(debug=http_server_debug, host=http_server_host, port=http_server_port)

    print("http_server/主线程退出")
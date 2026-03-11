# Src 模块架构文档

## 系统架构总览

```mermaid
graph TB
    subgraph Application Layer
        MAIN[main.py<br/>Entry Point]
        KP[KeyboardPressor<br/>Orchestrator]
        TCP[TCPServer<br/>LabVIEW Integration]
    end

    subgraph Vision Layer
        RS[RealSense435i<br/>Camera Capture]
        YOLO[Yolov8Engine2<br/>Object Detection]
    end

    subgraph Robot Layer
        DOBOT[DobotMG400<br/>Robot Controller]
    end

    subgraph Configuration
        CFG[Config<br/>cali.json Loader]
    end

    subgraph External Dependencies
        DASHBOARD[DobotApiDashboard]
        MOVE[DobotApiMove]
        FEED[DobotApi]
        RS_SDK[pyrealsense2]
        ONNX[onnxruntime]
    end

    MAIN --> KP
    MAIN --> TCP
    KP --> RS
    KP --> YOLO
    KP --> DOBOT
    KP --> CFG
    DOBOT --> DASHBOARD
    DOBOT --> MOVE
    DOBOT --> FEED
    RS --> RS_SDK
    YOLO --> ONNX
    TCP -.->|TCP:45678| EXTERNAL[LabVIEW Client]
```

---

## 模块依赖关系

```mermaid
graph LR
    subgraph src
        MAIN[main.py]
        KP[keyboard_pressor.py]
        TCP[tcp_server.py]
        CFG[config.py]
        RS[realsense_435i.py]
        DOBOT[dobot_mg400.py]
        YOLO[yolov8_onnx.py]
    end

    subgraph external
        API[dobot_api.py]
        RS2[pyrealsense2]
        CV2[opencv-python]
        ONNX[onnxruntime]
        NP[numpy]
        JSON[cali.json]
    end

    KP --> CFG
    KP --> RS
    KP --> DOBOT
    KP --> YOLO

    DOBOT --> API

    RS --> RS2
    RS --> CV2

    YOLO --> ONNX
    YOLO --> CV2
    YOLO --> NP

    CFG --> JSON
    TCP --> JSON
```

---

## 类架构图

```mermaid
classDiagram
    class KeyboardPressor {
        -click_point: tuple
        -config: Config
        -robot: DobotMG400
        -cap: RealSense435i
        -engine: Yolov8Engine2
        -robot_pix_x: float
        -robot_pix_y: float
        -scale: float
        -press_deep: float
        +__init__(model_path: str)
        +to_init_pose()
        +pix2pose(point: tuple) tuple
        +press_pix_point(point: tuple)
        +press_num(num: Keyboard)
        +mouse_callback(event, x, y, flags, param)
        -_inference_loop()
    }

    class Keyboard {
        <<enumeration>>
        NUM_ZERO = 0
        NUM_ONE = 1
        NUM_TWO = 2
    }

    class Config {
        -left_top: tuple
        -right_bottom: tuple
        -pix_left_top: tuple
        -pix_right_bottom: tuple
        -press_deep: float
        -max_deep: float
        -init_pix: tuple
        -init_pos: tuple
        -cap_to_robot_end: float
        +__init__()
        +scale: float
        +scale_x: float
        +scale_y: float
    }

    class RealSense435i {
        -lock: Lock
        -latest_color: ndarray
        -latest_depth: ndarray
        -depth_frame: Frame
        -pipeline: Pipeline
        -align_to_color: Align
        -colorizer: Colorizer
        -init_point: tuple
        -tar_x: float
        -tar_y: float
        -has_frame: bool
        +__init__(init_point: tuple)
        +init_tar()
        +mouse_callback(event, x, y, flags, param)
        -_capture_loop()
        +get_latest() tuple
        +get_point_depth(point: tuple) float
        +get_frames() generator
        +display()
        +cali()
    }

    class DobotMG400 {
        -ip: str
        -dashboard_port: int
        -mover_port: int
        -feed_port: int
        -init_pose: tuple
        -max_deep: float
        -x: float
        -y: float
        -z: float
        +__init__(ip, ports, init_pose, max_deep)
        +to_init_pose()
        +to_delta_pose(dx, dy, dz)
        -connect_robot()
        +run_point(point_list: list)
        -get_feed(feed: DobotApi)
        -wait_arrive(point_list: list)
        -clear_robot_error(dashboard: DobotApiDashboard)
    }

    class Yolov8Engine2 {
        -input_name: str
        -input_size: tuple
        -model_path: str
        -conf: float
        -iou: float
        -session: InferenceSession
        -orig_img_size: tuple
        -cur_detect_res: dict
        -_lock: Lock
        +__init__(model_path, conf, iou)
        +load()
        +inference(color_frame: ndarray)
        +latest_res: dict
        -preprocess(color_frame: ndarray)
        -postprocess(outputs)
    }

    class TCPServer {
        -host: str
        -port: int
        -socket: socket
        -msg_queue: Queue
        -_lock: Lock
        -cur_socket: socket
        +__init__(host, port)
        +send_msg(msg: str)
        +msg: dict
        -handle_client(client_socket, address)
        +start()
    }

    KeyboardPressor --> Keyboard
    KeyboardPressor --> Config
    KeyboardPressor --> RealSense435i
    KeyboardPressor --> DobotMG400
    KeyboardPressor --> Yolov8Engine2
    KeyboardPressor --> TCPServer
```

---

## 时序图

### 1. 系统初始化时序图

```mermaid
sequenceDiagram
    participant Main as main.py
    participant KP as KeyboardPressor
    participant CFG as Config
    participant RS as RealSense435i
    participant DOBOT as DobotMG400
    participant YOLO as Yolov8Engine2
    participant TCP as TCPServer

    Main->>TCP: TCPServer(host, port)
    activate TCP
    TCP-->>TCP: socket.bind()
    TCP-->>TCP: socket.listen()
    TCP-->>Main: 服务器启动
    deactivate TCP

    Main->>TCP: start() [Thread]
    activate TCP
    TCP->>TCP: socket.accept()
    TCP->>TCP: handle_client() [Thread]

    Main->>KP: KeyboardPressor(model_path)
    activate KP

    KP->>CFG: Config()
    activate CFG
    CFG->>CFG: open("cali.json")
    CFG->>CFG: parse config
    CFG-->>KP: config loaded
    deactivate CFG

    KP->>DOBOT: DobotMG400(init_pose, max_deep)
    activate DOBOT
    DOBOT->>DOBOT: connect_robot()
    DOBOT->>DOBOT: DobotApiDashboard(port 29999)
    DOBOT->>DOBOT: DobotApiMove(port 30003)
    DOBOT->>DOBOT: DobotApi(port 30004)
    DOBOT->>DOBOT: EnableRobot()
    DOBOT->>DOBOT: get_feed() [Thread]
    DOBOT->>DOBOT: clear_robot_error() [Thread]
    DOBOT-->>KP: robot connected
    deactivate DOBOT

    KP->>RS: RealSense435i(init_point)
    activate RS
    RS->>RS: rs.pipeline()
    RS->>RS: config.enable_stream()
    RS->>RS: pipeline.start()
    RS->>RS: align(rs.stream.color)
    RS->>RS: _capture_loop() [Thread]
    RS-->>KP: camera ready
    deactivate RS

    KP->>YOLO: Yolov8Engine2(model_path)
    activate YOLO
    YOLO->>YOLO: load()
    YOLO->>YOLO: ort.InferenceSession()
    YOLO->>YOLO: get_inputs()[0].shape
    YOLO-->>KP: model loaded
    deactivate YOLO

    KP->>KP: to_init_pose()
    KP->>RS: init_tar()
    KP->>DOBOT: to_init_pose()
    KP->>KP: _inference_loop() [Thread]

    KP-->>Main: initialization complete
    deactivate KP
```

---

### 2. 视觉推理时序图

```mermaid
sequenceDiagram
    participant INF as _inference_loop
    participant RS as RealSense435i
    participant YOLO as Yolov8Engine2
    participant CV2 as cv2
    participant UI as OpenCV Window

    loop Every Frame (~30fps)
        INF->>RS: get_latest()
        activate RS
        RS-->>RS: wait for _capture_loop
        RS-->>INF: (color_frame, depth_frame, depth)
        deactivate RS

        INF->>YOLO: inference(color_frame)
        activate YOLO
        YOLO->>YOLO: preprocess()
        YOLO->>YOLO: session.run()
        YOLO->>YOLO: postprocess()
        YOLO->>YOLO: update cur_detect_res
        deactivate YOLO

        INF->>INF: cv2.cvtColor(RGB→BGR)

        loop For each detection
            YOLO-->>INF: latest_res
            INF->>CV2: cv2.circle()
            INF->>CV2: cv2.line()
        end

        INF->>UI: cv2.imshow("frame", img)
        INF->>UI: cv2.setMouseCallback()
        INF->>UI: cv2.waitKey(1)
    end
```

---

### 3. 相机捕获时序图

```mermaid
sequenceDiagram
    participant RS as RealSense435i
    participant PIPELINE as rs.pipeline
    participant ALIGN as rs.align
    participant COLORIZER as rs.colorizer
    participant LOCK as threading.Lock

    Note over RS: __init__
    RS->>PIPELINE: rs.pipeline()
    RS->>PIPELINE: config.enable_stream(depth)
    RS->>PIPELINE: config.enable_stream(color)
    RS->>PIPELINE: pipeline.start(config)
    RS->>ALIGN: rs.align(rs.stream.color)
    RS->>COLORIZER: rs.colorizer()
    RS->>RS: _capture_loop() [Thread]

    loop Continuous Capture
        RS->>PIPELINE: wait_for_frames()
        PIPELINE-->>RS: frames

        RS->>ALIGN: align.process(frames)
        ALIGN-->>RS: aligned_frames

        RS->>RS: aligned.get_depth_frame()
        RS->>RS: aligned.get_color_frame()

        RS->>COLORIZER: colorize(depth_frame)
        COLORIZER-->>RS: depth_colormap_frame

        RS->>LOCK: acquire()
        RS->>RS: latest_color = color_img
        RS->>RS: latest_depth = depth_color
        RS->>RS: depth_frame = depth
        RS->>RS: has_frame = True
        RS->>LOCK: release()
    end

    Note over RS: External call: get_latest()
    RS->>RS: wait has_frame
    RS->>LOCK: acquire()
    RS->>RS: return (latest_color, latest_depth, depth_frame)
    RS->>LOCK: release()
```

---

### 4. YOLO 推理时序图

```mermaid
sequenceDiagram
    participant Caller as External
    participant YOLO as Yolov8Engine2
    participant ORT as onnxruntime.Session
    participant CV2 as cv2
    participant NP as numpy

    Note over YOLO: __init__
    Caller->>YOLO: Yolov8Engine2(model_path, conf, iou)
    YOLO->>YOLO: load()
    YOLO->>ORT: get_available_providers()
    YOLO->>ORT: SessionOptions()
    YOLO->>ORT: InferenceSession(model, providers)
    YOLO->>ORT: get_inputs()[0].shape[2:]
    YOLO-->>YOLO: input_size, input_name

    Note over YOLO: inference(color_frame)
    Caller->>YOLO: inference(color_frame)

    YOLO->>YOLO: preprocess(color_frame)
    CV2->>CV2: cvtColor(if needed)
    CV2->>CV2: resize(input_size, INTER_LINEAR)
    NP->>NP: astype(float32) / 255.0
    NP->>NP: transpose(2,0,1)
    NP->>NP: expand_dims(axis=0)
    YOLO-->>YOLO: input_image, resized_image

    YOLO->>ORT: session.run(None, {input_name: input_image})
    ORT-->>YOLO: outputs[0][0]

    YOLO->>YOLO: postprocess(outputs)
    NP->>NP: predictions[4:].max(axis=0)
    NP->>NP: scores > conf (mask)
    NP->>NP: filtered_predictions[:, mask]
    NP->>NP: argmax(axis=0) for class_ids
    CV2->>CV2: dnn.NMSBoxes(boxes, conf, conf, iou)
    YOLO-->>YOLO: final_boxes, confidences, class_ids

    loop For each detection
        YOLO->>YOLO: calc normalized xc, yc, w, h
        YOLO->>YOLO: temp_res[class_id] = {xc, yc, w, h, conf}
    end

    YOLO->>YOLO: _lock.acquire()
    YOLO->>YOLO: cur_detect_res = temp_res
    YOLO->>YOLO: _lock.release()

    Note over YOLO: External call: latest_res
    Caller->>YOLO: latest_res (property)
    YOLO->>YOLO: _lock.acquire()
    YOLO->>YOLO: return cur_detect_res.copy()
    YOLO->>YOLO: _lock.release()
```

---

### 5. 按键按压时序图 (press_num)

```mermaid
sequenceDiagram
    participant TCP as TCPServer
    participant KP as KeyboardPressor
    participant YOLO as Yolov8Engine2
    participant RS as RealSense435i
    participant DOBOT as DobotMG400
    participant MOVE as DobotApiMove

    TCP->>TCP: msg_queue.get()
    TCP-->>KP: {"cmd": 1}

    KP->>KP: press_num(Keyboard.NUM_ONE)
    KP->>YOLO: latest_res
    activate YOLO
    YOLO-->>KP: {1: {xc, yc, w, h, conf}}
    deactivate YOLO

    KP->>KP: calc pix_x, pix_y
    KP->>RS: get_point_depth((pix_x, pix_y))
    activate RS
    RS-->>RS: wait for depth_frame
    RS-->>KP: dist (meters)
    deactivate RS

    KP->>KP: delta_z = -(depth*1000 - cap_to_robot_end) - press_deep
    KP->>KP: pix2pose((pix_x, pix_y))
    KP->>KP: delta_pose_x = delta_pix_y / scale_x
    KP->>KP: delta_pose_y = delta_pix_x / scale_y
    KP-->>KP: (delta_x, delta_y)

    KP->>DOBOT: to_delta_pose(delta_x, delta_y, delta_z)
    activate DOBOT
    DOBOT->>DOBOT: x += delta_x, y += delta_y, z += delta_z
    DOBOT->>MOVE: MovL(x, y, init_pose[2], 0)
    activate MOVE
    MOVE-->>MOVE: send "MovL(...)" to port 30003
    DOBOT->>DOBOT: wait_arrive()
    loop Wait for arrival
        DOBOT->>DOBOT: check current_actual vs target
    end
    deactivate MOVE

    DOBOT->>MOVE: MovL(x, y, z, 0)
    activate MOVE
    MOVE-->>MOVE: send "MovL(...)" to port 30003
    DOBOT->>DOBOT: wait_arrive()
    deactivate MOVE
    deactivate DOBOT

    DOBOT-->>KP: motion complete
    KP->>KP: time.sleep(1)

    KP->>KP: to_init_pose()
    KP->>RS: init_tar()
    KP->>DOBOT: to_init_pose()
    activate DOBOT
    DOBOT->>MOVE: MovL to init z
    DOBOT->>MOVE: MovL to init x,y,z
    deactivate DOBOT

    KP-->>TCP: send_msg("1")
```

---

### 6. 像素点按压时序图 (press_pix_point)

```mermaid
sequenceDiagram
    participant UI as OpenCV Window
    participant KP as KeyboardPressor
    participant RS as RealSense435i
    participant DOBOT as DobotMG400
    participant MOVE as DobotApiMove

    UI->>KP: mouse_callback(LBUTTONDOWN, x, y)
    KP->>KP: click_point = (x, y)

    Note over KP: External command triggers press_pix_point
    KP->>KP: press_pix_point((pix_x, pix_y))

    KP->>RS: get_point_depth((pix_x, pix_y))
    activate RS
    RS-->>RS: wait depth_frame
    RS-->>KP: dist
    deactivate RS

    KP->>KP: delta_z = -(depth*1000 - cap_to_robot_end) - press_deep
    KP->>KP: pix2pose((pix_x, pix_y))
    KP-->>KP: (delta_x, delta_y)

    KP->>DOBOT: to_delta_pose(delta_x, delta_y, delta_z)
    activate DOBOT
    DOBOT->>DOBOT: update x, y, z
    DOBOT->>MOVE: MovL to approach
    DOBOT->>MOVE: MovL to press
    DOBOT-->>KP: complete
    deactivate DOBOT

    KP->>KP: time.sleep(1)
    KP->>KP: to_init_pose()
```

---

### 7. TCP 服务器运行时序图

```mermaid
sequenceDiagram
    participant Main as main.py
    participant TCP as TCPServer
    participant CLIENT as LabVIEW
    participant QUEUE as msg_queue
    participant KP as KeyboardPressor

    Main->>TCP: TCPServer('localhost', 45678)
    TCP->>TCP: socket.bind()
    TCP->>TCP: socket.listen(5)
    TCP-->>Main: server started

    Main->>TCP: start() [Daemon Thread]

    loop Server Loop
        TCP->>TCP: socket.accept()
        TCP-->>CLIENT: connection established
        TCP->>TCP: handle_client() [Thread]

        loop Client Connected
            CLIENT->>TCP: send(json_message)
            TCP->>TCP: recv(1024)
            TCP->>TCP: json.loads()
            TCP->>QUEUE: msg_queue.put(data)

            alt Response
                TCP->>CLIENT: send_msg("1")
            end
        end
    end

    Note over KP: Main loop
    KP->>QUEUE: msg_queue.get() [blocking]
    QUEUE-->>KP: {"cmd": 1} or {"points": [[x,y]]}

    KP->>KP: process command
    KP->>TCP: send_msg("1")
    TCP->>TCP: cur_socket.send()
    TCP-->>CLIENT: response
```

---

### 8. 机器人反馈循环时序图

```mermaid
sequenceDiagram
    participant DOBOT as DobotMG400
    participant FEED as DobotApi (port 30004)
    participant ROBOT as Physical Robot
    participant LOCK as globalLockValue

    Note over DOBOT: __init__
    DOBOT->>FEED: DobotApi(ip, feed_port)
    DOBOT->>DOBOT: get_feed() [Daemon Thread]

    loop Continuous Feedback (1ms interval)
        DOBOT->>FEED: socket_dobot.recv(1440)
        FEED-->>FEED: recv until 1440 bytes

        DOBOT->>DOBOT: np.frombuffer(data, dtype=MyType)
        DOBOT->>DOBOT: check test_value == 0x123456789abcdef

        DOBOT->>LOCK: acquire()
        DOBOT->>DOBOT: current_actual = tool_vector_actual[0]
        DOBOT->>DOBOT: algorithm_queue = isRunQueuedCmd[0]
        DOBOT->>DOBOT: enableStatus_robot = EnableStatus[0]
        DOBOT->>DOBOT: robotErrorState = ErrorStatus[0]
        DOBOT->>LOCK: release()

        DOBOT->>DOBOT: sleep(0.001)
    end

    Note over DOBOT: External: run_point()
    DOBOT->>DOBOT: wait_arrive(point_list)
    loop Wait for arrival
        DOBOT->>LOCK: acquire()
        DOBOT->>DOBOT: check |current_actual - target| < 1
        DOBOT->>LOCK: release()
        DOBOT->>DOBOT: sleep(0.001)
    end
    DOBOT-->>DOBOT: arrived
```

---

### 9. 机器人错误处理时序图

```mermaid
sequenceDiagram
    participant DOBOT as DobotMG400
    participant DASHBOARD as DobotApiDashboard
    participant ALARM as alarmAlarmJsonFile
    participant USER as User Input

    Note over DOBOT: clear_robot_error() [Daemon Thread]
    DOBOT->>ALARM: load alarm_controller.json
    DOBOT->>ALARM: load alarm_servo.json
    ALARM-->>DOBOT: dataController, dataServo

    loop Every 5 seconds
        DOBOT->>DOBOT: check robotErrorState

        alt Error State True
            DOBOT->>DASHBOARD: GetErrorID()
            DASHBOARD-->>DOBOT: error codes string

            DOBOT->>DOBOT: re.findall(error_codes)

            loop For each error code
                alt i == -2
                    DOBOT->>DOBOT: print("机器碰撞")
                else i in dataController
                    DOBOT->>DOBOT: lookup description (zh_CN)
                    DOBOT->>DOBOT: print alarm
                else i in dataServo
                    DOBOT->>DOBOT: lookup description (zh_CN)
                    DOBOT->>DOBOT: print alarm
                end
            end

            DOBOT->>USER: "输入 1, 将清除错误..."
            USER-->>DOBOT: choose

            alt choose == 1
                DOBOT->>DASHBOARD: ClearError()
                DOBOT->>DASHBOARD: sleep(0.01)
                DOBOT->>DASHBOARD: Continue()
            end
        else Normal State
            DOBOT->>DOBOT: check enableStatus && !algorithm_queue
            DOBOT->>DASHBOARD: Continue()
        end
    end
```

---

## 线程模型总览

```mermaid
graph TB
    subgraph Main_Process
        MAIN[Main Thread<br/>main.py]
    end

    subgraph TCPServer_Threads
        TCP_MAIN[TCPServer Thread<br/>socket.listen]
        TCP_CLIENT[Client Handler Thread<br/>handle_client]
    end

    subgraph KeyboardPressor_Threads
        KP_INF[Inference Thread<br/>_inference_loop]
    end

    subgraph DobotMG400_Threads
        FB_THREAD[Feedback Thread<br/>get_feed]
        ERR_THREAD[Error Handler Thread<br/>clear_robot_error]
    end

    subgraph RealSense435i_Threads
        CAP_THREAD[Capture Thread<br/>_capture_loop]
    end

    MAIN --> TCP_MAIN
    MAIN --> KP_INF
    MAIN --> FB_THREAD
    MAIN --> ERR_THREAD
    MAIN --> CAP_THREAD

    TCP_MAIN --> TCP_CLIENT
```

---

## 共享状态与锁

```mermaid
graph TB
    subgraph Global_Variables
        CURRENT[current_actual<br/>tool_vector_actual]
        QUEUE[algorithm_queue<br/>isRunQueuedCmd]
        ENABLE[enableStatus_robot<br/>EnableStatus]
        ERROR[robotErrorState<br/>ErrorStatus]
    end

    subgraph Locks
        GLOBAL_LOCK[globalLockValue<br/>threading.Lock]
        RS_LOCK[RealSense435i._lock]
        YOLO_LOCK[Yolov8Engine2._lock]
        TCP_LOCK[TCPServer._lock]
    end

    subgraph Thread_Safe_Data
        RS_DATA[latest_color, latest_depth<br/>depth_frame]
        YOLO_DATA[cur_detect_res]
        TCP_DATA[msg_queue]
    end

    GLOBAL_LOCK --> CURRENT
    GLOBAL_LOCK --> QUEUE
    GLOBAL_LOCK --> ENABLE
    GLOBAL_LOCK --> ERROR

    RS_LOCK --> RS_DATA
    YOLO_LOCK --> YOLO_DATA
    TCP_LOCK --> TCP_DATA
```

---

## 数据流图

```mermaid
graph LR
    subgraph Input
        CMD[TCP Command<br/>from LabVIEW]
        CLICK[Mouse Click<br/>OpenCV Window]
    end

    subgraph Vision
        CAM[RealSense Camera]
        DET[YOLO Detection]
    end

    subgraph Processing
        KP[KeyboardPressor]
        CFG[Config/cali.json]
    end

    subgraph Output
        ROBOT[Dobot MG400]
        RES[TCP Response<br/>to LabVIEW]
    end

    CMD --> KP
    CLICK --> KP
    CAM --> KP
    DET --> KP
    KP --> CFG
    CFG --> KP
    KP --> ROBOT
    KP --> RES
```

---

## 配置参数说明 (cali.json)

| 参数 | 类型 | 说明 |
|------|------|------|
| `left_top` | {x, y} | 相机视野左上角对应的机械臂坐标 |
| `right_bottom` | {x, y} | 相机视野右下角对应的机械臂坐标 |
| `pix_left_top` | {x, y} | 相机画面左上角像素坐标 |
| `pix_right_bottom` | {x, y} | 相机画面右下角像素坐标 |
| `press_deep` | float | 按键下压深度 (mm) |
| `cap_to_keyborad` | float | 相机到键盘表面距离 (mm) |
| `robot_on_keyboard` | float | 机械臂 Z 轴在键盘表面的坐标值 |
| `max_deep` | float | 最大允许下压深度 Z 坐标 |
| `init_pos` | {pix_x, pix_y, pos_x, pos_y, pos_z} | 初始位置配置 |

### Config 类计算属性

| 属性 | 计算公式 | 说明 |
|------|----------|------|
| `scale` | (y_scale + x_scale) / 2 | 平均缩放比例 |
| `scale_x` | (pix_right_bottom[1] - pix_left_top[1]) / (right_bottom[0] - left_top[0]) | 机械臂 X 轴缩放比例 |
| `scale_y` | (pix_right_bottom[0] - pix_left_top[0]) / (right_bottom[1] - left_top[1]) | 机械臂 Y 轴缩放比例 |
| `cap_to_robot_end` | cap_to_keyborad - robot_to_keyboard | 相机到机械臂末端的距离 |

---

## 关键方法说明

### KeyboardPressor

| 方法 | 作用 | 关键参数 |
|------|------|----------|
| `__init__` | 初始化所有子模块并启动推理线程 | model_path: ONNX 模型路径 |
| `to_init_pose` | 复位到初始位置和状态 | - |
| `pix2pose` | 像素坐标转机械臂增量坐标 | point: (pix_x, pix_y) |
| `press_pix_point` | 按压指定像素位置的点 | point: (pix_x, pix_y) |
| `press_num` | 按压检测到的指定按键 | num: Keyboard Enum |
| `_inference_loop` | 持续捕获帧并运行推理 | - |

### RealSense435i

| 方法 | 作用 | 关键参数 |
|------|------|----------|
| `__init__` | 初始化相机流水线并启动捕获线程 | init_point: 初始目标点 |
| `_capture_loop` | 持续捕获对齐的彩色和深度帧 | - |
| `get_latest` | 获取最新的彩色/深度帧 | - |
| `get_point_depth` | 获取指定像素点的深度值 | point: (x, y) |
| `cali` | 标定模式显示 | - |

### DobotMG400

| 方法 | 作用 | 关键参数 |
|------|------|----------|
| `__init__` | 连接机器人并使能 | ip, ports, init_pose, max_deep |
| `to_init_pose` | 返回初始位置 | - |
| `to_delta_pose` | 移动到相对增量位置 | delta_x, delta_y, delta_z |
| `run_point` | 移动到指定坐标点 | point_list: [x, y, z, r] |
| `get_feed` | 持续读取机器人反馈数据 | feed: DobotApi |
| `wait_arrive` | 等待机器人到达目标位置 | point_list: 目标坐标 |
| `clear_robot_error` | 监控并处理机器人错误 | dashboard: DobotApiDashboard |

### Yolov8Engine2

| 方法 | 作用 | 关键参数 |
|------|------|----------|
| `__init__` | 加载 ONNX 模型 | model_path, conf, iou |
| `load` | 初始化推理会话和配置 | - |
| `inference` | 执行单帧推理 | color_frame: ndarray |
| `preprocess` | 图像预处理 | color_frame: ndarray |
| `postprocess` | 解析输出并应用 NMS | outputs: list |
| `latest_res` | 获取最新推理结果 (线程安全) | - |

### TCPServer

| 方法 | 作用 | 关键参数 |
|------|------|----------|
| `__init__` | 初始化服务器 socket | host, port |
| `start` | 启动服务器监听循环 | - |
| `handle_client` | 处理单个客户端连接 | client_socket, address |
| `send_msg` | 向当前客户端发送消息 | msg: str |
| `msg` | 阻塞获取下一条消息 (property) | - |

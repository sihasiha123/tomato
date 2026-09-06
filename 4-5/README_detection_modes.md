# 4-5 检测节点版本说明

这份说明只管 `demo/4-5/` 下面这套。

## 版本一：对轨成功后停 YOLO

文件：

- `new_copy_no_capture.py`

特点：

- 等待导航时，默认不跑推理
- 对轨打开后正常检测
- 一旦对轨成功并触发工作流，就立即暂停轨道检测和 YOLO
- 等这一轮上轨/下轨/横移都完成、下一轮重新开放对轨后，再恢复检测
- 这个版本最省资源，优先把资源让给 FAST / 雷达 / 控制

补充：

- 这个版本在“正在对轨”的阶段还支持限频
- 默认 `RAIL_YOLO_MAX_FPS=8`

## 版本二：不停 YOLO，但可开关限频

文件：

- `new_copy_no_capture_rate_limit.py`

特点：

- 不会因为“对轨成功”而停掉检测
- 工作流进行中也继续跑检测
- 通过环境变量决定是否限频
- 这个版本适合你想持续看轨道结果，但又想给 FAST 多留一点资源的时候

默认行为：

- `RAIL_WAIT_NAV_INFER_ENABLE=1`
- `RAIL_YOLO_LIMIT_ENABLE=1`
- `RAIL_YOLO_MAX_FPS=8`

也就是说，默认是“持续检测 + 开启限频”。

## 相关节点

大流程节点：

- `traverse.py`

检测节点，二选一：

- `new_copy_no_capture.py`
- `new_copy_no_capture_rate_limit.py`

测试导航到位标志位：

- `send_fake_nav_success.py`

## 推荐启动顺序

1. 先启动 ROS Master、FAST-LIVO、底盘/CAN、相机这些基础节点
2. 启动 `traverse.py`
3. 启动一个检测节点（二选一）
4. 如果不用真导航，就在车已经到起始轨道入口后，手动发一次 fake nav success

## 启动命令

在 `demo/4-5/` 目录下：

### 1) 启动 traverse

```bash
python3 traverse.py
```

### 2A) 启动“对轨成功后停 YOLO”版

```bash
python3 new_copy_no_capture.py
```

### 2B) 启动“连续检测 + 可限频”版

```bash
python3 new_copy_no_capture_rate_limit.py
```

### 3) 测试时手动发导航成功

```bash
python3 send_fake_nav_success.py --once --repeat 8 --yaw-deg 0
```

只有不用真导航时才需要这一步。

## 常用环境变量

### 共同可用

```bash
export RAIL_YOLO_WEIGHTS=/home/ubuntu/mmLaneDet-master/work_dirs/yolov8s_421_from_new_e80/weights/best.pt
export RAIL_YOLO_DEVICE=0
export RAIL_YOLO_IMGSZ=960
export RAIL_YOLO_CONF=0.20
```

### 版本一：停 YOLO 版

正在对轨时可限频：

```bash
export RAIL_YOLO_MAX_FPS=8
```

如果你想等待导航时也继续跑推理：

```bash
export RAIL_WAIT_NAV_INFER_ENABLE=1
```

### 版本二：连续检测 + 可限频版

开启限频：

```bash
export RAIL_YOLO_LIMIT_ENABLE=1
export RAIL_YOLO_MAX_FPS=8
```

关闭限频：

```bash
export RAIL_YOLO_LIMIT_ENABLE=0
```

如果你只想在开放对轨后再推理：

```bash
export RAIL_WAIT_NAV_INFER_ENABLE=0
```

## 我建议怎么选

如果你现在最关心 FAST 慢、下轨回零超调，先用：

- `new_copy_no_capture.py`

如果你想保留持续检测画面，同时减少资源占用，再用：

- `new_copy_no_capture_rate_limit.py`

## 已做检查

下面这些文件已通过 `python3 -m py_compile` 语法检查：

- `traverse.py`
- `new_copy_no_capture.py`
- `new_copy_no_capture_rate_limit.py`


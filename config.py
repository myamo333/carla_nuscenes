# 接続・シミュレーション
CARLA_HOST = "192.168.11.20"
CARLA_PORT = 2000
TOWN = "Town02"
DURATION_SEC = 30

# 画像
IMG_W = 1600
IMG_H = 900
CAM_SENSOR_TICK = 0.0666667  # 15 Hz程度

# レーダー
RADAR_HFOV = 20
RADAR_VFOV = 5
RADAR_RANGE = 250

# LIDAR
LIDAR_RANGE = 120
LIDAR_CHANNELS = 32
LIDAR_PPS = 600000
LIDAR_ROTATION_HZ = 20
LIDAR_UPPER_FOV = 10
LIDAR_LOWER_FOV = -30
LIDAR_SENSOR_TICK = 1.0 / LIDAR_ROTATION_HZ

# 出力
BASE_DIR = "./data/nuscenes_eval"
VERSION = "v1.0-eval"

# カメラ（元コードのまま）
CAM_NAMES = [
    "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
    "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"
]
# cam_params: (x, y, z, yaw, fov_type)
# fov_type: "70" または "110"（使うBPを切替）
CAM_PARAMS = [
    (-1.5, 0,   2,    0,    "70"),
    ( 1.5, 0.7, 2,   55,    "70"),
    ( 1.5,-0.7, 2,  -55,    "70"),
    (-0.7, 0,   2, -110,    "70"),
    (-1.5, 0,   2,  180,   "110"),
    (-0.7, 0,   2,  110,    "70"),
]

# レーダー（元コードのまま）
RADAR_NAMES = [
    "RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT",
    "RADAR_BACK_LEFT", "RADAR_BACK_RIGHT"
]
# (x, y, z, yaw, name)
RADAR_PARAMS = [
    ( 2.0,  0.0, 1.0,    0, "RADAR_FRONT"),
    ( 1.5, -0.7, 1.0,  -55, "RADAR_FRONT_LEFT"),
    ( 1.5,  0.7, 1.0,   55, "RADAR_FRONT_RIGHT"),
    (-1.5, -0.7, 1.0, -125, "RADAR_BACK_LEFT"),
    (-1.5,  0.7, 1.0,  125, "RADAR_BACK_RIGHT"),
]

# LIDAR（元コードのまま）
LIDAR_NAME = "LIDAR_TOP"

# サンプル時刻
SAMPLE_INTERVAL_US = 500_000  # 0.5s

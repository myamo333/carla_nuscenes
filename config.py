# 接続・シミュレーション
CARLA_HOST = "192.168.11.20"
CARLA_PORT = 2000
TOWN = "Town02"
DURATION_SEC = 20

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
LIDAR_ROTATION_HZ = 60
LIDAR_UPPER_FOV = 10
LIDAR_LOWER_FOV = -30
LIDAR_SENSOR_TICK = 1.0 / LIDAR_ROTATION_HZ

# 出力
BASE_DIR = "./data/nuScenes"
VERSION = "v1.0-test"

# カメラ（元コードのまま）
CAM_NAMES = [
    "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
    "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"
]

# ========== nuScenes 由来のカメラ実キャリブ（必要分だけ追加） ==========
# ここに sensor.json / calibrated_sensor.json から手で抜き出した数値を入れる。
CAM_CONFIGS = {
    "CAM_FRONT": {
        "translation": [1.72200568478, 0.00475453292289, 1.49491291905],
        "rotation_wxyz": [0.5077241387638071, -0.4973392230703816,
                          0.49837167536166627, -0.4964832014373754],
        "intrinsic": [
            [1252.8131021185304, 0.0, 826.588114781398],
            [0.0, 1252.8131021185304, 469.9846626224581],
            [0.0, 0.0, 1.0]
        ],
        # "sensor_tick": 0.0666667,  # 個別に変えたい場合だけ指定
    },
    "CAM_FRONT_RIGHT": {
        "translation": [1.58082565783, -0.499078711449, 1.51749368405],
        "rotation_wxyz": [0.20335173766558642, -0.19146333228946724, 
                          0.6785710044972951, -0.6793609166212989],
        "intrinsic": [
            [1256.7485116440405, 0.0, 817.7887570959712],
            [0.0, 1256.7485116440403, 451.9541780095127],
            [0.0, 0.0, 1.0]
        ],
    },
    "CAM_FRONT_LEFT": {
        "translation": [1.5752559464, 0.500519383135, 1.50696032589],
        "rotation_wxyz": [0.6812088525125634, -0.6687507165046241,
                          0.2101702448905517, -0.21108161122114324],
        "intrinsic": [
            [1257.8625342125129, 0.0, 827.2410631095686],
            [0.0, 1257.8625342125129, 450.915498205774],
            [0.0, 0.0, 1.0]
        ],
    },
    "CAM_BACK": {
        "translation": [0.05524611077, 0.0107882366898, 1.56794286957],
        "rotation_wxyz": [0.5067997344989889, -0.4977567019405021,
                          -0.4987849934090844, 0.496594225837321],
        "intrinsic": [
            [796.8910634503094, 0.0, 857.7774326863696],
            [0.0, 796.8910634503094, 476.8848988407415],
            [0.0, 0.0, 1.0]
        ],
    },
    "CAM_BACK_LEFT": {
        "translation": [1.04852047718, 0.483058131052, 1.56210154484],
        "rotation_wxyz": [0.7048620297871717, -0.6907306801461466,
                          -0.11209091960167808, 0.11617345743327073],
        "intrinsic": [
            [1254.9860565800168, 0.0, 829.5769333630991],
            [0.0, 1254.9860565800168, 467.1680561863987],
            [0.0, 0.0, 1.0]
        ],
    },
    "CAM_BACK_RIGHT": {
        "translation": [1.05945173053, -0.46720294852, 1.55050857555],
        "rotation_wxyz": [0.13819187705364147, -0.13796718183628456,
                          -0.6893329941542625, 0.697630335509333],
        "intrinsic": [
            [1249.9629280788233, 0.0, 825.3768045375984],
            [0.0, 1249.9629280788233, 462.54816385708756],
            [0.0, 0.0, 1.0]
        ],
    },
}

# intrinsics が無いカメラに使うフォールバック FOV
CAM_DEFAULT_FOV = 70.0

# レーダー（元コードのまま）
RADAR_NAMES = [
    "RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT",
    "RADAR_BACK_LEFT", "RADAR_BACK_RIGHT"
]

RADAR_CONFIGS = {
    "RADAR_FRONT": {
        "translation": [3.412, 0.0, 0.5],
        "rotation_wxyz": [0.9999974259839071, 0.0, 0.0, -0.0022689260808398757],  
    },
    "RADAR_FRONT_LEFT": {
        "translation": [2.422, 0.8, 0.78],
        "rotation_wxyz": [0.7028982997921758, 0.0, 0.0, 0.7112903627557937], 
    },
    "RADAR_FRONT_RIGHT": {
        "translation": [2.422, -0.8, 0.77],
        "rotation_wxyz": [0.7087093341000862, 0.0, 0.0, -0.7055005880645404], 
    },
    "RADAR_BACK_LEFT": {
        "translation": [-0.562, 0.628, 0.53],
        "rotation_wxyz": [0.0458860416542946, 0.0, 0.0, 0.9989466808500344],  
    },
    "RADAR_BACK_RIGHT": {
        "translation": [-0.562, -0.618, 0.53],
        "rotation_wxyz": [0.04361938736533623, 0.0, 0.0, -0.9990482215818578], 
    },
}
# LIDAR（元コードのまま）
LIDAR_NAME = "LIDAR_TOP"

LIDAR_CONFIGS = {
    "LIDAR_TOP": {
        "translation": [0.985793, 0.0, 1.84019],
        "rotation_wxyz": [0.706749235646644, -0.015300993788500868, 0.01739745181256607, -0.7070846669051719],
    }
}

# サンプル時刻
SAMPLE_INTERVAL_US = 500_000  # 0.5s

# ===== NPC (前方に置く車) =====
NPC_ENABLED = True               # 置きたいとき True
NPC_MODEL = "vehicle.tesla.model3"  # 見つからなければ自動で別モデルを試します
NPC_AHEAD_METERS = 15.0          # 自車（Prius）の前方距離[m]
NPC_AUTOPILOT = False            # True にすると交通流に乗って走ります
NPC_SPAWN_Z_OFFSET = 0.5         # 地面から少し浮かせてスポーン（地面貫通防止）

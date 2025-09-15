import os
import numpy as np
import shutil
import carla
import config
from utils import make_directory
import math

def _set_camera_attr(bp, fov: float):
    bp.set_attribute('image_size_x', str(config.IMG_W))
    bp.set_attribute('image_size_y', str(config.IMG_H))
    bp.set_attribute('fov', str(fov))
    bp.set_attribute('sensor_tick', str(config.CAM_SENSOR_TICK))

def prepare_camera_bps(bl):
    cam_70 = bl.find('sensor.camera.rgb')
    _set_camera_attr(cam_70, 70)
    cam_110 = bl.find('sensor.camera.rgb')
    _set_camera_attr(cam_110, 110)
    return cam_70, cam_110

# [ADDED] ===== nuScenes → CARLA 変換ユーティリティ =====
def _quat_wxyz_to_rotmat(qw, qx, qy, qz):
    w, x, y, z = float(qw), float(qx), float(qy), float(qz)
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),    2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),    2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),    1-2*(x*x+y*y)],
    ], dtype=float)

def _rotmat_to_rpy_deg(R):
    pitch = math.degrees(math.asin(-float(R[2,0])))
    roll  = math.degrees(math.atan2(float(R[2,1]), float(R[2,2])))
    yaw   = math.degrees(math.atan2(float(R[1,0]), float(R[0,0])))
    return roll, pitch, yaw

def nus_cam_to_carla_transform(translation, rotation_wxyz):
    """
    nuScenes calibrated_sensor (translation, rotation[wxyz]=sensor→ego) → CARLA Transform（カメラ用）
    座標軸:
      nuScenes ego:  x前, y左, z上
      nuScenes cam:  x右, y下, z前
      CARLA(ego/センサ): x前, y右, z上
    """
    # 平行移動: y 反転
    x, y, z = [float(v) for v in translation]
    loc = carla.Location(x=x, y=-y, z=z)

    # 回転: R_se (sensor→ego) を取得 → 逆回転 R_es = R_se^T
    qw, qx, qy, qz = [float(v) for v in rotation_wxyz]
    R_se = _quat_wxyz_to_rotmat(qw, qx, qy, qz)
    # R_es = R_se.T

    # 軸変換: R_car = C_cam @ R_es @ S_ego
    S_ego = np.diag([1, -1, 1])  # nuScenes ego → CARLA ego
    C_cam = np.array([[0, 0, 1],  # nuScenes cam → CARLA cam（(x',y',z') = (z, x, -y)）
                      [1, 0, 0],
                      [0,-1, 0]], dtype=float)

    R_car = S_ego @ R_se @ C_cam.T     
    roll, pitch, yaw = _rotmat_to_rpy_deg(R_car)
    rot = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    return loc, rot

def hfov_from_intrinsics(K, img_w):
    fx = float(K[0][0])
    return math.degrees(2.0 * math.atan(img_w / (2.0 * fx)))

def attach_cameras(world, bl, vehicle, sweeps_dir):
    cam_70_bp, cam_110_bp = prepare_camera_bps(bl)
    actors = []
    captured = {name: [] for name in config.CAM_NAMES}

    def make_callback(cam_name):
        def callback(image: carla.Image):
            ts = int(image.timestamp * 1e6)
            frame = image.frame
            path = os.path.join(sweeps_dir, cam_name, f"{cam_name}_{frame}.png")
            make_directory(os.path.dirname(path))
            image.save_to_disk(path)
            captured[cam_name].append({"frame": frame, "path": path, "timestamp": ts})
        return callback

    for name in config.CAM_NAMES:
        cal = config.CAM_CONFIGS[name]
        # cfg は config.CAM_CONFIGS[name] から取り出した nuScenes の値
        loc, rot = nus_cam_to_carla_transform(cal["translation"], cal["rotation_wxyz"])
        bp = bl.find('sensor.camera.rgb')
        
        # intrinsics からFOV算出（既存の関数でOK）
        if cal.get("intrinsic"):
            fov = hfov_from_intrinsics(cal["intrinsic"], config.IMG_W)
        else:
            fov = config.CAM_DEFAULT_FOV
        bp.set_attribute('image_size_x', str(config.IMG_W))
        bp.set_attribute('image_size_y', str(config.IMG_H))
        bp.set_attribute('fov', f'{hfov_from_intrinsics(cal["intrinsic"], config.IMG_W):.6f}')
        bp.set_attribute('sensor_tick', str(config.CAM_SENSOR_TICK))
        
        actor = world.spawn_actor(bp, carla.Transform(loc, rot), attach_to=vehicle)
        actor.listen(make_callback(name))
        actors.append(actor)
        make_directory(os.path.join(sweeps_dir, name))

    return actors, captured

def _nus_radar_to_carla_transform(translation, rotation_wxyz):
    """
    nuScenes calibrated_sensor:
      - translation: [x,y,z]  (ego系: x前, y左, z上)
      - rotation[w,x,y,z]: R_se（sensor→ego）
    CARLA(親=車両)のローカル系: x前, y右, z上
    Radar/LiDAR は nuScenes のセンサ軸 ≒ ego軸なので、軸入替は不要、y反転のみ。

    回転変換の考え方:
      v_carla = S_ego * v_ego
      v_ego = R_se * v_sensor(nus)
      v_sensor(carla) = S_ego * v_sensor(nus)  （センサ軸も y のみ反転）
      ⇒ v_carla = (S_ego * R_se * S_ego) * v_sensor(carla)
      よって R_car = S_ego * R_se * S_ego
    CARLA に与える回転は「親(車)→子(センサ)」ではなく、センサ姿勢そのものなので
    上式の R_car を roll/pitch/yaw に分解して渡せば良い。
    """
    # 平行移動: y 反転
    x, y, z = [float(v) for v in translation]
    loc = carla.Location(x=x, y=-y, z=z)

    # 回転
    qw, qx, qy, qz = [float(v) for v in rotation_wxyz]
    R_se = _quat_wxyz_to_rotmat(qw, qx, qy, qz)
    S = np.diag([1, -1, 1])            # nuScenes → CARLA の符号反転（yのみ）
    R_car = S @ R_se @ S               # 上で導いた式
    roll, pitch, yaw = _rotmat_to_rpy_deg(R_car)
    rot = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    return loc, rot

def prepare_radar_bp(bl):
    bp = bl.find('sensor.other.radar')
    bp.set_attribute('horizontal_fov', str(config.RADAR_HFOV))
    bp.set_attribute('vertical_fov', str(config.RADAR_VFOV))
    bp.set_attribute('range', str(config.RADAR_RANGE))
    return bp

def attach_radars(world, bl, vehicle, sweeps_dir):
    bp = prepare_radar_bp(bl)
    actors = []
    captured = {name: [] for name in config.RADAR_NAMES}

    def make_callback(radar_name):
        def callback(radar_data: carla.RadarMeasurement):
            ts = int(radar_data.timestamp * 1e6)
            frame = radar_data.frame
            path = os.path.join(sweeps_dir, radar_name, f"{radar_name}_{frame}.bin")
            make_directory(os.path.dirname(path))
            pts = np.array([[d.depth, d.azimuth, d.altitude, d.velocity] for d in radar_data], dtype=np.float32)
            pts.tofile(path)
            captured[radar_name].append({"frame": frame, "path": path, "timestamp": ts})
        return callback

    for rname in config.RADAR_NAMES:
        t = config.RADAR_CONFIGS[rname]["translation"]
        q = config.RADAR_CONFIGS[rname]["rotation_wxyz"]
        loc, rot = _nus_radar_to_carla_transform(t, q)
        trans = carla.Transform(loc, rot)

        actor = world.spawn_actor(bp, trans, attach_to=vehicle)
        actor.listen(make_callback(rname))
        actors.append(actor)
        make_directory(os.path.join(sweeps_dir, rname))
    return actors, captured

def prepare_lidar_bp(bl):
    bp = bl.find('sensor.lidar.ray_cast')
    bp.set_attribute('range', str(config.LIDAR_RANGE))
    bp.set_attribute('channels', str(config.LIDAR_CHANNELS))
    bp.set_attribute('points_per_second', str(config.LIDAR_PPS))
    bp.set_attribute('rotation_frequency', str(config.LIDAR_ROTATION_HZ))
    bp.set_attribute('horizontal_fov','360') 
    bp.set_attribute('upper_fov', str(config.LIDAR_UPPER_FOV))
    bp.set_attribute('lower_fov', str(config.LIDAR_LOWER_FOV))
    bp.set_attribute('sensor_tick', str(config.LIDAR_SENSOR_TICK))
    return bp

def _nus_lidar_to_carla_transform(translation, rotation_wxyz):
    """
    nuScenes calibrated_sensor:
      translation: [x,y,z] (ego: x前, y左, z上)
      rotation[w,x,y,z]: R_se (sensor→ego)

    CARLA(親=車両)のローカル: x前, y右, z上

    Radar と同じく、センサ軸 ≒ ego軸なので軸入替不要。
    ただし y の符号反転が必要。
    回転: R_car = S * R_se * S,  S = diag(1,-1,1)
    """
    # 平行移動: y 反転
    x, y, z = [float(v) for v in translation]
    loc = carla.Location(x=x, y=-y, z=z)

    # 回転
    qw, qx, qy, qz = [float(v) for v in rotation_wxyz]
    R_se = _quat_wxyz_to_rotmat(qw, qx, qy, qz)
    S = np.diag([1, -1, 1])
    R_car = S @ R_se @ S
    roll, pitch, yaw = _rotmat_to_rpy_deg(R_car)
    rot = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    return loc, rot

def attach_lidar(world, bl, vehicle, sweeps_dir):
    bp = prepare_lidar_bp(bl)

    t = config.LIDAR_CONFIGS[config.LIDAR_NAME]["translation"]
    q = config.LIDAR_CONFIGS[config.LIDAR_NAME]["rotation_wxyz"]
    loc, rot = _nus_lidar_to_carla_transform(t, q)
    trans = carla.Transform(loc, rot)

    actor = world.spawn_actor(bp, trans, attach_to=vehicle)
    captured = []

    # sensors.py の attach_lidar 内コールバック
    def callback(lidar_data: carla.LidarMeasurement):
        ts = int(lidar_data.timestamp * 1e6)
        frame = lidar_data.frame
        # nuScenesのLiDARは拡張子が .pcd.bin（中身は5floatバイナリ）
        path = os.path.join(
            sweeps_dir,
            config.LIDAR_NAME,
            f"{config.LIDAR_NAME}_{frame}.pcd.bin",
        )
        make_directory(os.path.dirname(path))

        # CARLA: (x,y,z,intensity)[float32]  →  nuScenes: y 反転 + ring列追加(0埋め) の 5float
        pts4 = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        # nuScenes軸: x前+, y左+, z上+ → y を反転
        pts4[:, 1] *= -1.0

        # 追加: nuScenesが期待する 5floatに合わせて ring=0 列を追加
        ring = np.zeros((pts4.shape[0], 1), dtype=np.float32)
        pts5 = np.hstack([pts4, ring]).astype(np.float32)

        # 5float で保存
        pts5.tofile(path)

        captured.append({"frame": frame, "path": path, "timestamp": ts})


    actor.listen(callback)
    make_directory(os.path.join(sweeps_dir, config.LIDAR_NAME))
    return actor, captured

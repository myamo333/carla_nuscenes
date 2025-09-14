import os
import numpy as np
import shutil
import carla
import config
from utils import make_directory

def _set_camera_attr(bp, fov: float):
    bp.set_attribute('image_size_x', str(config.IMG_W))
    bp.set_attribute('image_size_y', str(config.IMG_H))
    bp.set_attribute('fov', str(fov))
    bp.set_attribute('sensor_tick', str(config.CAM_SENSOR_TICK))

def prepare_camera_bps(bl):
    cam_70 = bl.find('sensor.camera.rgb')
    _set_camera_attr(cam_70, 70)
    cam_110 = bl.find('sensor.camera.rgb')
    _set_camera_attr(cam_110, 100)
    return cam_70, cam_110

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

    for (x, y, z, yaw, fov_type), name in zip(config.CAM_PARAMS, config.CAM_NAMES):
        bp = cam_110_bp if fov_type == "110" else cam_70_bp
        trans = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
        actor = world.spawn_actor(bp, trans, attach_to=vehicle)
        actor.listen(make_callback(name))
        actors.append(actor)
        make_directory(os.path.join(sweeps_dir, name))
    return actors, captured

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

    for x, y, z, yaw, rname in config.RADAR_PARAMS:
        trans = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
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

def attach_lidar(world, bl, vehicle, sweeps_dir):
    bp = prepare_lidar_bp(bl)
    trans = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.5), carla.Rotation(yaw=0))
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

        # 現状: CARLAは (x,y,z,intensity) の4floatで来る（CARLA軸: x前+, y右+, z上+）
        # frombuffer は読み取り専用なので、後段の変換のために copy() で書き込み可能にする。
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

import carla
import time
import os
import json
import uuid
from PIL import Image
from datetime import datetime
import shutil
import numpy as np   # レーダー/ライダー データ保存に必要

def make_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

def set_camera(cam_bp, x, y, z, yaw, vehicle):
    camera_trans = carla.Transform(carla.Location(x=x, y=y, z=z),
                                   carla.Rotation(yaw=yaw))
    cam = world.spawn_actor(cam_bp, camera_trans, attach_to=vehicle)
    return cam
# ==== ③ sample_data.json に prev / next を付与 ====
def link_prev_next(sample_data_list):
    """
    sensor_token ごとに sample_data を timestamp 順に並べ、
    各レコードに prev / next を付与して返す。
    """
    from collections import defaultdict
    by_sensor = defaultdict(list)
    for rec in sample_data_list:
        by_sensor[rec["sensor_token"]].append(rec)

    for records in by_sensor.values():
        records.sort(key=lambda r: r["timestamp"])
        for i, r in enumerate(records):
            r["prev"] = records[i-1]["token"] if i > 0 else ""
            r["next"] = records[i+1]["token"] if i < len(records)-1 else ""
if __name__ == "__main__":
    # === Carla 初期化 ===
    client = carla.Client("192.168.11.20", 2000)
    client.set_timeout(10.0)
    world = client.load_world("Town02")
    world.set_weather(carla.WeatherParameters.ClearNoon)

    bl = world.get_blueprint_library()
    prius_bp = bl.find("vehicle.toyota.prius")

    # --- カメラ BluePrint ---
    cam_70_bp = bl.find('sensor.camera.rgb')
    for k, v in [('image_size_x', "1600"), ('image_size_y', "900"),
                 ('fov', "70"), ('sensor_tick', "0.0666667")]:
        cam_70_bp.set_attribute(k, v)
    cam_110_bp = bl.find('sensor.camera.rgb')
    for k, v in [('image_size_x', "1600"), ('image_size_y', "900"),
                 ('fov', "100"), ('sensor_tick', "0.0666667")]:
        cam_110_bp.set_attribute(k, v)

    cam_names = [
        "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
        "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"
    ]
    cam_params = [
        (-1.5,0,2,0, cam_70_bp),
        (1.5,0.7,2,55, cam_70_bp),
        (1.5,-0.7,2,-55, cam_70_bp),
        (-0.7,0,2,-110, cam_70_bp),
        (-1.5,0,2,180, cam_110_bp),
        (-0.7,0,2,110, cam_70_bp)
    ]

    # --- レーダー BluePrint ---
    radar_names = [
        "RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT",
        "RADAR_BACK_LEFT", "RADAR_BACK_RIGHT"
    ]
    radar_params = [
        (2.0, 0.0, 1.0,   0,   "RADAR_FRONT"),
        (1.5,-0.7, 1.0, -55,   "RADAR_FRONT_LEFT"),
        (1.5, 0.7, 1.0,  55,   "RADAR_FRONT_RIGHT"),
        (-1.5,-0.7,1.0,-125,   "RADAR_BACK_LEFT"),
        (-1.5, 0.7,1.0, 125,   "RADAR_BACK_RIGHT")
    ]

    # === 出力ディレクトリ ===
    base_dir = "./data/nuscenes_eval"
    samples_dir = os.path.join(base_dir, "samples")
    sweeps_dir = os.path.join(base_dir, "sweeps")
    maps_dir = os.path.join(base_dir, "maps")
    for d in [samples_dir, sweeps_dir, maps_dir]:
        os.makedirs(d, exist_ok=True)
    for name in cam_names + radar_names:
        make_directory(os.path.join(samples_dir, name))
        make_directory(os.path.join(sweeps_dir, name))

    # ★LIDAR 追加: ディレクトリ
    lidar_name = "LIDAR_TOP"
    make_directory(os.path.join(samples_dir, lidar_name))
    make_directory(os.path.join(sweeps_dir, lidar_name))

    # ダミーマップ
    Image.new('RGB', (1, 1), color=(0,0,0)).save(os.path.join(maps_dir, "eval_map.png"))

    # === Prius 車両 ===
    prius = world.spawn_actor(prius_bp, world.get_map().get_spawn_points()[0])

    # === カメラ設定と撮影 ===
    cam_actors = []
    captured_images = {name: [] for name in cam_names}
    for (x,y,z,yaw,bp), name in zip(cam_params, cam_names):
        cam = set_camera(bp, x, y, z, yaw, prius)
        cam_actors.append(cam)
        def make_callback(cam_name):
            def callback(image):
                ts = int(image.timestamp * 1e6)
                frame = image.frame
                path = os.path.join(sweeps_dir, cam_name, f"{cam_name}_{frame}.png")
                image.save_to_disk(path)
                captured_images[cam_name].append({
                    "frame": frame,
                    "path": path,
                    "timestamp": ts
                })
            return callback
        cam.listen(make_callback(name))

    # === レーダー設定と撮影 ===
    radar_bp = bl.find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', '20')
    radar_bp.set_attribute('vertical_fov', '5')
    radar_bp.set_attribute('range', '250')

    radar_actors = []
    captured_radar = {name: [] for name in radar_names}

    def make_radar_callback(radar_name):
        def callback(radar_data):
            ts = int(radar_data.timestamp * 1e6)
            frame = radar_data.frame
            path = os.path.join(sweeps_dir, radar_name, f"{radar_name}_{frame}.bin")
            pts = np.array([[d.depth, d.azimuth, d.altitude, d.velocity] for d in radar_data], dtype=np.float32)
            pts.tofile(path)
            captured_radar[radar_name].append({
                "frame": frame,
                "path": path,
                "timestamp": ts
            })
        return callback

    for x, y, z, yaw, rname in radar_params:
        radar = world.spawn_actor(
            radar_bp,
            carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw)),
            attach_to=prius
        )
        radar.listen(make_radar_callback(rname))
        radar_actors.append(radar)

    # === ★LIDAR 追加: センサー生成 & コールバック ===
    lidar_bp = bl.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range','120')
    lidar_bp.set_attribute('channels','32')
    lidar_bp.set_attribute('points_per_second','600000')
    lidar_bp.set_attribute('rotation_frequency','20')   # 20 Hz
    lidar_bp.set_attribute('upper_fov','10')
    lidar_bp.set_attribute('lower_fov','-30')
    lidar_bp.set_attribute('sensor_tick','0.05')        # 1/20 = 0.05s

    lidar_actor = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(x=0.0, y=0.0, z=2.5), carla.Rotation(yaw=0)),
        attach_to=prius
    )
    captured_lidar = []

    def lidar_callback(lidar_data: carla.LidarMeasurement):
        ts = int(lidar_data.timestamp * 1e6)
        frame = lidar_data.frame
        path = os.path.join(sweeps_dir, lidar_name, f"{lidar_name}_{frame}.bin")

        # CARLAは (x,y,z,intensity) の4float
        pts4 = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4)

        # nuScenesは (x,y,z,intensity,ring) の5floatを要求 → ringを0で埋める
        ring = np.zeros((pts4.shape[0], 1), dtype=np.float32)
        pts5 = np.hstack([pts4, ring]).astype(np.float32)

        pts5.tofile(path)

        captured_lidar.append({
            "frame": frame,
            "path": path,
            "timestamp": ts
        })

    lidar_actor.listen(lidar_callback)

    # === 走行＆撮影 ===
    prius.set_autopilot(True)
    time.sleep(30)

    # === サンプル（keyframe）時刻を 0.5s 間隔で生成 ===
    interval_us = 500000
    if not any(captured_images[name] for name in cam_names):
        raise RuntimeError("画像が記録されていません。")
    # LIDAR があれば先頭 LIDAR も考慮（最初の sample に LIDAR を入れる）
    min_ts_candidates = [img["timestamp"] for imgs in captured_images.values() for img in imgs]
    if captured_lidar:
        min_ts_candidates.append(min(m["timestamp"] for m in captured_lidar))
    min_ts = min(min_ts_candidates)
    max_ts = min_ts + int(30 * 1e6)

    sample_times = []
    t = min_ts
    while t <= max_ts:
        sample_times.append(int(t))
        t += interval_us

    # === Keyframe 選定（各 sample_time × 各センサ ちょうど1件） ===
    # カメラ: samples/ へコピー & keyframe 記録
    key_img_for_idx = {name: {} for name in cam_names}
    for cam_name in cam_names:
        for idx, st in enumerate(sample_times):
            if not captured_images[cam_name]:
                continue
            closest = min(captured_images[cam_name], key=lambda img: abs(img["timestamp"] - st))
            src = closest["path"]
            dst = src.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            key_img_for_idx[cam_name][idx] = src  # keyframe は「sweeps 側の元パス」で記録

    # レーダー: samples/ に .bin をコピー & keyframe 記録
    key_radar_for_idx = {name: {} for name in radar_names}
    for rname in radar_names:
        for idx, st in enumerate(sample_times):
            if not captured_radar[rname]:
                continue
            closest = min(captured_radar[rname], key=lambda meas: abs(meas["timestamp"] - st))
            src = closest["path"]
            dst = src.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            key_radar_for_idx[rname][idx] = src

    # LIDAR: samples/ に .bin をコピー & keyframe 記録（各 sample_time で必ず1件）
    key_lidar_for_idx = {}
    if captured_lidar:
        for idx, st in enumerate(sample_times):
            closest = min(captured_lidar, key=lambda meas: abs(meas["timestamp"] - st))
            src = closest["path"]
            dst = src.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            key_lidar_for_idx[idx] = src

    # === NuScenes JSON生成 ===
    version = "v1.0-eval"
    out_dir = os.path.join(base_dir, version)
    os.makedirs(out_dir, exist_ok=True)

    log_token = str(uuid.uuid4())
    scene_token = str(uuid.uuid4())
    ego_pose_token = str(uuid.uuid4())

    # sample.json（0.5s ごと）
    sample_json = []
    prev_token = ""
    for i, st in enumerate(sample_times):
        token = str(uuid.uuid4())
        sample_json.append({
            "token": token,
            "scene_token": scene_token,
            "prev": prev_token if i > 0 else "",
            "next": "",
            "timestamp": int(st)
        })
        if prev_token:
            sample_json[-2]["next"] = token
        prev_token = token

    # scene.json
    scene_json = [{
        "token": scene_token,
        "name": "scene_1",
        "description": "Evaluation scene",
        "log_token": log_token,
        "nbr_samples": len(sample_json),
        "first_sample_token": sample_json[0]["token"],
        "last_sample_token": sample_json[-1]["token"]
    }]

    # ego_pose.json（簡易固定値）
    ego_pose_json = [{
        "token": ego_pose_token,
        "timestamp": sample_times[0],
        "rotation": [0,0,0,1],
        "translation": [0,0,0]
    }]

    sensor_json, calib_json, sample_data_json = [], [], []

    # ---- センサ登録（固定 token を各センサに付与）----
    per_cam_tokens = {}
    for cam_name in cam_names:
        s_token = str(uuid.uuid4())
        c_token = str(uuid.uuid4())
        per_cam_tokens[cam_name] = (s_token, c_token)
        sensor_json.append({
            "token": s_token,
            "modality": "camera",
            "name": cam_name,
            "channel": cam_name
        })
        calib_json.append({
            "token": c_token,
            "sensor_token": s_token,
            "translation": [0,0,0],
            "rotation": [0,0,0,1],
            "camera_intrinsic": [
                [1600,0,800],
                [0,900,450],
                [0,0,1]
            ]
        })

    per_radar_tokens = {}
    for rname in radar_names:
        s_token = str(uuid.uuid4())
        c_token = str(uuid.uuid4())
        per_radar_tokens[rname] = (s_token, c_token)
        sensor_json.append({
            "token": s_token,
            "modality": "radar",
            "name": rname,
            "channel": rname
        })
        calib_json.append({
            "token": c_token,
            "sensor_token": s_token,
            "translation": [0,0,0],
            "rotation": [0,0,0,1],
            "camera_intrinsic": []
        })

    lidar_s_token = str(uuid.uuid4())
    lidar_c_token = str(uuid.uuid4())
    sensor_json.append({
        "token": lidar_s_token,
        "modality": "lidar",
        "name": lidar_name,
        "channel": lidar_name
    })
    calib_json.append({
        "token": lidar_c_token,
        "sensor_token": lidar_s_token,
        "translation": [0,0,0],
        "rotation": [0,0,0,1],
        "camera_intrinsic": []
    })

    # ==== ① keyframe の sample_data を各 sample × 各センサ ちょうど1件だけ作成 ====
    def add_sd(sample_idx, sensor_tokens, src_path, fileformat, width=0, height=0):
        sample_token = sample_json[sample_idx]["token"]
        s_token, c_token = sensor_tokens
        if src_path.endswith(os.sep + "sweeps" + os.sep):
            # 実際は常に sweeps 側のパスを受け取り、samples に写し替えたものを参照する
            pass
        # samples/ の相対パスに差し替え
        rel = src_path.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
        rel = rel.replace(base_dir + os.sep, "")
        if fileformat == "pcd":
            rel = rel.replace(".bin", ".pcd")   # ✅ Radar用に拡張子をpcdに変更
        sample_data_json.append({
            "token": str(uuid.uuid4()),
            "sample_token": sample_token,
            "ego_pose_token": ego_pose_token,
            "calibrated_sensor_token": c_token,
            "sensor_token": s_token,
            "filename": rel,
            "fileformat": fileformat,
            "is_key_frame": True,
            "timestamp": int(sample_times[sample_idx]),
            "width": width,
            "height": height
        })

    # カメラ keyframe
    for cam_name in cam_names:
        s_token, c_token = per_cam_tokens[cam_name]
        for idx in range(len(sample_times)):
            src = key_img_for_idx.get(cam_name, {}).get(idx)
            if not src:
                continue
            add_sd(idx, (s_token, c_token), src, "png", width=1600, height=900)

    # レーダー keyframe
    for rname in radar_names:
        s_token, c_token = per_radar_tokens[rname]
        for idx in range(len(sample_times)):
            src = key_radar_for_idx.get(rname, {}).get(idx)
            if not src:
                continue
            add_sd(idx, (s_token, c_token), src, "pcd")

    # LIDAR keyframe
    for idx in range(len(sample_times)):
        src = key_lidar_for_idx.get(idx)
        if not src:
            continue
        add_sd(idx, (lidar_s_token, lidar_c_token), src, "bin")

    # ==== ② sweeps（非 keyframe）を登録（keyframe と同一フレームは除外）====
    def nearest_sample_index(ts):
        return min(range(len(sample_times)), key=lambda i: abs(sample_times[i] - ts))

    # カメラ sweeps
    for cam_name in cam_names:
        s_token, c_token = per_cam_tokens[cam_name]
        for img in captured_images[cam_name]:
            idx = nearest_sample_index(img["timestamp"])
            # keyframe で使った元 sweeps パスと同一なら除外
            if key_img_for_idx.get(cam_name, {}).get(idx) == img["path"]:
                continue
            rel = img["path"].replace(base_dir + os.sep, "")
            sample_data_json.append({
                "token": str(uuid.uuid4()),
                "sample_token": sample_json[idx]["token"],
                "ego_pose_token": ego_pose_token,
                "calibrated_sensor_token": c_token,
                "sensor_token": s_token,
                "filename": rel,
                "fileformat": "png",
                "is_key_frame": False,
                "timestamp": img["timestamp"],
                "width": 1600,
                "height": 900
            })

    # レーダー sweeps
    for rname in radar_names:
        s_token, c_token = per_radar_tokens[rname]
        for meas in captured_radar[rname]:
            idx = nearest_sample_index(meas["timestamp"])
            if key_radar_for_idx.get(rname, {}).get(idx) == meas["path"]:
                continue
            rel = meas["path"].replace(base_dir + os.sep, "")
            rel_pcd = rel.replace(".bin", ".pcd")
            sample_data_json.append({
                "token": str(uuid.uuid4()),
                "sample_token": sample_json[idx]["token"],
                "ego_pose_token": ego_pose_token,
                "calibrated_sensor_token": c_token,
                "sensor_token": s_token,
                "filename": rel_pcd,
                "fileformat": "pcd",
                "is_key_frame": False,
                "timestamp": meas["timestamp"],
                "width": 0,
                "height": 0
            })

    # LIDAR sweeps
    for meas in captured_lidar:
        idx = nearest_sample_index(meas["timestamp"])
        if key_lidar_for_idx.get(idx) == meas["path"]:
            continue
        rel = meas["path"].replace(base_dir + os.sep, "")
        sample_data_json.append({
            "token": str(uuid.uuid4()),
            "sample_token": sample_json[idx]["token"],
            "ego_pose_token": ego_pose_token,
            "calibrated_sensor_token": lidar_c_token,
            "sensor_token": lidar_s_token,
            "filename": rel,
            "fileformat": "bin",
            "is_key_frame": False,
            "timestamp": meas["timestamp"],
            "width": 0,
            "height": 0
        })

    # log.json / map.json
    log_json = [{
        "token": log_token,
        "location": "eval",
        "date_captured": datetime.now().strftime("%Y-%m-%d"),
        "logfile": "eval.log",
        "duration": 30.0
    }]
    link_prev_next(sample_data_json)
    map_json = [{
        "token": str(uuid.uuid4()),
        "filename": "maps/eval_map.png",
        "log_tokens": [log_token],
        "layer_names": ["road_segment","lane","stop_line"]
    }]

    # 保存
    out_dir = os.path.join(base_dir, version)
    with open(os.path.join(out_dir, "log.json"), "w") as f: json.dump(log_json, f, indent=2)
    with open(os.path.join(out_dir, "scene.json"), "w") as f: json.dump(scene_json, f, indent=2)
    with open(os.path.join(out_dir, "ego_pose.json"), "w") as f: json.dump(ego_pose_json, f, indent=2)
    with open(os.path.join(out_dir, "sensor.json"), "w") as f: json.dump(sensor_json, f, indent=2)
    with open(os.path.join(out_dir, "calibrated_sensor.json"), "w") as f: json.dump(calib_json, f, indent=2)
    with open(os.path.join(out_dir, "sample.json"), "w") as f: json.dump(sample_json, f, indent=2)
    with open(os.path.join(out_dir, "sample_data.json"), "w") as f: json.dump(sample_data_json, f, indent=2)
    for n in ["category.json","attribute.json","visibility.json","sample_annotation.json","instance.json"]:
        with open(os.path.join(out_dir, n), "w") as f: json.dump([], f, indent=2)
    with open(os.path.join(out_dir, "map.json"), "w") as f: json.dump(map_json, f, indent=2)

    print("NuScenes 形式：カメラ + 5レーダー + LIDAR_TOP（keyframeは各サンプル1件）+ sweeps（重複除外）で生成完了！")

    # 後片付け
    for cam in cam_actors: cam.destroy()
    for radar in radar_actors: radar.destroy()
    lidar_actor.destroy()
    prius.destroy()

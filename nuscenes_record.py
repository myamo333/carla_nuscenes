import carla
import time
import os
import json
import uuid
from PIL import Image
from datetime import datetime
import shutil
import numpy as np   # レーダーデータ保存に必要

def make_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

def set_camera(cam_bp, x, y, z, yaw, vehicle):
    camera_trans = carla.Transform(carla.Location(x=x, y=y, z=z),
                                   carla.Rotation(yaw=yaw))
    cam = world.spawn_actor(cam_bp, camera_trans, attach_to=vehicle)
    return cam

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
            # まずは .bin で保存（後で変換）
            path = os.path.join(sweeps_dir, radar_name, f"{radar_name}_{frame}.bin")
            pts = np.array([[d.depth, d.azimuth, d.altitude, d.velocity] for d in radar_data])
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

    # === 走行＆撮影 ===
    prius.set_autopilot(True)
    time.sleep(30)

    # === Keyframe 判定用 ===
    interval_us = 500000
    if not any(captured_images[name] for name in cam_names):
        raise RuntimeError("画像が記録されていません。")
    min_ts = min(img["timestamp"] for imgs in captured_images.values() for img in imgs)
    max_ts = min_ts + int(30 * 1e6)

    # サンプル時刻リスト（約60個）
    sample_times = []
    t = min_ts
    while t <= max_ts:
        sample_times.append(int(t))
        t += interval_us

    # ---- カメラ Keyframe コピー ----
    key_img_for_idx = {name: {} for name in cam_names}
    for cam_name in cam_names:
        if not captured_images[cam_name]:
            continue
        for idx, st in enumerate(sample_times):
            closest = min(captured_images[cam_name], key=lambda img: abs(img["timestamp"] - st))
            src = closest["path"]
            dst = src.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            key_img_for_idx[cam_name][idx] = src  # sweeps側のパスで記録

    # ---- レーダー Keyframe コピー（★修正: samples にも入れる） ----
    key_radar_for_idx = {name: {} for name in radar_names}
    for rname in radar_names:
        if not captured_radar[rname]:
            continue
        for idx, st in enumerate(sample_times):
            closest = min(captured_radar[rname], key=lambda meas: abs(meas["timestamp"] - st))
            src = closest["path"]  # sweeps 側の .bin
            dst = src.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            key_radar_for_idx[rname][idx] = src  # sweeps 側のパスで記録（判定に使う）

    # === NuScenes JSON生成 ===
    version = "v1.0-eval"
    out_dir = os.path.join(base_dir, version)
    os.makedirs(out_dir, exist_ok=True)

    log_token = str(uuid.uuid4())
    scene_token = str(uuid.uuid4())
    ego_pose_token = str(uuid.uuid4())

    # sample.json
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

    scene_json = [{
        "token": scene_token,
        "name": "scene_1",
        "description": "Evaluation scene",
        "log_token": log_token,
        "nbr_samples": len(sample_json),
        "first_sample_token": sample_json[0]["token"],
        "last_sample_token": sample_json[-1]["token"]
    }]

    ego_pose_json = [{
        "token": ego_pose_token,
        "timestamp": sample_times[0],
        "rotation": [0,0,0,1],
        "translation": [0,0,0]
    }]

    sensor_json, calib_json, sample_data_json = [], [], []
    # カメラ: 固定 token
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

    # camera sample_data
    for cam_name in cam_names:
        s_token, c_token = per_cam_tokens[cam_name]
        for img in captured_images[cam_name]:
            closest_idx = min(range(len(sample_json)),
                              key=lambda i: abs(sample_json[i]["timestamp"] - img["timestamp"]))
            target_sample_token = sample_json[closest_idx]["token"]
            # keyframe 判定: samples にコピーされたもの（sweeps パス一致）
            is_key = key_img_for_idx.get(cam_name, {}).get(closest_idx) == img["path"]
            # ファイルパス: keyframe は samples/ に差し替え
            if is_key:
                rel = img["path"].replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            else:
                rel = img["path"]
            rel = rel.replace(base_dir + os.sep, "")
            sd_token = str(uuid.uuid4())
            sample_data_json.append({
                "token": sd_token,
                "sample_token": target_sample_token,
                "ego_pose_token": ego_pose_token,
                "calibrated_sensor_token": c_token,
                "sensor_token": s_token,
                "filename": rel,
                "fileformat": "png",
                "is_key_frame": bool(is_key),
                "timestamp": img["timestamp"],
                "width": 1600,
                "height": 900
            })

    # radar sensor & sample_data（★修正: keyframe は samples/ パスに）
    for rname in radar_names:
        s_token = str(uuid.uuid4())
        c_token = str(uuid.uuid4())
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
        for meas in captured_radar[rname]:
            closest_idx = min(range(len(sample_json)),
                              key=lambda i: abs(sample_json[i]["timestamp"] - meas["timestamp"]))
            target_sample_token = sample_json[closest_idx]["token"]
            # keyframe 判定は「samples へコピーされた sweeps 元パス一致」で判定
            is_key = key_radar_for_idx.get(rname, {}).get(closest_idx) == meas["path"]
            # ファイルパス: keyframe は samples/ に差し替え
            if is_key:
                rel = meas["path"].replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            else:
                rel = meas["path"]
            rel = rel.replace(base_dir + os.sep, "")
            sd_token = str(uuid.uuid4())
            sample_data_json.append({
                "token": sd_token,
                "sample_token": target_sample_token,
                "ego_pose_token": ego_pose_token,
                "calibrated_sensor_token": c_token,
                "sensor_token": s_token,
                "filename": rel,
                "fileformat": "bin",   # 後で pcd 等に変換予定
                "is_key_frame": bool(is_key),
                "timestamp": meas["timestamp"],
                "width": 0,
                "height": 0
            })

    log_json = [{
        "token": log_token,
        "location": "eval",
        "date_captured": datetime.now().strftime("%Y-%m-%d"),
        "logfile": "eval.log",
        "duration": 30.0
    }]
    map_json = [{
        "token": str(uuid.uuid4()),
        "filename": "maps/eval_map.png",
        "log_tokens": [log_token],
        "layer_names": ["road_segment","lane","stop_line"]
    }]

    # 保存
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

    print("NuScenes 形式：カメラ + 5レーダーの samples / sweeps / JSON 一式を生成完了！")

    # 後片付け
    for cam in cam_actors:
        cam.destroy()
    for radar in radar_actors:
        radar.destroy()
    prius.destroy()

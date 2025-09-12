import carla
import time
import os
import json
import uuid
from PIL import Image
from datetime import datetime
import shutil

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

    # === 出力ディレクトリ ===
    base_dir = "./data/nuscenes_eval"  # dataroot
    samples_dir = os.path.join(base_dir, "samples")
    sweeps_dir = os.path.join(base_dir, "sweeps")
    maps_dir = os.path.join(base_dir, "maps")
    for d in [samples_dir, sweeps_dir, maps_dir]:
        os.makedirs(d, exist_ok=True)
    for name in cam_names:
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
                ts = int(image.timestamp * 1e6)  # µs
                frame = image.frame
                path = os.path.join(sweeps_dir, cam_name, f"{cam_name}_{frame}.png")
                image.save_to_disk(path)
                captured_images[cam_name].append({
                    "frame": frame,
                    "path": path,           # いったん sweeps に保存
                    "timestamp": ts
                })
            return callback
        cam.listen(make_callback(name))

    prius.set_autopilot(True)
    time.sleep(30)  # 30秒間撮影

    # === Keyframe を samples/ にコピー（0.5秒ごと、全カメラ） ===
    # サンプル時刻（0.5秒 = 500000 µs）を作成
    interval_us = 500000
    # 1枚も撮れていなければ以降はスキップ
    if not any(captured_images[name] for name in cam_names):
        raise RuntimeError("画像が記録されていません。Carla / センサーを確認してください。")

    min_ts = min(img["timestamp"] for imgs in captured_images.values() for img in imgs)
    max_ts = min_ts + int(30 * 1e6)

    # サンプル時刻リスト（60個前後）
    sample_times = []
    t = min_ts
    while t <= max_ts:
        sample_times.append(int(t))
        t += interval_us

    # カメラごとに、各サンプル時刻に最も近い1枚を samples にコピー
    # かつ「どのサンプル idx の keyframe か」を記録
    key_img_for_idx = {name: {} for name in cam_names}  # cam_name -> {idx: sweeps_path}
    for cam_name in cam_names:
        if not captured_images[cam_name]:
            continue
        for idx, st in enumerate(sample_times):
            closest = min(captured_images[cam_name], key=lambda img: abs(img["timestamp"] - st))
            src = closest["path"]  # sweeps 側のファイル
            dst = src.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            # 同一ファイル名でも毎回上書き（最も近い1枚でOK）
            shutil.copyfile(src, dst)
            key_img_for_idx[cam_name][idx] = src  # 「sweeps 側のパス」を記録（判定に使う）

    # === NuScenes JSON生成 ===
    version = "v1.0-eval"
    out_dir = os.path.join(base_dir, version)  # JSON は dataroot/v1.0-eval 配下
    os.makedirs(out_dir, exist_ok=True)

    log_token = str(uuid.uuid4())
    scene_token = str(uuid.uuid4())
    ego_pose_token = str(uuid.uuid4())

    # ---- sample.json（0.5秒ごと）----
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

    # ---- scene.json ----
    scene_json = [{
        "token": scene_token,
        "name": "scene_1",
        "description": "Evaluation scene",
        "log_token": log_token,
        "nbr_samples": len(sample_json),
        "first_sample_token": sample_json[0]["token"],
        "last_sample_token": sample_json[-1]["token"]
    }]

    # ---- ego_pose.json ----
    ego_pose_json = [{
        "token": ego_pose_token,
        "timestamp": sample_times[0],
        "rotation": [0,0,0,1],
        "translation": [0,0,0]
    }]

    # ---- sensor & calibrated_sensor & sample_data ----
    sensor_json, calib_json, sample_data_json = [], [], []
    # カメラごとに固定の token を発行
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

    # 画像 → 最近傍サンプル idx を求め、その sample_token に紐付け
    for cam_name in cam_names:
        s_token, c_token = per_cam_tokens[cam_name]
        for img in captured_images[cam_name]:
            # どの sample に属するか（最近傍の idx）
            closest_idx = min(range(len(sample_json)),
                              key=lambda i: abs(sample_json[i]["timestamp"] - img["timestamp"]))
            target_sample_token = sample_json[closest_idx]["token"]

            # その idx の keyframe に選ばれた画像と一致するか（sweeps パスで比較）
            is_key = key_img_for_idx.get(cam_name, {}).get(closest_idx) == img["path"]

            # nuScenes の filename は dataroot 相対。keyframe は samples/、それ以外は sweeps/
            if is_key:
                rel = img["path"].replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            else:
                rel = img["path"]
            # dataroot 相対にする
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
                "width": 1600,       # ← 追加
                "height": 900        # ← 追加
            })

    # ---- log.json & map.json ----
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

    # ---- 保存 ----
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

    print("NuScenes 形式：samples(0.5s keyframes) + sweeps(中間) + JSON 一式の生成完了！")

    # === 後片付け ===
    for cam in cam_actors:
        cam.destroy()
    prius.destroy()

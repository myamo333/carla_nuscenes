import carla
import time
import os
import json
import uuid
from PIL import Image
from datetime import datetime

# ==== ディレクトリ作成関数 ====
def make_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

# ==== カメラ設定関数 ====
def set_camera(cam_bp, x, y, z, yaw, vehicle):
    camera_trans = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
    cam = world.spawn_actor(cam_bp, camera_trans, attach_to=vehicle)
    return cam

# ==== メイン ====
if __name__ == "__main__":
    # ------------------------
    # Carla 初期化
    # ------------------------
    client = carla.Client("192.168.11.20", 2000)
    client.set_timeout(10.0)
    world = client.load_world("Town02")
    world.set_weather(carla.WeatherParameters.ClearNoon)

    bl = world.get_blueprint_library()
    prius_bp = bl.find("vehicle.toyota.prius")

    cam_70_bp = bl.find('sensor.camera.rgb')
    cam_70_bp.set_attribute('image_size_x', "1600")
    cam_70_bp.set_attribute('image_size_y', "900")
    cam_70_bp.set_attribute('fov', "70")
    cam_70_bp.set_attribute('sensor_tick', "0.0666667")  # 15fps

    cam_110_bp = bl.find('sensor.camera.rgb')
    cam_110_bp.set_attribute('image_size_x', "1600")
    cam_110_bp.set_attribute('image_size_y', "900")
    cam_110_bp.set_attribute('fov', "100")
    cam_110_bp.set_attribute('sensor_tick', "0.0666667")  # 15fps

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

    # ------------------------
    # 出力ディレクトリ作成
    # ------------------------
    base_dir = "./data/nuscenes_eval"
    samples_dir = os.path.join(base_dir, "samples")
    sweeps_dir = os.path.join(base_dir, "sweeps")
    maps_dir = os.path.join(base_dir, "maps")
    os.makedirs(samples_dir, exist_ok=True)
    os.makedirs(sweeps_dir, exist_ok=True)
    os.makedirs(maps_dir, exist_ok=True)

    for name in cam_names:
        make_directory(os.path.join(samples_dir, name))
        make_directory(os.path.join(sweeps_dir, name))

    # ダミーマップ PNG
    dummy_map = Image.new('RGB', (1, 1), color=(0,0,0))
    dummy_map.save(os.path.join(maps_dir, "eval_map.png"))

    # ------------------------
    # Prius 車両
    # ------------------------
    prius = world.spawn_actor(prius_bp, world.get_map().get_spawn_points()[0])

    # ------------------------
    # カメラスポーン & 画像保存
    # ------------------------
    cam_actors = []
    captured_images = {name: [] for name in cam_names}
    keyframe_saved = {name: False for name in cam_names}

    for (x,y,z,yaw,bp), name in zip(cam_params, cam_names):
        cam = set_camera(bp, x, y, z, yaw, prius)
        cam_actors.append(cam)

        def make_callback(cam_name):
            def callback(image):
                frame = image.frame
                ts = int(image.timestamp * 1e6)
                if not keyframe_saved[cam_name]:
                    path = os.path.join(samples_dir, cam_name, f"{cam_name}_{frame}.png")
                    keyframe_saved[cam_name] = True
                    is_key = True
                else:
                    path = os.path.join(sweeps_dir, cam_name, f"{cam_name}_{frame}.png")
                    is_key = False
                image.save_to_disk(path)
                captured_images[cam_name].append({
                    "path": path,
                    "timestamp": ts,
                    "is_key": is_key
                })
            return callback

        cam.listen(make_callback(name))

    # ------------------------
    # オートパイロット走行
    # ------------------------
    prius.set_autopilot(True)
    time.sleep(30)  # 30秒撮影

    # ------------------------
    # JSON 生成 (NuScenes 評価用)
    # ------------------------
    version = "v1.0-eval"
    out_dir = base_dir + '/v1.0-eval'
    os.makedirs(out_dir, exist_ok=True)
    now_ts = int(datetime.now().timestamp()*1e6)

    # Tokens
    log_token = str(uuid.uuid4())
    scene_token = str(uuid.uuid4())
    ego_pose_token = str(uuid.uuid4())
    sample_token = str(uuid.uuid4())

    # log.json
    log_json = [{
        "token": log_token,
        "location": "eval",
        "date_captured": datetime.now().strftime("%Y-%m-%d"),
        "logfile": "eval.log",
        "duration": 30.0
    }]
    with open(os.path.join(out_dir, "log.json"), "w") as f:
        json.dump(log_json, f, indent=2)

    # scene.json
    scene_json = [{
        "token": scene_token,
        "name": "scene_1",
        "description": "Evaluation scene",
        "log_token": log_token,
        "nbr_samples": 1,
        "first_sample_token": sample_token,   # ← 追加
        "last_sample_token": sample_token     # ← 追加
    }]
    with open(os.path.join(out_dir, "scene.json"), "w") as f:
        json.dump(scene_json, f, indent=2)

    # ego_pose.json
    ego_pose_json = [{
        "token": ego_pose_token,
        "timestamp": now_ts,   # ← 追加
        "rotation": [0,0,0,1],
        "translation": [0,0,0]
    }]
    with open(os.path.join(out_dir, "ego_pose.json"), "w") as f:
        json.dump(ego_pose_json, f, indent=2)

    # sensor.json / calibrated_sensor.json / sample.json / sample_data.json
    sensor_json = []
    calib_json = []
    sample_json = [{
        "token": sample_token,
        "scene_token": scene_token,
        "prev": "",
        "next": "",
        "timestamp": now_ts
    }]
    sample_data_json = []

    for cam_name in cam_names:
        s_token = str(uuid.uuid4())
        c_token = str(uuid.uuid4())
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
                [1600, 0, 800],
                [0, 900, 450],
                [0, 0, 1]
            ]
        })
        for img in captured_images[cam_name]:
            sd_token = str(uuid.uuid4())
            sample_data_json.append({
                "token": sd_token,
                "sample_token": sample_token,
                "ego_pose_token": ego_pose_token,
                "calibrated_sensor_token": c_token,
                "sensor_token": s_token,
                "filename": img["path"].replace("./data/nuscenes_eval/",""),
                "fileformat": "png",
                "is_key_frame": img["is_key"],
                "timestamp": img["timestamp"]
            })

    with open(os.path.join(out_dir, "sensor.json"), "w") as f:
        json.dump(sensor_json, f, indent=2)
    with open(os.path.join(out_dir, "calibrated_sensor.json"), "w") as f:
        json.dump(calib_json, f, indent=2)
    with open(os.path.join(out_dir, "sample.json"), "w") as f:
        json.dump(sample_json, f, indent=2)
    with open(os.path.join(out_dir, "sample_data.json"), "w") as f:
        json.dump(sample_data_json, f, indent=2)

    # category / attribute / visibility / sample_annotation / instanceを空で生成
    for name in ["category.json","attribute.json","visibility.json","sample_annotation.json","instance.json"]:
        with open(os.path.join(out_dir, name), "w") as f:
            json.dump([], f, indent=2)

    # map.json
    map_json = [{
        "token": str(uuid.uuid4()),
        "filename": "maps/eval_map.png",
        "log_tokens": [log_token],
        "layer_names": ["road_segment","lane","stop_line"]
    }]
    with open(os.path.join(out_dir, "map.json"), "w") as f:
        json.dump(map_json, f, indent=2)

    print("NuScenes 評価用 JSON + sweeps データ生成完了!")

    # ------------------------
    # 後片付け
    # ------------------------
    for cam in cam_actors:
        cam.destroy()
    prius.destroy()

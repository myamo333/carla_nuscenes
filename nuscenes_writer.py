import os
import uuid
import math
from datetime import datetime
import config
from utils import save_json, link_prev_next


def _yaw_deg_to_quat_wxyz(yaw_deg: float):
    """Return [w, x, y, z] quaternion for a yaw rotation about +Z.
    nuScenes stores quaternions as [w, x, y, z].
    """
    half = math.radians(yaw_deg) / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]

def write_nuscenes_jsons(base_dir,
                         sample_times,
                         key_img_for_idx,
                         key_radar_for_idx,
                         key_lidar_for_idx,
                         captured_images,
                         captured_radar,
                         captured_lidar):
    version = config.VERSION
    out_dir = os.path.join(base_dir, version)
    os.makedirs(out_dir, exist_ok=True)

    # tokens
    log_token = str(uuid.uuid4())
    scene_token = str(uuid.uuid4())
    ego_pose_token = str(uuid.uuid4())

    # sample.json
    sample_json = []
    prev = ""
    for st in sample_times:
        tok = str(uuid.uuid4())
        sample_json.append({
            "token": tok,
            "scene_token": scene_token,
            "prev": prev if prev else "",
            "next": "",
            "timestamp": int(st)
        })
        if prev:
            sample_json[-2]["next"] = tok
        prev = tok

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

    # ego_pose.json（固定）
    ego_pose_json = [{
        "token": ego_pose_token,
        "timestamp": sample_times[0],
        "rotation": [0,0,0,1],
        "translation": [0,0,0]
    }]

    # sensor.json / calibrated_sensor.json
    sensor_json, calib_json = [], []
    per_cam_tokens = {}

    # ---- カメラ: config.CAM_CONFIGS をそのまま出力 ----
    for cam_name in config.CAM_NAMES:
        if cam_name not in config.CAM_CONFIGS:
            raise KeyError(f"config.CAM_CONFIGS に {cam_name} の定義がありません。")

        s_token, c_token = str(uuid.uuid4()), str(uuid.uuid4())
        per_cam_tokens[cam_name] = (s_token, c_token)

        sensor_json.append({
            "token": s_token,
            "modality": "camera",
            "name": cam_name,
            "channel": cam_name
        })

        t = [float(v) for v in config.CAM_CONFIGS[cam_name]["translation"]]
        q = [float(v) for v in config.CAM_CONFIGS[cam_name]["rotation_wxyz"]]
        K = config.CAM_CONFIGS[cam_name]["intrinsic"]

        calib_json.append({
            "token": c_token,
            "sensor_token": s_token,
            "translation": t,                 # nuScenes そのまま
            "rotation": q,                    # nuScenes そのまま (w, x, y, z)
            "camera_intrinsic": K             # nuScenes そのまま
        })

    # ---- レーダ（従来どおり；必要なら RADAR_CALIB に移行可能）----
    per_radar_tokens = {}
    for rname in config.RADAR_NAMES:
        s_token, c_token = str(uuid.uuid4()), str(uuid.uuid4())
        per_radar_tokens[rname] = (s_token, c_token)
        sensor_json.append({
            "token": s_token, "modality": "radar",
            "name": rname, "channel": rname
        })

        # nuScenes 値をそのまま
        t = [float(v) for v in config.RADAR_CONFIGS[rname]["translation"]]
        q = [float(v) for v in config.RADAR_CONFIGS[rname]["rotation_wxyz"]]

        calib_json.append({
            "token": c_token, "sensor_token": s_token,
            "translation": t,
            "rotation": q,
            "camera_intrinsic": []
        })

    lidar_s_token, lidar_c_token = str(uuid.uuid4()), str(uuid.uuid4())
    sensor_json.append({
        "token": lidar_s_token, "modality": "lidar",
        "name": config.LIDAR_NAME, "channel": config.LIDAR_NAME
    })
    # LIDAR mount (sensors.attach_lidar): (0,0,2.5), yaw=0
    calib_json.append({
        "token": lidar_c_token, "sensor_token": lidar_s_token,
        "translation": [0.0, 0.0, 2.5], "rotation": _yaw_deg_to_quat_wxyz(0.0),
        "camera_intrinsic": []
    })

    # sample_data.json
    sample_data_json = []

    def add_sd(sample_idx, sensor_tokens, src_path, fileformat, width=0, height=0, base_dir=config.BASE_DIR):
        sample_token = sample_json[sample_idx]["token"]
        s_token, c_token = sensor_tokens
        rel = src_path.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
        rel = rel.replace(base_dir + os.sep, "")
        if fileformat == "pcd":
            # If already .pcd.bin keep it. If plain .bin, change to .pcd
            if rel.endswith(".pcd.bin"):
                pass
            elif rel.endswith(".bin"):
                rel = rel[:-4] + ".pcd"
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

    # keyframes: camera
    for cam_name in config.CAM_NAMES:
        s_token, c_token = per_cam_tokens[cam_name]
        for idx in range(len(sample_times)):
            src = key_img_for_idx.get(cam_name, {}).get(idx)
            if src: add_sd(idx, (s_token, c_token), src, "png", width=config.IMG_W, height=config.IMG_H)

    # keyframes: radar
    for rname in config.RADAR_NAMES:
        s_token, c_token = per_radar_tokens[rname]
        for idx in range(len(sample_times)):
            src = key_radar_for_idx.get(rname, {}).get(idx)
            if src: add_sd(idx, (s_token, c_token), src, "pcd")

    # keyframes: lidar
    for idx in range(len(sample_times)):
        src = key_lidar_for_idx.get(idx)
        if src:
            add_sd(idx, (lidar_s_token, lidar_c_token), src, "pcd")

    # sweeps: 非keyframe
    def nearest_sample_index(ts):
        return min(range(len(sample_times)), key=lambda i: abs(sample_times[i] - ts))

    # camera sweeps
    for cam_name, imgs in captured_images.items():
        s_token, c_token = per_cam_tokens[cam_name]
        for img in imgs:
            idx = nearest_sample_index(img["timestamp"])
            if key_img_for_idx.get(cam_name, {}).get(idx) == img["path"]:
                continue
            rel = img["path"].replace(config.BASE_DIR + os.sep, "")
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
                "width": config.IMG_W,
                "height": config.IMG_H
            })

    # radar sweeps（.pcdに差し替え）
    for rname, meas_list in captured_radar.items():
        s_token, c_token = per_radar_tokens[rname]
        for meas in meas_list:
            idx = nearest_sample_index(meas["timestamp"])
            if key_radar_for_idx.get(rname, {}).get(idx) == meas["path"]:
                continue
            rel = meas["path"].replace(config.BASE_DIR + os.sep, "")
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

    # lidar sweeps
    for meas in captured_lidar:
        idx = nearest_sample_index(meas["timestamp"])
        rel = meas["path"].replace(config.BASE_DIR + os.sep, "")
        sample_data_json.append({
            "token": str(uuid.uuid4()),
            "sample_token": sample_json[idx]["token"],
            "ego_pose_token": ego_pose_token,
            "calibrated_sensor_token": lidar_c_token,
            "sensor_token": lidar_s_token,
            "filename": rel,
            "fileformat": "pcd",
            "is_key_frame": False,
            "timestamp": meas["timestamp"],
            "width": 0,
            "height": 0
        })

    # prev/next
    link_prev_next(sample_data_json)

    # log.json / map.json
    log_json = [{
        "token": log_token,
        "location": "eval",
        "date_captured": datetime.now().strftime("%Y-%m-%d"),
        "logfile": "eval.log",
        "duration": float(config.DURATION_SEC)
    }]

    map_json = [{
        "token": str(uuid.uuid4()),
        "filename": "maps/eval_map.png",
        "log_tokens": [log_token],
        "layer_names": ["road_segment","lane","stop_line"]
    }]

    # 保存
    save_json(os.path.join(out_dir, "log.json"), log_json)
    save_json(os.path.join(out_dir, "scene.json"), scene_json)
    save_json(os.path.join(out_dir, "ego_pose.json"), ego_pose_json)
    save_json(os.path.join(out_dir, "sensor.json"), sensor_json)
    save_json(os.path.join(out_dir, "calibrated_sensor.json"), calib_json)
    save_json(os.path.join(out_dir, "sample.json"), sample_json)
    save_json(os.path.join(out_dir, "sample_data.json"), sample_data_json)
    for n in ["category.json","attribute.json","visibility.json","sample_annotation.json","instance.json"]:
        save_json(os.path.join(out_dir, n), [])
    save_json(os.path.join(out_dir, "map.json"), map_json)

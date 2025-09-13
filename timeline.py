import os
import shutil
import config

def compute_sample_times(captured_images, captured_lidar):
    # 何か1つでも画像があるチャンネルを基準に
    any_imgs = [v for v in captured_images.values() if len(v) > 0]
    if not any_imgs:
        raise RuntimeError("画像が記録されていません。")
    min_ts_candidates = [img["timestamp"] for imgs in captured_images.values() for img in imgs]
    if captured_lidar:
        min_ts_candidates.append(min(m["timestamp"] for m in captured_lidar))
    min_ts = min(min_ts_candidates)
    max_ts = min_ts + int(config.DURATION_SEC * 1e6)

    sample_times = []
    t = min_ts
    while t <= max_ts:
        sample_times.append(int(t))
        t += config.SAMPLE_INTERVAL_US
    return sample_times

def pick_keyframes_and_copy(captured_dict, sample_times, sweeps_dir, samples_dir):
    """
    captured_dict: {channel: [ {path, timestamp, ...}, ... ]}
    samples_dir にコピーし、各 sample index で最も近いフレームの元(sweeps)パスを記録
    戻り値: key_for_idx = {channel: {idx: src_sweeps_path}}
    """
    key_for_idx = {ch: {} for ch in captured_dict.keys()}

    def nearest_sample_index(ts):
        return min(range(len(sample_times)), key=lambda i: abs(sample_times[i] - ts))

    for ch, items in captured_dict.items():
        if not items:
            continue
        for idx in range(len(sample_times)):
            # そのサンプル時刻に最も近いもの
            closest = min(items, key=lambda it: abs(it["timestamp"] - sample_times[idx]))
            src = closest["path"]
            dst = src.replace(os.sep + "sweeps" + os.sep, os.sep + "samples" + os.sep)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            key_for_idx[ch][idx] = src
    return key_for_idx

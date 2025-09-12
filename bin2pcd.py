import os
import numpy as np
from tqdm import tqdm

def convert_bin_to_pcd(bin_path: str, pcd_path: str):
    """
    Carla radar .bin → PCD ASCII 変換
    Carla保存: depth, azimuth, altitude, velocity (float32 × 4)
    PCD出力: x y z velocity (float32 × 4)
    """
    # bin → numpy
    pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    depth, azimuth, altitude, velocity = pts.T

    # Carlaの極座標(depth, azimuth, altitude) → x,y,z（メートル）
    x = depth * np.cos(altitude) * np.cos(azimuth)
    y = depth * np.cos(altitude) * np.sin(azimuth)
    z = depth * np.sin(altitude)

    xyzv = np.stack([x, y, z, velocity], axis=1)

    # PCDヘッダ（ASCII）
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z velocity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {xyzv.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyzv.shape[0]}\n"
        "DATA ascii\n"
    )

    # 保存
    with open(pcd_path, 'w') as f:
        f.write(header)
        np.savetxt(f, xyzv, fmt="%.6f %.6f %.6f %.6f")


def batch_convert(bin_root: str, pcd_root: str):
    """
    dataroot 内の全 .bin ファイルを探索し、
    同じ相対パス構造で .pcd に変換する。
    """
    for root, _, files in os.walk(bin_root):
        for fname in tqdm(files):
            if fname.endswith(".bin"):
                bin_path = os.path.join(root, fname)
                rel = os.path.relpath(bin_path, bin_root)
                pcd_path = os.path.join(pcd_root, rel.replace(".bin", ".pcd"))
                os.makedirs(os.path.dirname(pcd_path), exist_ok=True)
                convert_bin_to_pcd(bin_path, pcd_path)


if __name__ == "__main__":
    # 例: ./data/nuscenes_eval/sweeps/RADAR_*
    bin_root = "./data/nuscenes_eval/sweeps"   # 変換元
    pcd_root = bin_root # 変換先
    os.makedirs(pcd_root, exist_ok=True)
    batch_convert(bin_root, pcd_root)
    print("✅ 変換完了: .bin → .pcd")

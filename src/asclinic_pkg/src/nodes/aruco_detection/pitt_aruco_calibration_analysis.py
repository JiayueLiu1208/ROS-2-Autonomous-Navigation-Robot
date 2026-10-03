#!/usr/bin/env python3

# ----------------------------------------------------------------------------
# ArUco 位姿估计精度验证 — 数据分析脚本
#
# 使用方法：
#   python3 aruco_calibration_analysis.py [csv文件路径]
#
# 默认读取: saved_camera_images/aruco_calibration_data.csv
# 生成图表保存到同目录下
# ----------------------------------------------------------------------------

import sys
import os
import csv
import numpy as np

DEFAULT_CSV_PATH = "saved_camera_images/aruco_calibration_data.csv"


def load_data(csv_path):
    """读取 CSV 数据"""
    data = {
        'sample_id': [], 'marker_id': [],
        'est_x': [], 'est_y': [], 'est_z': [], 'est_distance': [],
        'est_yaw_deg': [],
        'actual_distance': [], 'actual_angle': [],
        'distance_error': [], 'distance_error_pct': []
    }

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['sample_id'].append(int(row['sample_id']))
            data['marker_id'].append(int(row['marker_id']))
            data['est_x'].append(float(row['est_x']))
            data['est_y'].append(float(row['est_y']))
            data['est_z'].append(float(row['est_z']))
            data['est_distance'].append(float(row['est_distance']))
            data['est_yaw_deg'].append(float(row['est_yaw_deg']))
            data['actual_distance'].append(float(row['actual_distance']))
            data['actual_angle'].append(float(row['actual_angle']) if row['actual_angle'] else float('nan'))
            data['distance_error'].append(float(row['distance_error']))
            data['distance_error_pct'].append(float(row['distance_error_pct']))

    # 转为 numpy 数组
    for key in data:
        data[key] = np.array(data[key])

    return data


def print_statistics(data):
    """打印统计结果"""
    n = len(data['sample_id'])
    err = data['distance_error']
    err_pct = data['distance_error_pct']
    abs_err = np.abs(err)

    print("=" * 60)
    print(f"ArUco 距离估计精度分析报告")
    print(f"数据点数: {n}")
    print("=" * 60)

    print(f"\n距离误差统计（估计 - 实际）：")
    print(f"  均值 (Mean Error):       {np.mean(err):+.4f} m")
    print(f"  标准差 (Std Dev):        {np.std(err):.4f} m")
    print(f"  绝对误差均值 (MAE):      {np.mean(abs_err):.4f} m")
    print(f"  均方根误差 (RMSE):       {np.sqrt(np.mean(err**2)):.4f} m")
    print(f"  最大误差:                {np.max(abs_err):.4f} m")
    print(f"  最小误差:                {np.min(abs_err):.4f} m")
    print(f"  百分比误差均值:          {np.mean(err_pct):+.2f} %")
    print(f"  百分比误差标准差:        {np.std(err_pct):.2f} %")

    # 按距离段统计
    print(f"\n按距离段统计：")
    print(f"  {'距离范围':>15s}  {'样本数':>6s}  {'MAE(m)':>8s}  {'RMSE(m)':>8s}  {'Mean%':>8s}")
    print(f"  {'-'*15}  {'-'*6}  {'-'*8}  {'-'*8}  {'-'*8}")

    bins = [(0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0), (3.0, 5.0), (5.0, 100.0)]
    for lo, hi in bins:
        mask = (data['actual_distance'] >= lo) & (data['actual_distance'] < hi)
        count = np.sum(mask)
        if count > 0:
            seg_err = err[mask]
            seg_abs = abs_err[mask]
            seg_pct = err_pct[mask]
            label = f"{lo:.1f}-{hi:.1f}m"
            print(f"  {label:>15s}  {count:>6d}  {np.mean(seg_abs):>8.4f}  {np.sqrt(np.mean(seg_err**2)):>8.4f}  {np.mean(seg_pct):>+8.2f}")

    # 检查角度数据
    angle_mask = ~np.isnan(data['actual_angle'])
    if np.any(angle_mask):
        angle_err = data['est_yaw_deg'][angle_mask] - data['actual_angle'][angle_mask]
        print(f"\n角度误差统计（估计 - 实际）：")
        print(f"  样本数:                  {np.sum(angle_mask)}")
        print(f"  均值:                    {np.mean(angle_err):+.2f}°")
        print(f"  标准差:                  {np.std(angle_err):.2f}°")
        print(f"  绝对误差均值 (MAE):      {np.mean(np.abs(angle_err)):.2f}°")


def plot_results(data, save_dir):
    """生成分析图表"""
    try:
        import matplotlib
        matplotlib.use('Agg')  # 无显示器环境
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n⚠ matplotlib 未安装，跳过图表生成")
        print("  安装：pip3 install matplotlib")
        return

    actual = data['actual_distance']
    estimated = data['est_distance']
    err = data['distance_error']

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('ArUco Distance Estimation Accuracy Analysis', fontsize=14, fontweight='bold')

    # 图1: 实际距离 vs 估计距离 散点图
    ax1 = axes[0, 0]
    ax1.scatter(actual, estimated, alpha=0.6, s=20, color='steelblue')
    lim_max = max(np.max(actual), np.max(estimated)) * 1.1
    ax1.plot([0, lim_max], [0, lim_max], 'r--', linewidth=1, label='Perfect estimation')
    ax1.set_xlabel('Actual Distance (m)')
    ax1.set_ylabel('Estimated Distance (m)')
    ax1.set_title('Estimated vs Actual Distance')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # 图2: 距离误差 vs 实际距离
    ax2 = axes[0, 1]
    ax2.scatter(actual, err, alpha=0.6, s=20, color='coral')
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=1)
    ax2.axhline(y=np.mean(err), color='red', linestyle='-', linewidth=1, label=f'Mean error = {np.mean(err):+.3f}m')
    ax2.fill_between(
        [np.min(actual), np.max(actual)],
        np.mean(err) - np.std(err), np.mean(err) + np.std(err),
        alpha=0.15, color='red', label=f'±1σ = {np.std(err):.3f}m'
    )
    ax2.set_xlabel('Actual Distance (m)')
    ax2.set_ylabel('Distance Error (m)')
    ax2.set_title('Distance Error vs Actual Distance')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 图3: 误差分布直方图
    ax3 = axes[1, 0]
    ax3.hist(err, bins=30, color='steelblue', edgecolor='white', alpha=0.8)
    ax3.axvline(x=np.mean(err), color='red', linestyle='--', linewidth=1.5, label=f'Mean = {np.mean(err):+.3f}m')
    ax3.set_xlabel('Distance Error (m)')
    ax3.set_ylabel('Count')
    ax3.set_title('Error Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 图4: 百分比误差 vs 实际距离
    ax4 = axes[1, 1]
    ax4.scatter(actual, data['distance_error_pct'], alpha=0.6, s=20, color='seagreen')
    ax4.axhline(y=0, color='gray', linestyle='--', linewidth=1)
    ax4.set_xlabel('Actual Distance (m)')
    ax4.set_ylabel('Error (%)')
    ax4.set_title('Percentage Error vs Actual Distance')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(save_dir, 'aruco_calibration_analysis.png')
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"\n图表已保存到: {plot_path}")

    # 角度分析图（如果有角度数据）
    angle_mask = ~np.isnan(data['actual_angle'])
    if np.sum(angle_mask) >= 5:
        fig2, (ax_a1, ax_a2) = plt.subplots(1, 2, figsize=(12, 5))
        fig2.suptitle('ArUco Angle Estimation Analysis', fontsize=14, fontweight='bold')

        est_angle = data['est_yaw_deg'][angle_mask]
        act_angle = data['actual_angle'][angle_mask]
        angle_err = est_angle - act_angle

        ax_a1.scatter(act_angle, est_angle, alpha=0.6, s=20, color='steelblue')
        alim = max(np.max(np.abs(act_angle)), np.max(np.abs(est_angle))) * 1.1
        ax_a1.plot([-alim, alim], [-alim, alim], 'r--', linewidth=1)
        ax_a1.set_xlabel('Actual Angle (°)')
        ax_a1.set_ylabel('Estimated Angle (°)')
        ax_a1.set_title('Estimated vs Actual Angle')
        ax_a1.grid(True, alpha=0.3)

        ax_a2.hist(angle_err, bins=20, color='coral', edgecolor='white', alpha=0.8)
        ax_a2.axvline(x=np.mean(angle_err), color='red', linestyle='--', label=f'Mean = {np.mean(angle_err):+.1f}°')
        ax_a2.set_xlabel('Angle Error (°)')
        ax_a2.set_ylabel('Count')
        ax_a2.set_title('Angle Error Distribution')
        ax_a2.legend()
        ax_a2.grid(True, alpha=0.3)

        plt.tight_layout()
        angle_plot_path = os.path.join(save_dir, 'aruco_angle_analysis.png')
        plt.savefig(angle_plot_path, dpi=150)
        plt.close()
        print(f"角度图表已保存到: {angle_plot_path}")


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV_PATH

    if not os.path.isfile(csv_path):
        print(f"文件不存在: {csv_path}")
        print("请先运行 aruco_calibration_collector.py 采集数据")
        sys.exit(1)

    data = load_data(csv_path)

    if len(data['sample_id']) == 0:
        print("CSV 文件中没有数据")
        sys.exit(1)

    print_statistics(data)
    plot_results(data, os.path.dirname(csv_path))


if __name__ == '__main__':
    main()

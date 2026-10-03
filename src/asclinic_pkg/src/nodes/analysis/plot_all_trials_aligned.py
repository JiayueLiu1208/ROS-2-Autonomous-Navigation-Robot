#!/usr/bin/env python3

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def load_trial_files(experiment_dir):
    pattern = os.path.join(experiment_dir, 'trial_*_trace_*.csv')
    files = sorted(glob.glob(pattern))
    if len(files) == 0:
        raise FileNotFoundError(f'No trial trace files found in: {experiment_dir}')
    return files


def read_trial_csv(file_path):
    """
    Read one trace CSV robustly.
    Supports comma/semicolon automatically.
    """
    df = pd.read_csv(file_path, sep=None, engine='python', encoding='utf-8-sig')

    required_cols = [
        'time_sec',
        'x_odom',
        'y_odom',
        'yaw_odom',
        'pose_cov_xx',
        'pose_cov_yy',
        'pose_cov_yawyaw'
    ]

    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f'{file_path} is missing columns: {missing}')

    df = df.drop_duplicates(subset='time_sec').reset_index(drop=True)
    df = df.sort_values(by='time_sec').reset_index(drop=True)

    # 起点对齐
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
    df['x_odom'] = df['x_odom'] - df['x_odom'].iloc[0]
    df['y_odom'] = df['y_odom'] - df['y_odom'].iloc[0]
    df['yaw_odom'] = wrap_angle(df['yaw_odom'] - df['yaw_odom'].iloc[0])

    return df


def interpolate_trials(files, column_name, n_points=400):
    """
    Interpolate all trials to a common time axis,
    then compute the mean curve.
    """
    all_trials = []
    end_times = []

    for f in files:
        df = read_trial_csv(f)

        t = df['time_sec'].to_numpy(dtype=float)
        v = df[column_name].to_numpy(dtype=float)

        if len(t) < 2:
            continue

        all_trials.append((t, v, os.path.basename(f)))
        end_times.append(t[-1])

    if len(all_trials) == 0:
        raise ValueError(f'No valid trial data found for column: {column_name}')

    # 统一时间轴：取所有实验里最短的结束时间
    t_common_end = min(end_times)
    t_common = np.linspace(0.0, t_common_end, n_points)

    trial_matrix = []
    trial_names = []

    for t, v, name in all_trials:
        v_interp = np.interp(t_common, t, v)
        trial_matrix.append(v_interp)
        trial_names.append(name)

    trial_matrix = np.array(trial_matrix)
    mean_curve = np.mean(trial_matrix, axis=0)

    return t_common, trial_matrix, mean_curve, trial_names


def plot_variable(experiment_dir, files, column_name, ylabel, title, output_name):
    t_common, trial_matrix, mean_curve, _ = interpolate_trials(files, column_name)

    plt.figure(figsize=(9, 5))

    for i in range(trial_matrix.shape[0]):
        plt.plot(
            t_common,
            trial_matrix[i, :],
            linewidth=1.2,
            alpha=0.85,
            label=f'Trial {i+1}'
        )

    plt.plot(
        t_common,
        mean_curve,
        'k-',
        linewidth=2.8,
        label='Mean'
    )

    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend(loc='best')
    plt.tight_layout()

    output_path = os.path.join(experiment_dir, output_name)
    plt.savefig(output_path, dpi=200)
    plt.close()

    print(f'Saved: {output_path}')


def plot_trajectory(experiment_dir, files, output_name='trajectory_5trials_mean.png'):
    # 对 x 和 y 用同一个统一时间轴插值
    t_x, X, X_mean, _ = interpolate_trials(files, 'x_odom')
    t_y, Y, Y_mean, _ = interpolate_trials(files, 'y_odom')

    plt.figure(figsize=(7, 7))

    # 单次实验轨迹
    for i in range(min(X.shape[0], Y.shape[0])):
        plt.plot(
            X[i, :],
            Y[i, :],
            linewidth=1.2,
            alpha=0.85,
            label=f'Trial {i+1}'
        )

    # 平均轨迹
    plt.plot(
        X_mean,
        Y_mean,
        'k-',
        linewidth=2.8,
        label='Mean'
    )

    plt.plot(0.0, 0.0, 'go', markersize=8, label='Aligned Start')

    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('x_odom (aligned) [m]')
    plt.ylabel('y_odom (aligned) [m]')
    plt.title('Trajectory: 5 Trials + Mean (Aligned Start)')
    plt.legend(loc='best')
    plt.tight_layout()

    output_path = os.path.join(experiment_dir, output_name)
    plt.savefig(output_path, dpi=200)
    plt.close()

    print(f'Saved: {output_path}')


def plot_all(experiment_dir):
    files = load_trial_files(experiment_dir)

    print('Found trace files:')
    for f in files:
        print('  ', os.path.basename(f))

    # 1. x_odom
    plot_variable(
        experiment_dir,
        files,
        column_name='x_odom',
        ylabel='x_odom (aligned) [m]',
        title='x_odom(t): 5 Trials + Mean',
        output_name='x_odom_5trials_mean_aligned.png'
    )

    # 2. y_odom
    plot_variable(
        experiment_dir,
        files,
        column_name='y_odom',
        ylabel='y_odom (aligned) [m]',
        title='y_odom(t): 5 Trials + Mean',
        output_name='y_odom_5trials_mean_aligned.png'
    )

    # 3. yaw_odom
    plot_variable(
        experiment_dir,
        files,
        column_name='yaw_odom',
        ylabel='yaw_odom (aligned) [rad]',
        title='yaw_odom(t): 5 Trials + Mean',
        output_name='yaw_odom_5trials_mean_aligned.png'
    )

    # 4. pose_cov_xx
    plot_variable(
        experiment_dir,
        files,
        column_name='pose_cov_xx',
        ylabel='pose_cov_xx',
        title='pose_cov_xx(t): 5 Trials + Mean',
        output_name='pose_cov_xx_5trials_mean.png'
    )

    # 5. pose_cov_yy
    plot_variable(
        experiment_dir,
        files,
        column_name='pose_cov_yy',
        ylabel='pose_cov_yy',
        title='pose_cov_yy(t): 5 Trials + Mean',
        output_name='pose_cov_yy_5trials_mean.png'
    )

    # 6. pose_cov_yawyaw
    plot_variable(
        experiment_dir,
        files,
        column_name='pose_cov_yawyaw',
        ylabel='pose_cov_yawyaw',
        title='pose_cov_yawyaw(t): 5 Trials + Mean',
        output_name='pose_cov_yawyaw_5trials_mean.png'
    )

    # 7. trajectory
    plot_trajectory(
        experiment_dir,
        files,
        output_name='trajectory_5trials_mean_aligned.png'
    )

    print('\nAll figures saved to:')
    print(experiment_dir)


if __name__ == '__main__':
    experiment_dir = os.path.expanduser('~/asclinic-ros2/ros2_ws/results/straight_0p5m')
    plot_all(experiment_dir)
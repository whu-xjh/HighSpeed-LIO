#!/usr/bin/env python3
"""
6DOF Trajectory Visualization Tool

This module provides functions to visualize 6DOF trajectory data from text files.
Each line in the input file should contain: timestamp x y z qx qy qz qw

Author: Claude Code Assistant
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os


def load_trajectory(file_path):
    """
    Load 6DOF trajectory data from a text file.

    Args:
        file_path (str): Path to the trajectory file

    Returns:
        tuple: (timestamps, positions, quaternions)
            timestamps: numpy array of timestamps
            positions: numpy array of shape (N, 3) for x, y, z coordinates
            quaternions: numpy array of shape (N, 4) for quaternion components
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Trajectory file not found: {file_path}")

    try:
        data = np.loadtxt(file_path)
        if data.shape[1] != 8:
            raise ValueError(f"Expected 8 columns per line, got {data.shape[1]}")

        timestamps = data[:, 0]
        positions = data[:, 1:4]  # x, y, z
        quaternions = data[:, 4:8]  # qx, qy, qz, qw

        return timestamps, positions, quaternions

    except Exception as e:
        raise RuntimeError(f"Error loading trajectory file: {e}")


def plot_trajectories(file1_path, file2_path, output_path=None):
    """
    Plot and compare two 6DOF trajectories.

    Args:
        file1_path (str): Path to the first trajectory file
        file2_path (str): Path to the second trajectory file
        output_path (str, optional): Path to save the plot image

    Returns:
        matplotlib.figure.Figure: The generated figure object
    """
    # Load trajectories
    print(f"Loading trajectory from: {file1_path}")
    ts1, pos1, quat1 = load_trajectory(file1_path)
    print(f"  Loaded {len(pos1)} poses")

    print(f"Loading trajectory from: {file2_path}")
    ts2, pos2, quat2 = load_trajectory(file2_path)
    print(f"  Loaded {len(pos2)} poses")

    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))

    # 3D trajectory plot
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(pos1[:, 0], pos1[:, 1], pos1[:, 2], 'b-', label='Trajectory 1', linewidth=2)
    ax1.plot(pos2[:, 0], pos2[:, 1], pos2[:, 2], 'r-', label='Trajectory 2', linewidth=2)
    ax1.scatter(pos1[0, 0], pos1[0, 1], pos1[0, 2], c='blue', s=50, marker='o')
    ax1.scatter(pos2[0, 0], pos2[0, 1], pos2[0, 2], c='red', s=50, marker='o')
    ax1.scatter(pos1[-1, 0], pos1[-1, 1], pos1[-1, 2], c='blue', s=50, marker='s')
    ax1.scatter(pos2[-1, 0], pos2[-1, 1], pos2[-1, 2], c='red', s=50, marker='s')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory Comparison')
    ax1.legend()
    ax1.grid(True)

    # XY projection (top view)
    ax2 = fig.add_subplot(222)
    ax2.plot(pos1[:, 0], pos1[:, 1], 'b-', label='Trajectory 1', linewidth=2)
    ax2.plot(pos2[:, 0], pos2[:, 1], 'r-', label='Trajectory 2', linewidth=2)
    ax2.scatter(pos1[0, 0], pos1[0, 1], c='blue', s=50, marker='o', label='Start 1')
    ax2.scatter(pos2[0, 0], pos2[0, 1], c='red', s=50, marker='o', label='Start 2')
    ax2.scatter(pos1[-1, 0], pos1[-1, 1], c='blue', s=50, marker='s', label='End 1')
    ax2.scatter(pos2[-1, 0], pos2[-1, 1], c='red', s=50, marker='s', label='End 2')

    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('XY Projection (Top View)')
    ax2.legend()
    ax2.grid(True)
    ax2.axis('equal')

    # XZ projection (side view)
    ax3 = fig.add_subplot(223)
    ax3.plot(pos1[:, 0], pos1[:, 2], 'b-', label='Trajectory 1', linewidth=2)
    ax3.plot(pos2[:, 0], pos2[:, 2], 'r-', label='Trajectory 2', linewidth=2)
    ax3.scatter(pos1[0, 0], pos1[0, 2], c='blue', s=50, marker='o')
    ax3.scatter(pos2[0, 0], pos2[0, 2], c='red', s=50, marker='o')
    ax3.scatter(pos1[-1, 0], pos1[-1, 2], c='blue', s=50, marker='s')
    ax3.scatter(pos2[-1, 0], pos2[-1, 2], c='red', s=50, marker='s')

    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('XZ Projection (Side View)')
    ax3.legend()
    ax3.grid(True)

    # Position error over time (if timestamps are comparable)
    ax4 = fig.add_subplot(224)
    if len(ts1) > 0 and len(ts2) > 0:
        # Simple interpolation if timestamps differ
        min_length = min(len(pos1), len(pos2))
        if len(ts1) != len(ts2):
            print("Warning: Trajectories have different numbers of poses. Comparing first {} poses.".format(min_length))

        # Calculate Euclidean distance between corresponding poses
        pos1_subset = pos1[:min_length]
        pos2_subset = pos2[:min_length]

        distances = np.sqrt(np.sum((pos1_subset - pos2_subset) ** 2, axis=1))

        ax4.plot(distances, 'g-', linewidth=2)
        ax4.set_xlabel('Pose Index')
        ax4.set_ylabel('Distance (m)')
        ax4.set_title('Position Distance Between Trajectories')
        ax4.grid(True)

        # Add statistics
        mean_error = np.mean(distances)
        max_error = np.max(distances)
        ax4.text(0.05, 0.95, f'Mean Error: {mean_error:.3f} m\nMax Error: {max_error:.3f} m',
                transform=ax4.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.suptitle('6DOF Trajectory Comparison', fontsize=16)
    plt.tight_layout()

    # Save plot if output path is provided
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")

    # Always show the plot
    plt.show()

    return fig


if __name__ == "__main__":
    # Example usage - modify these file paths as needed
    file1 = "/home/xjh/Doc/highspeed-lio/catkin_ws/src/HighSpeed-LIO-main/Log/result/exp04_construction_upper_level.txt"  # Replace with your first trajectory file
    file2 = "/media/xjh/Extreme SSD/data/rosbag/Hilti SLAM Challenge 2022/6DOF-Exp04"  # Replace with your second trajectory file

    try:
        fig = plot_trajectories(file1, file2, output_path="trajectory_comparison.png")
        print("Trajectory visualization completed successfully!")
    except Exception as e:
        print(f"Error: {e}")
        print("Please update the file paths in the script with your actual trajectory files.")
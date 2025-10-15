#!/usr/bin/env python3
"""
Plot CPU and memory resource usage from play_launch logs.

Reads CSV files from the metrics directory and generates:
1. CPU usage timeline (per-node with indexed labels)
2. Memory (RSS) usage timeline (per-node with indexed labels)
3. Legend image mapping index to node name and color
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple
import sys

try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    import numpy as np
except ImportError:
    print("Error: matplotlib and numpy are required. Install with: pip install matplotlib numpy")
    sys.exit(1)


def find_latest_log_dir(base_log_dir: Path) -> Path:
    """Find the most recent timestamped log directory."""
    log_dirs = [d for d in base_log_dir.iterdir() if d.is_dir() and d.name[0].isdigit()]
    if not log_dirs:
        raise FileNotFoundError(f"No log directories found in {base_log_dir}")

    # Sort by directory name (timestamp format: YYYY-MM-DD_HH-MM-SS)
    latest = sorted(log_dirs, key=lambda d: d.name)[-1]
    return latest


def parse_csv_file(csv_path: Path) -> Tuple[List[float], List[float], List[float]]:
    """
    Parse a CSV file and extract timestamps, CPU%, and RSS bytes.

    Returns:
        Tuple of (relative_times, cpu_percents, rss_mb)
        - relative_times: seconds from first timestamp
        - cpu_percents: CPU usage percentages
        - rss_mb: Memory usage in MB
    """
    timestamps = []
    cpu_percents = []
    rss_bytes = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Parse ISO timestamp
            ts = datetime.fromisoformat(row['timestamp'].replace('Z', '+00:00'))
            timestamps.append(ts)
            cpu_percents.append(float(row['cpu_percent']))
            rss_bytes.append(int(row['rss_bytes']))

    if not timestamps:
        return [], [], []

    # Convert to relative time in seconds
    start_time = timestamps[0]
    relative_times = [(ts - start_time).total_seconds() for ts in timestamps]

    # Convert RSS to MB
    rss_mb = [rss / (1024 * 1024) for rss in rss_bytes]

    return relative_times, cpu_percents, rss_mb


def load_all_metrics(metrics_dir: Path) -> Dict[str, Tuple[List[float], List[float], List[float]]]:
    """
    Load all CSV files from the metrics directory.

    Returns:
        Dict mapping node name to (times, cpu%, rss_mb)
    """
    metrics = {}
    csv_files = list(metrics_dir.glob("*.csv"))

    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {metrics_dir}")

    print(f"Loading {len(csv_files)} CSV files...")

    for csv_path in csv_files:
        # Extract node name from filename (remove "NODE '" prefix and "'.csv" suffix)
        node_name = csv_path.stem
        if node_name.startswith("NODE '"):
            node_name = node_name[6:]  # Remove "NODE '"
        if node_name.endswith("'"):
            node_name = node_name[:-1]  # Remove trailing "'"

        times, cpu, rss = parse_csv_file(csv_path)
        if times:  # Only add if we got data
            metrics[node_name] = (times, cpu, rss)

    print(f"Loaded metrics for {len(metrics)} nodes")
    return metrics


def calculate_statistics(metrics: Dict[str, Tuple[List[float], List[float], List[float]]], node_colors: Dict[int, Tuple[str, any]]) -> str:
    """
    Calculate statistics for all nodes.

    Args:
        metrics: Node metrics data
        node_colors: Node index to name mapping

    Returns:
        Formatted statistics string
    """
    stats = []

    # Calculate max and avg for each node
    for idx, (node_name, (times, cpu_vals, rss_vals)) in enumerate(sorted(metrics.items())):
        if not cpu_vals or not rss_vals:
            continue

        max_cpu = max(cpu_vals)
        avg_cpu = sum(cpu_vals) / len(cpu_vals)
        max_mem = max(rss_vals)
        avg_mem = sum(rss_vals) / len(rss_vals)

        stats.append({
            'index': idx,
            'name': node_name,
            'max_cpu': max_cpu,
            'avg_cpu': avg_cpu,
            'max_mem': max_mem,
            'avg_mem': avg_mem
        })

    # Build statistics report
    report = []
    report.append("=" * 80)
    report.append("RESOURCE USAGE STATISTICS")
    report.append("=" * 80)
    report.append("")

    # Top 10 nodes by max CPU
    report.append("Top 10 Nodes by Maximum CPU Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Max CPU %':<12} {'Node Name'}")
    report.append("-" * 80)
    top_max_cpu = sorted(stats, key=lambda x: x['max_cpu'], reverse=True)[:10]
    for rank, node_stat in enumerate(top_max_cpu, 1):
        report.append(f"{rank:<6} [{node_stat['index']}]{'':<6} {node_stat['max_cpu']:>10.2f}%  {node_stat['name']}")
    report.append("")

    # Top 10 nodes by avg CPU
    report.append("Top 10 Nodes by Average CPU Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Avg CPU %':<12} {'Node Name'}")
    report.append("-" * 80)
    top_avg_cpu = sorted(stats, key=lambda x: x['avg_cpu'], reverse=True)[:10]
    for rank, node_stat in enumerate(top_avg_cpu, 1):
        report.append(f"{rank:<6} [{node_stat['index']}]{'':<6} {node_stat['avg_cpu']:>10.2f}%  {node_stat['name']}")
    report.append("")

    # Top 10 nodes by max memory
    report.append("Top 10 Nodes by Maximum Memory Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Max Mem (MB)':<14} {'Node Name'}")
    report.append("-" * 80)
    top_max_mem = sorted(stats, key=lambda x: x['max_mem'], reverse=True)[:10]
    for rank, node_stat in enumerate(top_max_mem, 1):
        report.append(f"{rank:<6} [{node_stat['index']}]{'':<6} {node_stat['max_mem']:>12.2f}  {node_stat['name']}")
    report.append("")

    # Top 10 nodes by avg memory
    report.append("Top 10 Nodes by Average Memory Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Avg Mem (MB)':<14} {'Node Name'}")
    report.append("-" * 80)
    top_avg_mem = sorted(stats, key=lambda x: x['avg_mem'], reverse=True)[:10]
    for rank, node_stat in enumerate(top_avg_mem, 1):
        report.append(f"{rank:<6} [{node_stat['index']}]{'':<6} {node_stat['avg_mem']:>12.2f}  {node_stat['name']}")
    report.append("")

    report.append("=" * 80)

    return "\n".join(report)


def generate_container_listing(log_dir: Path, node_colors: Dict[int, Tuple[str, any]]) -> str:
    """
    Generate a listing of containers and their composable nodes.

    Args:
        log_dir: Path to the log directory
        node_colors: Node index to name mapping

    Returns:
        Formatted container listing string
    """
    load_node_dir = log_dir / "load_node"

    if not load_node_dir.exists():
        return "No load_node directory found - no composable nodes were loaded."

    # Build a mapping of node names to indices
    node_name_to_idx = {name: idx for idx, (name, _) in node_colors.items()}

    # Scan containers
    containers = {}
    for container_path in sorted(load_node_dir.iterdir()):
        if not container_path.is_dir():
            continue

        container_name = container_path.name
        composable_nodes = []

        # Scan nodes within this container
        for node_dir in sorted(container_path.iterdir()):
            if node_dir.is_dir():
                # Reconstruct the full node name as it appears in metrics
                # The metrics use the pattern: package_executable-number
                # We need to find matching node names
                node_simple_name = node_dir.name

                # Find nodes that match this container and composable node
                # Look for nodes in metrics that contain this name
                matching_nodes = []
                for full_node_name, idx in node_name_to_idx.items():
                    # Match based on the simple name being part of the full name
                    if node_simple_name in full_node_name:
                        matching_nodes.append((idx, full_node_name))

                if matching_nodes:
                    # Take the first match (usually there's only one)
                    for idx, full_name in matching_nodes:
                        composable_nodes.append((idx, node_simple_name, full_name))
                else:
                    # Node not found in metrics (might not have monitoring)
                    composable_nodes.append((None, node_simple_name, None))

        containers[container_name] = composable_nodes

    # Build report
    report = []
    report.append("=" * 100)
    report.append("CONTAINER AND COMPOSABLE NODE LISTING")
    report.append("=" * 100)
    report.append("")

    if not containers:
        report.append("No containers or composable nodes found.")
    else:
        for container_name in sorted(containers.keys()):
            composable_nodes = containers[container_name]

            report.append(f"Container: {container_name}")
            report.append("-" * 100)
            report.append(f"  {'Index':<10} {'Composable Node':<40} {'Full Node Name (in metrics)'}")
            report.append("-" * 100)

            for idx, simple_name, full_name in composable_nodes:
                if idx is not None:
                    report.append(f"  [{idx}]{'':<6} {simple_name:<40} {full_name}")
                else:
                    report.append(f"  {'N/A':<10} {simple_name:<40} (not monitored)")

            report.append("")

    report.append("=" * 100)

    return "\n".join(report)


def create_legend_image(node_colors: Dict[int, Tuple[str, any]], output_path: Path):
    """
    Create a separate legend image mapping index to node name and color.

    Args:
        node_colors: Dict mapping index to (node_name, color)
        output_path: Path to save the legend image
    """
    fig, ax = plt.subplots(figsize=(10, max(8, len(node_colors) * 0.3)))
    ax.axis('off')

    # Create text with color boxes
    y_pos = 0.98
    for idx in sorted(node_colors.keys()):
        node_name, color = node_colors[idx]
        # Draw colored box
        ax.add_patch(plt.Rectangle((0.02, y_pos - 0.015), 0.04, 0.02,
                                   facecolor=color, edgecolor='black', linewidth=0.5,
                                   transform=ax.transAxes))
        # Draw text
        ax.text(0.08, y_pos, f"[{idx}] {node_name}",
               fontsize=9, verticalalignment='center', transform=ax.transAxes)
        y_pos -= 0.025

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    plt.title('Node Index Legend', fontsize=14, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Legend saved to: {output_path}")
    plt.close()


def plot_cpu_usage(metrics: Dict[str, Tuple[List[float], List[float], List[float]]], output_path: Path) -> Dict[int, Tuple[str, any]]:
    """
    Plot CPU usage timeline for all nodes.

    Returns:
        Dict mapping index to (node_name, color) for legend generation
    """
    plt.figure(figsize=(14, 8))

    node_colors = {}
    max_cpu = 0.0

    # Plot individual nodes with thin lines and indexed labels
    for idx, (node_name, (times, cpu_vals, _)) in enumerate(sorted(metrics.items())):
        line, = plt.plot(times, cpu_vals, linewidth=0.8, alpha=0.7, label=f"[{idx}]")
        node_colors[idx] = (node_name, line.get_color())
        max_cpu = max(max_cpu, max(cpu_vals, default=0))

    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('CPU Usage (%)', fontsize=12)
    plt.title('CPU Usage Over Time', fontsize=14, fontweight='bold')
    plt.ylim(bottom=0, top=max(max_cpu * 1.1, 10))  # 10% headroom or min 10%
    plt.grid(True, alpha=0.3)

    # Legend with index numbers only
    plt.legend(loc='best', fontsize=8, ncol=4)
    plt.tight_layout()

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"CPU plot saved to: {output_path}")
    plt.close()

    return node_colors


def plot_memory_usage(metrics: Dict[str, Tuple[List[float], List[float], List[float]]], output_path: Path, node_colors: Dict[int, Tuple[str, any]]):
    """
    Plot memory (RSS) usage timeline for all nodes.

    Args:
        metrics: Node metrics data
        output_path: Output path for the plot
        node_colors: Pre-computed node colors from CPU plot to maintain consistency
    """
    plt.figure(figsize=(14, 8))

    max_mem = 0.0

    # Plot individual nodes with thin lines and indexed labels, using same colors as CPU plot
    for idx, (node_name, (times, _, rss_vals)) in enumerate(sorted(metrics.items())):
        color = node_colors[idx][1] if idx in node_colors else None
        plt.plot(times, rss_vals, linewidth=0.8, alpha=0.7, label=f"[{idx}]", color=color)
        max_mem = max(max_mem, max(rss_vals, default=0))

    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Memory Usage (MB)', fontsize=12)
    plt.title('Memory (RSS) Usage Over Time', fontsize=14, fontweight='bold')
    plt.ylim(bottom=0, top=max(max_mem * 1.1, 100))  # 10% headroom or min 100MB
    plt.grid(True, alpha=0.3)

    # Legend with index numbers only
    plt.legend(loc='best', fontsize=8, ncol=4)
    plt.tight_layout()

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Memory plot saved to: {output_path}")
    plt.close()


def plot_cpu_distribution(metrics: Dict[str, Tuple[List[float], List[float], List[float]]], output_path: Path, node_colors: Dict[int, Tuple[str, any]]):
    """
    Plot CPU usage distribution for all nodes showing min, Q1, median, Q3, and max.

    Args:
        metrics: Node metrics data
        output_path: Output path for the plot
        node_colors: Node index to name mapping for colors
    """
    fig, ax = plt.subplots(figsize=(16, 8))

    # Prepare data for each node
    indices = []
    cpu_stats = []
    colors = []

    for idx, (node_name, (times, cpu_vals, _)) in enumerate(sorted(metrics.items())):
        if not cpu_vals:
            continue

        indices.append(idx)
        cpu_array = np.array(cpu_vals)

        # Calculate statistics
        stats = {
            'min': np.min(cpu_array),
            'q1': np.percentile(cpu_array, 25),
            'median': np.percentile(cpu_array, 50),
            'q3': np.percentile(cpu_array, 75),
            'max': np.max(cpu_array)
        }
        cpu_stats.append(stats)
        colors.append(node_colors[idx][1] if idx in node_colors else 'blue')

    # Plot error bars showing range
    for i, (idx, stats, color) in enumerate(zip(indices, cpu_stats, colors)):
        # Draw vertical line from min to max
        ax.plot([idx, idx], [stats['min'], stats['max']],
                color=color, alpha=0.4, linewidth=1.5, zorder=1)

        # Draw box from Q1 to Q3
        box_height = stats['q3'] - stats['q1']
        rect = plt.Rectangle((idx - 0.3, stats['q1']), 0.6, box_height,
                             facecolor=color, edgecolor='black', alpha=0.6, linewidth=1, zorder=2)
        ax.add_patch(rect)

        # Draw median line
        ax.plot([idx - 0.3, idx + 0.3], [stats['median'], stats['median']],
                color='black', linewidth=2, zorder=3)

        # Draw min and max markers
        ax.plot(idx, stats['min'], 'o', color=color, markersize=4, zorder=2)
        ax.plot(idx, stats['max'], 'o', color=color, markersize=4, zorder=2)

    ax.set_xlabel('Node Index', fontsize=12)
    ax.set_ylabel('CPU Usage (%)', fontsize=12)
    ax.set_title('CPU Usage Distribution by Node (Min, Q1, Median, Q3, Max)',
                 fontsize=14, fontweight='bold')
    ax.set_xticks(indices)
    ax.set_xticklabels([f'[{i}]' for i in indices], rotation=90, fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    # Set y-axis limit
    if cpu_stats:
        max_val = max(s['max'] for s in cpu_stats)
        ax.set_ylim(bottom=0, top=max(max_val * 1.1, 10))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"CPU distribution plot saved to: {output_path}")
    plt.close()


def plot_memory_distribution(metrics: Dict[str, Tuple[List[float], List[float], List[float]]], output_path: Path, node_colors: Dict[int, Tuple[str, any]]):
    """
    Plot memory usage distribution for all nodes showing min, Q1, median, Q3, and max.

    Args:
        metrics: Node metrics data
        output_path: Output path for the plot
        node_colors: Node index to name mapping for colors
    """
    fig, ax = plt.subplots(figsize=(16, 8))

    # Prepare data for each node
    indices = []
    mem_stats = []
    colors = []

    for idx, (node_name, (times, _, rss_vals)) in enumerate(sorted(metrics.items())):
        if not rss_vals:
            continue

        indices.append(idx)
        mem_array = np.array(rss_vals)

        # Calculate statistics
        stats = {
            'min': np.min(mem_array),
            'q1': np.percentile(mem_array, 25),
            'median': np.percentile(mem_array, 50),
            'q3': np.percentile(mem_array, 75),
            'max': np.max(mem_array)
        }
        mem_stats.append(stats)
        colors.append(node_colors[idx][1] if idx in node_colors else 'blue')

    # Plot error bars showing range
    for i, (idx, stats, color) in enumerate(zip(indices, mem_stats, colors)):
        # Draw vertical line from min to max
        ax.plot([idx, idx], [stats['min'], stats['max']],
                color=color, alpha=0.4, linewidth=1.5, zorder=1)

        # Draw box from Q1 to Q3
        box_height = stats['q3'] - stats['q1']
        rect = plt.Rectangle((idx - 0.3, stats['q1']), 0.6, box_height,
                             facecolor=color, edgecolor='black', alpha=0.6, linewidth=1, zorder=2)
        ax.add_patch(rect)

        # Draw median line
        ax.plot([idx - 0.3, idx + 0.3], [stats['median'], stats['median']],
                color='black', linewidth=2, zorder=3)

        # Draw min and max markers
        ax.plot(idx, stats['min'], 'o', color=color, markersize=4, zorder=2)
        ax.plot(idx, stats['max'], 'o', color=color, markersize=4, zorder=2)

    ax.set_xlabel('Node Index', fontsize=12)
    ax.set_ylabel('Memory Usage (MB)', fontsize=12)
    ax.set_title('Memory Usage Distribution by Node (Min, Q1, Median, Q3, Max)',
                 fontsize=14, fontweight='bold')
    ax.set_xticks(indices)
    ax.set_xticklabels([f'[{i}]' for i in indices], rotation=90, fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    # Set y-axis limit
    if mem_stats:
        max_val = max(s['max'] for s in mem_stats)
        ax.set_ylim(bottom=0, top=max(max_val * 1.1, 100))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Memory distribution plot saved to: {output_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Plot CPU and memory usage from play_launch logs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Plot from latest log
  ./plot_resource_usage.py

  # Plot from specific log directory
  ./plot_resource_usage.py --log-dir play_log/2025-10-15_18-13-00

  # Save plots to custom location
  ./plot_resource_usage.py --output-dir /tmp/plots
        '''
    )

    parser.add_argument(
        '--log-dir',
        type=Path,
        help='Path to specific log directory (default: latest in play_log/)'
    )

    parser.add_argument(
        '--base-log-dir',
        type=Path,
        default=Path('play_log'),
        help='Base log directory containing timestamped logs (default: play_log/)'
    )

    parser.add_argument(
        '--output-dir',
        type=Path,
        help='Directory to save plots (default: same as log directory)'
    )

    args = parser.parse_args()

    # Determine which log directory to use
    if args.log_dir:
        log_dir = args.log_dir
    else:
        log_dir = find_latest_log_dir(args.base_log_dir)

    print(f"Using log directory: {log_dir}")

    # Find metrics directory
    metrics_dir = log_dir / "metrics"
    if not metrics_dir.exists():
        print(f"Error: Metrics directory not found: {metrics_dir}")
        print("Ensure the log was generated with resource monitoring enabled (--enable-monitoring)")
        sys.exit(1)

    # Load all metrics
    try:
        metrics = load_all_metrics(metrics_dir)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)

    if not metrics:
        print("Error: No metrics data found")
        sys.exit(1)

    # Determine output directory - create plots subdirectory
    base_output_dir = args.output_dir if args.output_dir else log_dir
    output_dir = base_output_dir / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate plots
    print("\nGenerating plots...")
    node_colors = plot_cpu_usage(metrics, output_dir / "cpu_usage.png")
    plot_memory_usage(metrics, output_dir / "memory_usage.png", node_colors)
    plot_cpu_distribution(metrics, output_dir / "cpu_distribution.png", node_colors)
    plot_memory_distribution(metrics, output_dir / "memory_distribution.png", node_colors)
    create_legend_image(node_colors, output_dir / "legend.png")

    # Generate statistics
    print("Generating statistics...")
    stats_report = calculate_statistics(metrics, node_colors)
    stats_path = output_dir / "statistics.txt"
    with open(stats_path, 'w') as f:
        f.write(stats_report)
    print(f"Statistics saved to: {stats_path}")

    # Generate container listing
    print("Generating container listing...")
    container_listing = generate_container_listing(log_dir, node_colors)
    container_path = output_dir / "containers.txt"
    with open(container_path, 'w') as f:
        f.write(container_listing)
    print(f"Container listing saved to: {container_path}")

    print(f"\nAll plots saved to: {output_dir}")
    print("Done!")


if __name__ == '__main__':
    main()

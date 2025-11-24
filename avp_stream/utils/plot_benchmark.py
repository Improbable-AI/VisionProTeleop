"""Plot benchmark results from pre-recorded JSON file.

This script reads benchmark data (typically from benchmark_comparison.json) and
generates a latency plot. By default, it filters to show only gRPC results.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


def load_benchmark_data(input_path: Path) -> Dict:
    """Load benchmark data from JSON file."""
    if not input_path.exists():
        raise FileNotFoundError(f"Benchmark file not found: {input_path}")
    
    with open(input_path, "r") as f:
        data = json.load(f)
    
    return data


def filter_results_by_backend(
    results: List[Dict], 
    backends: List[str]
) -> List[Dict]:
    """Filter results to include only specified backends."""
    if not backends:
        return results
    
    backends_lower = [b.lower() for b in backends]
    return [
        entry for entry in results 
        if str(entry.get("backend", "")).lower() in backends_lower
    ]


def plot_benchmark_results(
    data: Dict,
    output_path: Path,
    backends: List[str] | None = None,
    title: str | None = None,
) -> None:
    """Generate and save a latency plot from benchmark data.
    
    Args:
        data: Benchmark data dictionary with 'config' and 'results' keys
        output_path: Path where the plot image will be saved
        backends: List of backends to include in plot (None = all)
        title: Custom title for the plot (None = default)
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("❌ matplotlib is not installed. Install it with: pip install matplotlib")
        return
    
    config = data.get("config", {})
    results = data.get("results", [])
    
    if not results:
        print("⚠️ No results found in benchmark data.")
        return
    
    # Filter by backend if specified
    if backends:
        results = filter_results_by_backend(results, backends)
        if not results:
            print(f"⚠️ No results found for backends: {backends}")
            return
    
    # Get label ordering from config or deduce from results
    label_order = config.get("labels", [])
    if not label_order:
        # Deduce from results
        seen_labels = []
        for entry in results:
            label = entry.get("label", entry.get("resolution", ""))
            if label and label not in seen_labels:
                seen_labels.append(label)
        label_order = seen_labels
    
    if not label_order:
        print("⚠️ No resolution labels found in data.")
        return
    
    label_to_index = {label: idx for idx, label in enumerate(label_order)}
    x_positions = np.arange(len(label_order))
    
    # Group results by backend
    backends_in_data = []
    for entry in results:
        backend = str(entry.get("backend", "unknown"))
        if backend not in backends_in_data:
            backends_in_data.append(backend)
    
    if not backends_in_data:
        print("⚠️ No backend information found in results.")
        return
    
    # Prepare data series
    series_data: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for backend in backends_in_data:
        means = np.full(len(label_order), np.nan, dtype=float)
        jitters = np.zeros(len(label_order), dtype=float)
        series_data[backend] = (means, jitters)
    
    # Fill in the data
    for entry in results:
        backend = str(entry.get("backend", "unknown"))
        if backend not in series_data:
            continue
        
        label = str(entry.get("label", entry.get("resolution", "")))
        if label not in label_to_index:
            continue
        
        summary = entry.get("summary") or {}
        mean = summary.get("mean_ms")
        jitter = summary.get("jitter_ms", 0.0)
        
        if mean is None:
            continue
        
        idx = label_to_index[label]
        series_data[backend][0][idx] = float(mean)
        series_data[backend][1][idx] = float(jitter)
    
    # Create the plot
    plt.figure(figsize=(12, 4))
    
    color_cycle = plt.rcParams.get("axes.prop_cycle", None)
    colors = None
    if color_cycle is not None:
        colors = [c["color"] for c in color_cycle]
    
    any_series = False
    for idx_backend, backend in enumerate(backends_in_data):
        means, jitters = series_data.get(backend, (None, None))
        if means is None:
            continue
        
        valid_mask = ~np.isnan(means)
        if not np.any(valid_mask):
            continue
        
        color = None
        if colors is not None:
            color = colors[idx_backend % len(colors)]
        
        label = f"Hand tracking stream with: {backend}"
        plt.plot(
            x_positions[valid_mask], 
            means[valid_mask], 
            marker="o", 
            label=label, 
            color=color
        )
        
        # Add jitter shading
        lower = np.maximum(0.0, means[valid_mask] - jitters[valid_mask])
        upper = means[valid_mask] + jitters[valid_mask]
        plt.fill_between(
            x_positions[valid_mask],
            lower,
            upper,
            color=color,
            alpha=0.2,
        )
        any_series = True
    
    if not any_series:
        print("⚠️ No valid data to plot.")
        return
    
    plt.xticks(x_positions, label_order)
    plt.ylabel("Round-trip latency (ms)")
    plt.xlabel("Resolution")
    
    if title:
        plt.title(title)
    else:
        plt.title("VisionOS round-trip latency sweep")
    
    plt.grid(axis="y", linestyle="--", alpha=0.3)
    plt.legend(loc="best")
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    
    print(f"✅ Saved latency plot to {output_path}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot benchmark results from pre-recorded JSON file"
    )
    parser.add_argument(
        "input",
        type=Path,
        help="Path to benchmark JSON file (e.g., assets/benchmark_comparison.json)"
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        help="Output path for the plot image (default: input filename with .png extension)"
    )
    parser.add_argument(
        "--backend",
        action="append",
        dest="backends",
        help="Filter to specific backend(s) (can be used multiple times). Default: all backends"
    )
    parser.add_argument(
        "--all-backends",
        action="store_true",
        help="Include all backends (overrides --backend filter)"
    )
    parser.add_argument(
        "--title",
        type=str,
        help="Custom title for the plot"
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    
    # Load data
    try:
        data = load_benchmark_data(args.input)
    except FileNotFoundError as e:
        print(f"❌ {e}")
        return
    except json.JSONDecodeError as e:
        print(f"❌ Failed to parse JSON file: {e}")
        return
    
    # Determine output path
    if args.output:
        output_path = args.output
    else:
        output_path = args.input.with_suffix(".png")
    
    # Determine which backends to plot
    backends = args.backends
    if args.all_backends:
        backends = None
    
    # Generate plot
    plot_benchmark_results(
        data=data,
        output_path=output_path,
        backends=backends,
        title=args.title,
    )


if __name__ == "__main__":
    main()

import json
import argparse
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def load_benchmark(filepath):
    """Load a benchmark JSON file."""
    with open(filepath, 'r') as f:
        return json.load(f)

def plot_benchmarks(json_files, output_path=None):
    """
    Plot multiple benchmark results comparing mean latency and jitter across resolutions.
    Separates mono and stereo benchmarks into two columns.
    
    Args:
        json_files: List of paths to benchmark JSON files
        output_path: Optional path to save the plot (if None, displays interactively)
    """
    # Color palette for different benchmark files
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    # Resolution order and labels
    resolution_order = ['240p', '360p', '720p', '1080p', '2160p']
    resolution_labels_mono = {
        '240p': '240p\n(426×240)',
        '360p': '360p\n(640×360)',
        '720p': '720p\n(1280×720)',
        '1080p': '1080p\n(1920×1080)',
        '2160p': '2160p\n(3840×2160)'
    }
    resolution_labels_stereo = {
        '240p': '240p\n(852×240)',
        '360p': '360p\n(1280×360)',
        '720p': '720p\n(2560×720)',
        '1080p': '1080p\n(3840×1080)',
        '2160p': '2160p\n(7680×2160)'
    }
    
    # Load and separate benchmark data into mono and stereo
    mono_benchmarks = []
    stereo_benchmarks = []
    
    for filepath in json_files:
        data = load_benchmark(filepath)
        label = Path(filepath).stem  # Use filename without extension as label
        benchmark = {
            'label': label,
            'data': data
        }
        
        # Check if filename contains "stereo"
        if 'stereo' in label.lower():
            stereo_benchmarks.append(benchmark)
        else:
            mono_benchmarks.append(benchmark)
    
    # Create figure with two subplots (side by side) with shared y-axis
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6), sharey=True)
    fig.suptitle('Round-Trip Latency Test', fontsize=16, fontweight='bold')
    
    x_positions = np.arange(len(resolution_order))
    
    # Helper function to plot benchmarks on a given axis
    def plot_on_axis(ax, benchmarks, title, is_stereo=False):
        for idx, benchmark in enumerate(benchmarks):
            means = []
            stds = []
            p95s = []
            p99s = []
            
            for res in resolution_order:
                if res in benchmark['data']['resolutions']:
                    res_data = benchmark['data']['resolutions'][res]
                    if 'error' not in res_data:
                        means.append(res_data['mean_ms'])
                        stds.append(res_data['std_ms'])
                        p95s.append(res_data.get('p95_ms', 0))
                        p99s.append(res_data.get('p99_ms', 0))
                    else:
                        means.append(0)
                        stds.append(0)
                        p95s.append(0)
                        p99s.append(0)
                else:
                    means.append(0)
                    stds.append(0)
                    p95s.append(0)
                    p99s.append(0)
            
            means = np.array(means)
            stds = np.array(stds)
            
            color = colors[idx % len(colors)]
            
            # Plot mean line
            ax.plot(x_positions, means, marker='o', linewidth=2, 
                   label=f'{benchmark["label"]}', color=color, markersize=8)
            
            # Plot shaded region for ±1 std deviation (no legend entry)
            ax.fill_between(x_positions, means - stds, means + stds, 
                           alpha=0.3, color=color)
        
        ax.set_xlabel('Resolution', fontsize=12, fontweight='bold')
        ax.set_ylabel('Latency (ms)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=13, fontweight='bold', pad=10)
        ax.set_xticks(x_positions)
        
        # Use appropriate labels based on mono/stereo
        labels = resolution_labels_stereo if is_stereo else resolution_labels_mono
        ax.set_xticklabels([labels[r] for r in resolution_order])
        ax.legend(loc='upper left', framealpha=0.9, fontsize=12)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
    
    # Plot mono benchmarks on left
    plot_on_axis(ax1, mono_benchmarks, 'Mono', is_stereo=False)
    
    # Plot stereo benchmarks on right
    plot_on_axis(ax2, stereo_benchmarks, 'Stereo', is_stereo=True)
    
    # Add test configuration info as text
    all_benchmarks = mono_benchmarks + stereo_benchmarks
    if all_benchmarks:
        config = all_benchmarks[0]['data']['test_config']
        info_text = f"Test Config: {config['trials_per_resolution']} trials/resolution. For stereo tests, width is doubled."
        if 'warmup_frames' in config:
            info_text += f", {config['warmup_frames']} warmup frames"
        fig.text(0.5, 0.02, info_text, ha='center', fontsize=10, style='italic', color='gray')
    
    plt.tight_layout(rect=[0, 0.04, 1, 0.96])
    
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"✅ Plot saved to: {output_path}")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="Plot multiple benchmark results for comparison")
    parser.add_argument('json_files', nargs='+', help='Paths to benchmark JSON files')
    parser.add_argument('--output', '-o', type=str, help='Output path for the plot (PNG/PDF)')
    args = parser.parse_args()
    
    # Verify all files exist
    for filepath in args.json_files:
        if not Path(filepath).exists():
            print(f"❌ Error: File not found: {filepath}")
            return
    
    print(f"📊 Plotting {len(args.json_files)} benchmark file(s)...")
    plot_benchmarks(args.json_files, args.output)

if __name__ == "__main__":
    main()

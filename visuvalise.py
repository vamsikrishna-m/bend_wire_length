"""
3D Visualization for pythonocc-based Wire Analyzer
Works with any number of bends
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))

from step_analyzer_final import analyze_step_file


def plot_wire_3d(analyzer, title="Wire Geometry", show_segment_numbers=True):
    """
    Plot 3D wire geometry with bends highlighted
    
    Args:
        analyzer: STEPWireAnalyzer object
        title: Plot title
        show_segment_numbers: Show segment numbers on plot
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    if not analyzer.centerline_points:
        print("No centerline points to plot")
        return None, None
    
    points = np.array(analyzer.centerline_points)
    
    # Plot centerline
    ax.plot(points[:, 0], points[:, 1], points[:, 2], 
            'b-', linewidth=3, label='Centerline', alpha=0.8)
    
    # Mark start and end
    ax.scatter([points[0, 0]], [points[0, 1]], [points[0, 2]], 
              c='green', s=300, marker='o', label='Start', 
              edgecolors='black', linewidths=2, zorder=5)
    ax.scatter([points[-1, 0]], [points[-1, 1]], [points[-1, 2]], 
              c='red', s=300, marker='s', label='End', 
              edgecolors='black', linewidths=2, zorder=5)
    
    # Mark and label each bend
    points_per_segment = 20
    current_idx = 0
    
    bend_num = 1
    straight_num = 1
    
    for segment in analyzer.segments:
        mid_idx = current_idx + points_per_segment // 2
        
        if mid_idx < len(points):
            mid_point = points[mid_idx]
            
            if segment.type == 'BEND':
                # Mark bend location
                ax.scatter([mid_point[0]], [mid_point[1]], [mid_point[2]], 
                         c='orange', s=250, marker='^', alpha=0.9, 
                         edgecolors='black', linewidths=2, zorder=4)
                
                # Label bend
                if show_segment_numbers:
                    label_text = f"B{bend_num}"
                    if segment.radius and segment.angle_deg:
                        label_text += f"\nR={segment.radius:.1f}\nθ={segment.angle_deg:.0f}°"
                    
                    ax.text(mid_point[0], mid_point[1], mid_point[2], 
                           label_text, fontsize=9, fontweight='bold',
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='orange', alpha=0.7))
                
                bend_num += 1
            else:
                # Mark straight segment
                if show_segment_numbers:
                    ax.scatter([mid_point[0]], [mid_point[1]], [mid_point[2]], 
                             c='lightblue', s=100, marker='o', alpha=0.6, zorder=3)
                    
                    ax.text(mid_point[0], mid_point[1], mid_point[2], 
                           f"S{straight_num}", fontsize=8,
                           bbox=dict(boxstyle='round,pad=0.2', facecolor='lightblue', alpha=0.6))
                
                straight_num += 1
        
        current_idx += points_per_segment
    
    # Formatting
    ax.set_xlabel('X (mm)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (mm)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z (mm)', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    
    # Equal aspect ratio
    max_range = np.array([
        points[:, 0].max() - points[:, 0].min(),
        points[:, 1].max() - points[:, 1].min(),
        points[:, 2].max() - points[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    ax.legend(loc='upper right', fontsize=11)
    ax.grid(True, alpha=0.3)
    
    # Info box
    info_text = f"Total Length: {analyzer.total_length:.2f} mm\n"
    info_text += f"Bends: {analyzer.bend_count}\n"
    info_text += f"Straights: {analyzer.straight_count}\n"
    info_text += f"Total Segments: {len(analyzer.segments)}"
    
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.9)
    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, 
             fontsize=11, verticalalignment='top', bbox=props, 
             family='monospace', fontweight='bold')
    
    # Bend details box
    if analyzer.bend_count > 0:
        bend_text = "Bend Details:\n"
        bend_num = 1
        for segment in analyzer.segments:
            if segment.type == 'BEND':
                bend_text += f"  B{bend_num}: "
                if segment.radius:
                    bend_text += f"R={segment.radius:.1f}mm "
                if segment.angle_deg:
                    bend_text += f"θ={segment.angle_deg:.1f}°"
                bend_text += "\n"
                bend_num += 1
                
                # Limit to first 5 bends in text box
                if bend_num > 6:
                    bend_text += f"  ... ({analyzer.bend_count - 5} more)\n"
                    break
        
        props2 = dict(boxstyle='round', facecolor='lightblue', alpha=0.9)
        ax.text2D(0.02, 0.65, bend_text, transform=ax.transAxes, 
                 fontsize=10, verticalalignment='top', bbox=props2, 
                 family='monospace')
    
    plt.tight_layout()
    
    return fig, ax


def create_bend_statistics_plot(analyzer):
    """Create a plot showing bend statistics"""
    
    bends = [s for s in analyzer.segments if s.type == 'BEND' and s.radius and s.angle_deg]
    
    if not bends:
        print("No bends with complete data to plot")
        return None
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Extract data
    radii = [b.radius for b in bends]
    angles = [b.angle_deg for b in bends]
    lengths = [b.length for b in bends]
    bend_numbers = list(range(1, len(bends) + 1))
    
    # Plot 1: Radius per bend
    axes[0, 0].bar(bend_numbers, radii, color='steelblue', edgecolor='black', linewidth=1.5)
    axes[0, 0].set_xlabel('Bend Number', fontsize=12, fontweight='bold')
    axes[0, 0].set_ylabel('Radius (mm)', fontsize=12, fontweight='bold')
    axes[0, 0].set_title('Bend Radius', fontsize=14, fontweight='bold')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Plot 2: Angle per bend
    axes[0, 1].bar(bend_numbers, angles, color='coral', edgecolor='black', linewidth=1.5)
    axes[0, 1].set_xlabel('Bend Number', fontsize=12, fontweight='bold')
    axes[0, 1].set_ylabel('Angle (degrees)', fontsize=12, fontweight='bold')
    axes[0, 1].set_title('Bend Angle', fontsize=14, fontweight='bold')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Arc length per bend
    axes[1, 0].bar(bend_numbers, lengths, color='mediumseagreen', edgecolor='black', linewidth=1.5)
    axes[1, 0].set_xlabel('Bend Number', fontsize=12, fontweight='bold')
    axes[1, 0].set_ylabel('Arc Length (mm)', fontsize=12, fontweight='bold')
    axes[1, 0].set_title('Bend Arc Length', fontsize=14, fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Plot 4: Summary pie chart
    straight_length = sum(s.length for s in analyzer.segments if s.type == 'STRAIGHT')
    bend_length = sum(s.length for s in analyzer.segments if s.type == 'BEND')
    
    axes[1, 1].pie([straight_length, bend_length], 
                   labels=['Straight', 'Bent'], 
                   colors=['lightblue', 'orange'],
                   autopct='%1.1f%%',
                   startangle=90,
                   textprops={'fontsize': 12, 'fontweight': 'bold'})
    axes[1, 1].set_title('Length Distribution', fontsize=14, fontweight='bold')
    
    # Add overall stats
    stats_text = f"Total Bends: {analyzer.bend_count}\n"
    stats_text += f"Avg Radius: {np.mean(radii):.2f} mm\n"
    stats_text += f"Avg Angle: {np.mean(angles):.2f}°\n"
    stats_text += f"Total Length: {analyzer.total_length:.2f} mm"
    
    fig.text(0.5, 0.02, stats_text, ha='center', fontsize=11, 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
             family='monospace', fontweight='bold')
    
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    
    return fig


def main():
    """Main visualization function"""
    
    print("=" * 70)
    print("3D WIRE GEOMETRY VISUALIZATION (pythonocc-core)")
    print("=" * 70)
    print()
    
    # Check for command line argument
    if len(sys.argv) > 1:
        step_file = sys.argv[1]
    else:
        # Default: look for sample files
        step_file = "step_samples/3_bend_wire.step"
    
    if not Path(step_file).exists():
        print(f"Error: File not found: {step_file}")
        print("\nUsage: python visualize_wires_occ.py <step_file>")
        return
    
    # Analyze file
    print(f"Analyzing: {step_file}\n")
    analyzer = analyze_step_file(step_file)
    
    # Create output directory
    output_dir = Path("visualizations_occ")
    output_dir.mkdir(exist_ok=True)
    
    # Create 3D plot
    print("\nCreating 3D visualization...")
    fig1, ax1 = plot_wire_3d(analyzer, f"Wire Geometry: {Path(step_file).stem}")
    
    if fig1:
        output_file = output_dir / f"{Path(step_file).stem}_3d.png"
        fig1.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✓ Saved: {output_file}")
        plt.close(fig1)
    
    # Create statistics plot
    if analyzer.bend_count > 0:
        print("Creating statistics plot...")
        fig2 = create_bend_statistics_plot(analyzer)
        
        if fig2:
            output_file = output_dir / f"{Path(step_file).stem}_stats.png"
            fig2.savefig(output_file, dpi=150, bbox_inches='tight')
            print(f"✓ Saved: {output_file}")
            plt.close(fig2)
    
    print()
    print("=" * 70)
    print("✓ Visualization complete!")
    print(f"✓ Files saved to: {output_dir}/")
    print("=" * 70)


if __name__ == "__main__":
    main()

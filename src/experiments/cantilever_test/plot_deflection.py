import os
import pandas as pd
import matplotlib.pyplot as plt

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    csv_path = os.path.join(project_root, "data", "usd_models", "cantilever_log.csv")
    
    if not os.path.exists(csv_path):
        print(f"Error: CSV file not found at {csv_path}")
        print("Run the simulation first to log data.")
        return
        
    print(f"Reading data from {csv_path}...")
    df = pd.read_csv(csv_path)
    
    if df.empty:
        print("CSV is empty. No deflection was logged.")
        return
        
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # Plot Deflection vs Time on left Y-axis
    ax1.plot(df['Time'], df['Deflection_mm'], label='Simulated Deflection (Isaac Sim)', color='blue', linewidth=2)
    
    # Theoretical Ground Truth (7.6 mm)
    theoretical_deflection = 7.6
    ax1.axhline(y=theoretical_deflection, color='red', linestyle='--', linewidth=2, label=f'Theoretical Deflection ({theoretical_deflection} mm)')
    
    ax1.set_xlabel('Time (seconds)', fontsize=14)
    ax1.set_ylabel('Deflection (mm)', fontsize=14, color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1.grid(True, linestyle='--', alpha=0.7)
    
    # Secondary Y-axis for Force
    ax2 = ax1.twinx()
    ax2.plot(df['Time'], df['Force_N'], label='Applied Force (N)', color='orange', linewidth=2)
    ax2.set_ylabel('Force (N)', fontsize=14, color='orange')
    ax2.tick_params(axis='y', labelcolor='orange')
    
    plt.title('Cantilever Bending Test (Manual Force)', fontsize=16)
    
    # Combine legends from both axes
    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper left')
    
    fig.tight_layout()
    
    save_path = os.path.join(script_dir, "Figure_1.png")
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot automatically saved to {save_path}")
    
    print("Opening plot...")
    plt.show()

if __name__ == "__main__":
    main()

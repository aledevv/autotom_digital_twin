import os
import pandas as pd
import matplotlib.pyplot as plt

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    csv_path = os.path.join(project_root, "data", "usd_models", "forces_log.csv")
    
    if not os.path.exists(csv_path):
        print(f"Error: CSV file not found at {csv_path}")
        print("Run the simulation first and poke the branch to log data.")
        return
        
    print(f"Reading data from {csv_path}...")
    df = pd.read_csv(csv_path)
    
    if df.empty:
        print("CSV is empty. No forces were logged.")
        return
        
    plt.figure(figsize=(12, 6))
    
    # Plot a line for each unique joint
    joints = df['JointName'].unique()
    print(f"Found {len(joints)} joints in the log.")
    
    for joint in joints:
        joint_data = df[df['JointName'] == joint]
        plt.plot(joint_data['Time'], joint_data['F_norm'], label=joint, linewidth=2)
        
    plt.title('Force Magnitude on Joints Over Time', fontsize=16)
    plt.xlabel('Time (seconds)', fontsize=14)
    plt.ylabel('Force Norm (Newtons)', fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
    plt.tight_layout()
    
    save_path = os.path.join(script_dir, "Figure_1.png")
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot automatically saved to {save_path}")
    
    print("Opening plot...")
    plt.show()

if __name__ == "__main__":
    main()

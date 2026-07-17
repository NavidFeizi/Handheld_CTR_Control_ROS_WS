import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path

# Enable LaTeX rendering in matplotlib
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 11

output_file = Path(__file__).parent / "targetVelocity.csv"

# Parameters
sample_time = 10e-3  # 25 milliseconds
duration = 60.0      # 10 seconds
initial_position = np.array([-0.070, -0.035, 0.0, np.pi/2, np.pi/2, 0.0])  # [β1, β2, β3, α1, α2, α3]

# Generate time array
num_samples = int(duration / sample_time) + 1
time_array = np.linspace(0, duration, num_samples)

# Generate sinusoidal velocities
beta1_vel = -3.0e-3 * np.sin(2 * np.pi * (4/duration) * time_array)
beta2_vel = -1.5e-3 * np.sin(2 * np.pi * (4/duration) * time_array)
beta3_vel = np.zeros_like(time_array)

alpha3_vel = np.zeros_like(time_array)
alpha2_vel = np.ones_like(time_array) * (-np.pi / duration)
alpha1_vel = alpha2_vel + 0.5 * np.cos(2 * np.pi * (2/duration) * time_array)


# Integrate velocities to compute positions using cumulative trapezoidal integration
beta1_pos = initial_position[0] + np.cumsum(beta1_vel * sample_time)
beta2_pos = initial_position[1] + np.cumsum(beta2_vel * sample_time)
beta3_pos = initial_position[2] + np.cumsum(beta3_vel * sample_time)
alpha1_pos = initial_position[3] + np.cumsum(alpha1_vel * sample_time)
alpha2_pos = initial_position[4] + np.cumsum(alpha2_vel * sample_time)
alpha3_pos = initial_position[5] + np.cumsum(alpha3_vel * sample_time)

# Create DataFrame with all velocity components (saved to CSV)
velocity_data = {
    'timestamp': time_array,
    'beta1_vel': beta1_vel,
    'beta2_vel': beta2_vel,
    'beta3_vel': beta3_vel,
    'alpha1_vel': alpha1_vel,
    'alpha2_vel': alpha2_vel,
    'alpha3_vel': alpha3_vel,
}

df_velocity = pd.DataFrame(velocity_data)

# Create DataFrame with positions (for plotting only, not saved)
position_data = {
    'timestamp': time_array,
    'beta1_pos': beta1_pos,
    'beta2_pos': beta2_pos,
    'beta3_pos': beta3_pos,
    'alpha1_pos': alpha1_pos,
    'alpha2_pos': alpha2_pos,
    'alpha3_pos': alpha3_pos,
}

df_position = pd.DataFrame(position_data)

# Write velocity targets to CSV file
output_file.parent.mkdir(parents=True, exist_ok=True)

df_velocity.to_csv(output_file, index=False, float_format='%.6f')

print(f"Generated velocity targets: {output_file}")
print(f"Number of samples: {len(df_velocity)}")
print(f"Duration: {duration} seconds")
print(f"Sample time: {sample_time*1e3} ms")
print(f"\nVelocity Data (first 5 rows):")
print(df_velocity.head())
print(f"\nPosition Data (first 5 rows):")
print(df_position.head())


def plot_velocity_and_position(df_vel, df_pos, save_dir=None):
    """
    Plot velocity and position trajectories with LaTeX-formatted labels.
    
    Args:
        df_vel: DataFrame with velocity data
        df_pos: DataFrame with position data
        save_dir: Directory to save plot (optional)
    """
    fig = plt.figure(figsize=(8, 9))
    gs = GridSpec(4, 1, figure=fig, height_ratios=[1, 1, 1, 1], hspace=0.35)
    
    # Extract time data
    time = df_vel['timestamp'].values
    
    # ===== VELOCITY PLOTS =====
    
    # Plot Beta Velocities
    ax_beta_vel = fig.add_subplot(gs[0, 0])
    ax_beta_vel.plot(time, df_vel['beta1_vel'].values, 'b-', linewidth=2, label=r'$\dot{\beta}_1$')
    ax_beta_vel.plot(time, df_vel['beta2_vel'].values, 'r-', linewidth=2, label=r'$\dot{\beta}_2$')
    ax_beta_vel.plot(time, df_vel['beta3_vel'].values, 'g-', linewidth=2, label=r'$\dot{\beta}_3$')
    ax_beta_vel.set_ylabel('Velocity [m/s]', fontsize=10)
    ax_beta_vel.set_title("Beta Joint Velocities vs Time", fontsize=10)
    ax_beta_vel.grid(True, alpha=0.3, linestyle='-')
    ax_beta_vel.legend(loc='upper right', fontsize=10, framealpha=0.9)
    
    # Plot Alpha Velocities
    ax_alpha_vel = fig.add_subplot(gs[1, 0])
    ax_alpha_vel.plot(time, df_vel['alpha1_vel'].values, 'b-', linewidth=2, label=r'$\dot{\alpha}_1$')
    ax_alpha_vel.plot(time, df_vel['alpha2_vel'].values, 'r-', linewidth=2, label=r'$\dot{\alpha}_2$')
    ax_alpha_vel.plot(time, df_vel['alpha3_vel'].values, 'g-', linewidth=2, label=r'$\dot{\alpha}_3$')
    ax_alpha_vel.set_ylabel('Velocity [rad/s]', fontsize=10)
    ax_alpha_vel.set_title("Alpha Joint Velocities vs Time", fontsize=10)
    ax_alpha_vel.grid(True, alpha=0.3, linestyle='-')
    ax_alpha_vel.legend(loc='upper right', fontsize=10, framealpha=0.9)
    
    # ===== POSITION PLOTS (Integrated from velocities) =====
    
    # Plot Beta Positions
    ax_beta_pos = fig.add_subplot(gs[2, 0])
    ax_beta_pos.plot(time, df_pos['beta1_pos'].values, 'b-', linewidth=2, label=r'$\beta_1$')
    ax_beta_pos.plot(time, df_pos['beta2_pos'].values, 'r-', linewidth=2, label=r'$\beta_2$')
    ax_beta_pos.plot(time, df_pos['beta3_pos'].values, 'g-', linewidth=2, label=r'$\beta_3$')
    ax_beta_pos.set_ylabel('Position [rad]', fontsize=10)
    ax_beta_pos.set_title("Beta Joint Positions vs Time (Integrated)", fontsize=10)
    ax_beta_pos.grid(True, alpha=0.3, linestyle='-')
    ax_beta_pos.legend(loc='upper right', fontsize=11, framealpha=0.9)
    
    # Plot Alpha Positions
    ax_alpha_pos = fig.add_subplot(gs[3, 0])
    ax_alpha_pos.plot(time, df_pos['alpha1_pos'].values, 'b-', linewidth=2, label=r'$\alpha_1$')
    ax_alpha_pos.plot(time, df_pos['alpha2_pos'].values, 'r-', linewidth=2, label=r'$\alpha_2$')
    ax_alpha_pos.plot(time, df_pos['alpha3_pos'].values, 'g-', linewidth=2, label=r'$\alpha_3$')
    ax_alpha_pos.set_xlabel('Time [s]', fontsize=10)
    ax_alpha_pos.set_ylabel('Position [rad]', fontsize=10)
    ax_alpha_pos.set_title("Alpha Joint Positions vs Time (Integrated)", fontsize=10)
    ax_alpha_pos.grid(True, alpha=0.3, linestyle='-')
    ax_alpha_pos.legend(loc='upper right', fontsize=10, framealpha=0.9)
    
    plt.tight_layout()
    
    # Save plot if save_dir provided
    if save_dir:
        save_dir = Path(save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        plot_file = save_dir / "velocity_and_position_trajectories.png"
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved: {plot_file}")
    
    plt.show()


# Generate plots
if __name__ == "__main__":
    plot_velocity_and_position(df_velocity, df_position, save_dir=Path(__file__).parent)
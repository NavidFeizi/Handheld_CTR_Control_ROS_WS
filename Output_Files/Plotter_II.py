import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def getSO3(h):
    I = np.identity(3)

    R = np.array([
        [-h[2] * h[2] - h[3] * h[3], h[1] * h[2] - h[3] * h[0], h[1] * h[3] + h[2] * h[0]],
        [h[1] * h[2] + h[3] * h[0], -h[1] * h[1] - h[3] * h[3], h[2] * h[3] - h[1] * h[0]],
        [h[1] * h[3] - h[2] * h[0], h[2] * h[3] + h[1] * h[0], -h[1] * h[1] - h[2] * h[2]]
    ])

    R *= 2.00 / np.linalg.norm(h)**2
    R += I

    return R

def homogeneous_transform(R, p):
    """
    Create a 4x4 homogeneous transformation matrix from a 3x3 rotation matrix R
    and a 3-dimensional translation vector p.

    Parameters:
    - R: 3x3 rotation matrix
    - p: 3-dimensional translation vector

    Returns:
    - T: 4x4 homogeneous transformation matrix
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

# Function to compute the inverse of a homogeneous transformation matrix
def inverse_homogeneous_transform(H):
    R_inv = np.linalg.inv(H[:3, :3])  # Inverse of the upper-left 3x3 rotation matrix
    p_inv = -np.dot(R_inv, H[:3, 3])   # Inverse of the translation vector

    H_inv = np.eye(4)
    H_inv[:3, :3] = R_inv
    H_inv[:3, 3] = p_inv

    return H_inv

# Read data from the .dat file
data = np.genfromtxt("Phantom/EM_Trajectory_hypo_2.dat", delimiter=",", dtype=float)

# Extract data columns
x_tip, y_tip, z_tip = data[:, 0] * 1.00E3, data[:, 1] * 1.00E3, data[:, 2] * 1.00E3
x_truth, y_truth, z_truth = data[:, 3] * 1.00E3, data[:, 4] * 1.00E3, data[:, 5] * 1.00E3
x_origin, y_origin, z_origin = data[:, 6] * 1.00E3, data[:, 7] * 1.00E3, data[:, 8] * 1.00E3
w_quaternion, x_quaternion, y_quaternion, z_quaternion = data[:, 9], data[:, 10], data[:, 11], data[:, 12]

# Initialize lists to store transformed points
transformed_points_tip = []
transformed_points_truth = []

# Iterate over each row of data
for i in range(len(data)):
    # Extract quaternion and translation vector for the current row
    h = np.array([w_quaternion[i], x_quaternion[i], y_quaternion[i], z_quaternion[i]])
    translation_vector = np.array([x_origin[i], y_origin[i], z_origin[i]])

    # Build rotation matrix using quaternion
    R = getSO3(h)

    # Build homogeneous transformation matrix
    H = homogeneous_transform(R, translation_vector)

    H_inv = inverse_homogeneous_transform(H)

    # Transform points using the homogeneous transformation matrix
    point_tip = np.array([x_tip[i], y_tip[i], z_tip[i], 1.00])
    point_truth = np.array([x_truth[i], y_truth[i], z_truth[i], 1.00])

    transformed_point_tip = np.dot(H_inv, point_tip)[:3]
    transformed_point_truth = np.dot(H_inv, point_truth)[:3]

    transformed_points_tip.append(transformed_point_tip)
    transformed_points_truth.append(transformed_point_truth)

# Convert lists to arrays
transformed_points_tip = np.array(transformed_points_tip)
transformed_points_truth = np.array(transformed_points_truth)

# Plot the original and transformed points with equal aspect ratio and LaTeX fonts
plt.rc('text', usetex=True)  # Enable LaTeX text rendering

# Plot the original and transformed points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax.scatter(x_tip, y_tip, z_tip, label=r'Original Tip Points', c='blue')
# ax.scatter(x_truth, y_truth, z_truth, label=r'Original Truth Points', c='green')
ax.scatter(transformed_points_tip[:, 0], transformed_points_tip[:, 1], transformed_points_tip[:, 2], label=r'CTR Distal End', c='red', marker='.')
ax.plot(transformed_points_truth[:, 0], transformed_points_truth[:, 1], transformed_points_truth[:, 2], label=r'Ground Truth', c='blue')

ax.set_xlabel(r'$x$ [mm]')
ax.set_ylabel(r'$y$ [mm]')
ax.set_zlabel(r'$z$ [mm]')
ax.legend()

# Set equal aspect ratio
ax.set_box_aspect(
    [np.ptp(arr) for arr in [ax.get_xlim(), ax.get_ylim(), ax.get_zlim()]]
)

# Apply tight layout
plt.tight_layout()

# show plot
plt.show()

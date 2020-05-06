import numpy as np
import matplotlib.pyplot as plt
import csv

# waypoint file to load
WAYPOINTS_FILENAME = r'C:\Users\Windows\Downloads\PyAdvancedControl\tracking_bicycle\racetrack_waypoints.txt'  
INTERP_DISTANCE_RES = 0.01  # distance between interpolated points
INTERP_LOOKAHEAD_DISTANCE = 20   # lookahead in meters

# Opens the waypoint file and stores it to "waypoints"
waypoints_file = WAYPOINTS_FILENAME
with open(waypoints_file) as waypoints_file_handle:
    waypoints = list(csv.reader(waypoints_file_handle,
                                delimiter=',',
                                quoting=csv.QUOTE_NONNUMERIC))
    waypoints_np = np.array(waypoints)

# Linear interpolation computations
wp_distance = []  # distance array
for i in range(1, waypoints_np.shape[0]):
    wp_distance.append(
        np.sqrt((waypoints_np[i, 0] - waypoints_np[i - 1, 0]) ** 2 +
                (waypoints_np[i, 1] - waypoints_np[i - 1, 1]) ** 2))
# last distance is 0 because it is the distance from the last waypoint to the last waypoint
wp_distance.append(0)

# Linearly interpolate between waypoints and store in a list
wp_interp = []  # interpolated values
# (rows = waypoints, columns = [x, y, v])
wp_interp_hash = []  # hash table which indexes waypoints_np
# to the index of the waypoint in wp_interp
interp_counter = 0  # counter for current interpolated point index
reached_the_end = False

for i in range(waypoints_np.shape[0] - 1):
    # Add original waypoint to interpolated waypoints list (and append
    # it to the hash table)
    wp_interp.append(list(waypoints_np[i]))
    wp_interp_hash.append(interp_counter)
    interp_counter += 1

    # Interpolate to the next waypoint. First compute the number of
    # points to interpolate based on the desired resolution and
    # incrementally add interpolated points until the next waypoint
    # is about to be reached.
    num_pts_to_interp = int(np.floor(wp_distance[i] / \
                                        float(INTERP_DISTANCE_RES)) - 1)
    wp_vector = waypoints_np[i + 1] - waypoints_np[i]
    wp_uvector = wp_vector / np.linalg.norm(wp_vector)
    for j in range(num_pts_to_interp):
        next_wp_vector = INTERP_DISTANCE_RES * float(j + 1) * wp_uvector
        wp_interp.append(list(waypoints_np[i] + next_wp_vector))
        interp_counter += 1
# add last waypoint at the end
wp_interp.append(list(waypoints_np[-1]))
wp_interp_hash.append(interp_counter)
interp_counter += 1

x_ref = list(waypoints_np[:, 0])
y_ref = list(waypoints_np[:, 1])
plt.plot(x_ref, y_ref, 'b--')
plt.xlim(-250, 400)
plt.ylim(-800, 100)
plt.show()
# Header of the last received input cloud (zero stamp before the first cloud).
std_msgs/Header header
# False at startup, on invalid clouds, or after cloud_timeout seconds without data.
bool data_valid
# True only while detection is enabled and at least one ROI cluster passes the filters.
bool obstacle_detected
# Always false while detection is paused, including invalid or timed-out input.
# While enabled: true only on accepted obstacles or valid-cloud acquisition timeout.
# Invalid frames do not reset the timeout. Waiting for a new frame alone does not stop.
bool stop_required
# Minimum forward X coordinate (meters) among accepted clusters, from the LiDAR.
# +inf: evaluated valid cloud with no accepted cluster.
# NaN: paused, data invalid, or waiting for a fresh evaluation after enable/config change.
float32 nearest_distance
# Total point count of all accepted ROI clusters.
uint32 obstacle_points
# Compatibility field: equals obstacle_points; the whole ROI is the stop zone.
uint32 stop_zone_points

# Gas concentration contour data for environmental monitoring missions.
# Isopleths represent regions of equal gas concentration, enabling visualization
# of plume boundaries and identification of high-concentration zones.

builtin_interfaces/Time stamp      # Observation timestamp for temporal correlation
string species                     # Chemical formula (e.g., "CH4", "H2S") being measured
string units                       # Concentration units (e.g., "ppm", "ppb") for interpretation
geometry_msgs/Polygon[] polygons   # Contour polygons at different concentration thresholds
geometry_msgs/Point32[] centerline # Spine of the plume for source localization algorithms

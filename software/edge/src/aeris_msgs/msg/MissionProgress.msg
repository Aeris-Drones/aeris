# Mission execution metrics for multi-drone search and survey operations.
# Provides real-time situational awareness for mission operators and progress tracking.

float64 coverage_percent         # Percentage of search area with adequate sensor coverage
float64 search_area_km2          # Total mission area assigned for the operation
float64 covered_area_km2         # Cumulative area processed by all active vehicles
uint8 active_drones              # Currently participating vehicles (may exclude RTB/failed)
uint8 total_drones               # Vehicles originally assigned to this mission
float64 estimated_time_remaining # Seconds until completion based on current progress rate
uint32 grid_completed            # Search cells finished across all vehicles
uint32 grid_total                # Total cells in the mission search grid

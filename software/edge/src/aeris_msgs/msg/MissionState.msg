# High-level mission lifecycle state for coordination and logging.
# State transitions trigger operational behaviors and are recorded for post-mission analysis.

string state          # Current state: "idle", "planning", "active", "paused", "completed", "aborted"
string mission_id     # UUID linking to the mission definition in the database
float64 timestamp     # State change epoch time for event sequencing
string previous_state # Prior state for transition validation and audit trails

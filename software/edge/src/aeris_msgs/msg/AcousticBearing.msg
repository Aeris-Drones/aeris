# Acoustic bearing measurement from microphone array processing

builtin_interfaces/Time stamp          # Measurement timestamp
float32 bearing_deg                    # Direction of arrival (degrees, 0-360)
float32 confidence                     # Detection confidence (0.0 to 1.0)
float32 snr_db                         # Signal-to-noise ratio in decibels
string mic_array                       # Identifier of the microphone array used
string vehicle_id                      # Vehicle that recorded the measurement
string classification                  # Acoustic source classification (e.g., "engine", "voice")

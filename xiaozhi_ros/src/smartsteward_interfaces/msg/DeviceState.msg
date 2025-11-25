# IoT device state message
# Represents the current state of a smart home device

string device_id       # Unique identifier for the device
string device_type     # Type of device (e.g., "lamp", "car", "sensor")
string state           # Current state (device-specific, e.g., "on", "off", "moving")
float64 value          # Optional numeric value (e.g., brightness level, temperature)
string[] properties    # Additional properties as key=value strings
std_msgs/Header header

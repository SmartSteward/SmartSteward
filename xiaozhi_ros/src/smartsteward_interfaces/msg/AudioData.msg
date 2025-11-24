# Audio data message for voice processing
# Contains raw audio samples and metadata

std_msgs/Header header
int32 sample_rate      # Sampling rate in Hz (e.g., 16000)
int32 channels         # Number of audio channels (1 for mono, 2 for stereo)
string encoding        # Audio encoding format (e.g., "pcm_s16le", "opus")
uint8[] data           # Raw audio data bytes

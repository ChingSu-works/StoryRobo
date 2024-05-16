import sounddevice as sd
import soundfile as sf

# sd.default.device = 2
# filename = 'output.wav'
# # Extract data and sampling rate from file
# data, fs = sf.read(filename, dtype='float32')  
# sd.play(data, fs)
# status = sd.wait()

def increase_volume(audio, factor):
    return audio * factor

# Set the default audio device
sd.default.device = 0

# Load audio file
filename = 'output.wav'
data, fs = sf.read(filename, dtype='float32')

# Increase volume by default (e.g., double the volume)
volume_factor = 10.0
data = increase_volume(data, volume_factor)

# Play the audio
sd.play(data, fs)
status = sd.wait()




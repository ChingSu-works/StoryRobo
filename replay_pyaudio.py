import sounddevice as sd
import soundfile as sf

sd.default.device = 2
filename = 'output.wav'
# Extract data and sampling rate from file
data, fs = sf.read(filename, dtype='float32')  
sd.play(data, fs)
status = sd.wait()



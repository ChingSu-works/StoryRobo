import pyaudio

# Create an instance of PyAudio
p = pyaudio.PyAudio()

# Get the number of audio I/O devices
devices = p.get_device_count()
# print(f'Total number of devices: {devices}')

# Iterate through all devices
for i in range(devices):
   # Get the device info
   device_info = p.get_device_info_by_index(i)
   # Check if this device is a microphone (an input device)
   if device_info.get('maxOutputChannels') >0:
      print(f"Microphone: {device_info.get('name')} , Device Index: {device_info.get('index')}")
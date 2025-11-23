# file: show_caps.py
import evdev
from evdev import InputDevice

for path in ['/dev/input/event17', '/dev/input/event18']:
    try:
        dev = InputDevice(path)
        print("====", path, dev.name, "====")
        caps = dev.capabilities(verbose=True)
        for etype, info in caps.items():
            print(" ", etype, ":", info)
        print()
    except Exception as e:
        print("Error opening", path, e)

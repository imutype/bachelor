from evdev import uinput, ecodes

with uinput.UInput() as ui:
    # ...
    ui.write(ecodes.EV_KEY, event.code, event.pressed)
    ui.syn()

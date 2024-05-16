# Core 2 Audio Monitor

An adaptation and simplification using UIX and the ESP LCD Panel API

- Less flicker than the original
- Supports the ESP-IDF and Arduino
- Doesn't use deprecated APIs
- No dependency to FastLED or the M5 stack libraries
- Uses the htcw_* ecosystem, including UIX and GFX for rendering.
- No more labyrinth of classes
- No more requiring PSRAM
- Far fewer dynamic allocations
- Main logic is isolated to ui.hpp and main.cpp
- More modern C++ using GP more than OOP

by honey the codewitch

Adapted from
# M5Stack Core 2 Audio


You can watch a video explainer [here (YouTube)](https://www.youtube.com/watch?v=CwIWpBqa-nM) which goes into a bit more detail on the audio capabilities of the device.

[![Demo Video](https://img.youtube.com/vi/CwIWpBqa-nM/0.jpg)](https://www.youtube.com/watch?v=CwIWpBqa-nM)

This project is a nice little demo of audio on the M5Stack Core 2 with some simple visualisations.

You'll need to use PlatformIO to build the project.

To clone the repo use:

```
git clone https://github.com/atomic14/m5stack-core2-audio-monitor.git
```

Hopefully, the code should be easy to understand.

If you want to add some more visualisations then please open up a pull request and contribute some code.

And if you'd like to buy me a coffee...

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/Z8Z734F5Y)

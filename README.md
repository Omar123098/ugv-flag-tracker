# UGV Flag Tracker

A low-level computer vision algorithm written in **C++** that runs entirely on an **ESP32-CAM**. It finds a red, green or blue flag, tells whether it is left, right or centered, and sends the result to the car's controller over UART.

No OpenCV, no ML model, no laptop. Just raw pixels, bit manipulation and integer math on a microcontroller.

```
ESP32-CAM  ──UART──►  Arduino (car controller)  ──►  motors
```

## How it works

1. **Read raw pixels.** The camera gives a 160 x 120 frame in RGB565. Each 16-bit pixel is unpacked by hand into 8-bit R, G, B with bit shifts and masks.
2. **Classify.** Each pixel is converted to HSV with integer math. Dark, gray or white pixels are ignored; the rest are labeled red, green or blue by hue.
3. **Filter noise.** A pixel only counts if its neighbor has the same color, so scattered noise is dropped and only solid areas (flags) remain.
4. **Vote.** Every color counts its pixels. The color with the most votes wins if it passes a minimum (`MIN_VOTES 200`).
5. **Locate.** The average x position of the winning color is compared to the screen center. The screen is split into three equal parts: left, center, right.
6. **Send.** One message per frame goes to the controller.

## Message format

```
<COLOR,DIR,LOCK>      e.g.  <RED,L,true>   <BLUE,C,false>   <NONE,N,true>
```

- `COLOR`: `RED`, `GREEN`, `BLUE` or `NONE`
- `DIR`: `L` left, `R` right, `C` center, `N` no flag
- `LOCK`: `true` = stop / spin to search, `false` = flag centered, drive forward

UART: 9600 baud, 8N1, CAM GPIO14 → controller RX, common GND.

## Test mode

With `TEST_MODE 1`, the board serves a live view at `http://<ip>/` showing detected pixels in color, the center zone, the flag position and the LOCKED / FREE state. With `TEST_MODE 0` (competition) it only sends UART messages.

## Build

Uses [PlatformIO](https://platformio.org/).

```bash
cp include/secrets.h.example include/secrets.h   # add your WiFi for test mode
pio run -t upload
```

## What I did

- **Low-level programming and control (my main strength).** Worked directly with the hardware: unpacking RGB565 pixels bit by bit, writing straight into the frame buffer to draw the debug view, configuring the camera sensor registers (exposure, saturation, gain), and driving the UART link that controls the car.
- **Built the vision algorithm from scratch in C++.** Pixel classification, noise filtering, color voting and position tracking, all with integer math so it runs fast on the ESP32.
- **Designed the control protocol** between the camera and the car so each frame turns into a simple drive command.
- **Made it work in real lighting.** Fixed flags looking white by tuning the camera and the color thresholds.

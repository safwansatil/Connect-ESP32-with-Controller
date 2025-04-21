# Connect-ESP32-with-Controller
Code to control your ESP32 or Robot via Dualshock or DualSense controllers.
<br>


# SoccerBot Controller – PS5 + ESP32

This is a lightweight and smooth controller setup made for our SoccerBot project during a uni competition.  
We used a PS5 DualSense controller, but it also works well with DualShock 3 and 4.  
The bot is controlled wirelessly via an ESP32—works great, low latency, and honestly just feels good to drive.

## About the Code

The base template for the controller logic was adapted from [racheldebarros.com](https://racheldebarros.com), so big shoutout there.  
We made a few tweaks to tailor it for ESP32-based bots and to improve responsiveness for competitive use.

## What It Works With

- ESP32 (tested with basic Bluetooth and serial comms)
- Controllers:
  - DualShock 3
  - DualShock 4
  - DualSense (PS5)
- Should be fine with any bot that reads commands via serial, UART, or Bluetooth

## Features

- Smooth analog stick control
- Low input latency
- Customizable button mappings
- Should also work on most Linux systems 

## Why We Made It

We built this for our SoccerBot competition at uni. The idea was to have precise, responsive movement with as little delay as possible—especially during fast-paced matches.  
Turns out, using a gaming controller is way more intuitive than DIY remotes. You can control your robot made with ESP-32 with it

## Getting Started
Make sure you downloaded the libraries properly. (Bluepad32 and arduino.h)
Make sure your controller is paired with your system (Bluetooth preferred).  
Flash your ESP32 with the receiving code, then just run the script.

If you have any troubles or have any suggestions for me, please reach out.



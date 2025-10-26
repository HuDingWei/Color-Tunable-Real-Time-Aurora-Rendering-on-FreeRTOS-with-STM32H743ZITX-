# Aurora RGB Rendering on STM32

## Overview  
This project implements an **Aurora Light Rendering System** on an **STM32-based custom development board**.  
It controls RGB LEDs to simulate smooth, color-shifting **northern light effects** using color interpolation and timing-based animation.  
The system also supports **interactive control** — users can adjust RGB channels in real time via a joystick,  
and the light behavior dynamically responds to sensor inputs for adaptive color transitions.  

The main goal of this work is to explore **embedded lighting control**, **color blending algorithms**,  
and **efficient animation rendering** on resource-limited systems.

> **Note:**  
> This repository contains only the **software logic** (color computation and rendering).  
> Hardware-specific configuration files, pin definitions, and BSPs are **not included**,  
> as the project uses a **custom university board**.

---

## Features
- Aurora-style RGB gradient animation  
- Adjustable brightness and transition rate  
- **Joystick control** for real-time RGB single-channel color tuning  
- **Sensor-driven color variation** — LED effects change dynamically based on detected sensor values  
- **Preset mode storage** for predefined animation or lighting settings  
- Modular and easy-to-extend code structure  
- Designed for STM32 microcontrollers (C/C++) 

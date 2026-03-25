回到 [中文README.md](./README.md)

![轨道机器人整体图](./03-docs/Images/overview.png)

## Table of Contents

- [Mechanical Design Notes](#mechanical-design-notes)
  - [Rotation Module](#rotation-module)
  - [Translation Module](#translation-module)
  - [Armor Plate Layout](#armor-plate-layout)
  - [BOM](#bom)
- [Firmware Notes](#firmware-notes)
- [Limitations \& Improvements](#limitations--improvements)
  - [Design Phase](#design-phase)
  - [Manufacturing Phase](#manufacturing-phase)
  - [Assembly Phase](#assembly-phase)
  - [Debugging Phase](#debugging-phase)

# Mechanical Design Notes

Track-based training robot that reproduces RoboMaster armor-plate motion for vision auto-aim training.

![Real-world photo](./03-docs/Images/实物图.jpg)

## System Overview
The system has two motion modules:
- **Rotation module**: spins the armor plates.
- **Translation module**: moves along the track.

## Why Open Source

1. To mark the completion of my first large RM project.
2. To share notes and pitfalls for others.

Thanks to Bihua Wen (track openings and joints) and Lan Feng (code modifications base on 2024 Infantry).

## Repository Structure & Quick Start

- Hardware / mechanical files: `01-hardware/`
	- `移动部分总装.step`, `总装.step`, `总装.f3z`, `BOM.csv`
- Firmware project: `02-firmware/`
	- Project/build notes: [02-firmware/README.md](./02-firmware/README.md)
	- Key control file: [`Robo_Chassis.c`](./02-firmware/SEML/App/Robo/Robo_Chassis.c)
- Docs and images: `03-docs/`
	- Overview image: `03-docs/Images/overview.png`

---

## Requirements Engineering

### Design Goals

Design a track robot that emulates the spatial motion trajectory of the four armor plates on a RoboMaster robot:

1. Stay as close to the ground as possible to leave more adjustable space above.
2. Keep the rotation module as light as possible; choose a reasonable gear ratio to ensure safety while reaching higher rotation speeds.
3. Keep the translation module light overall to improve translation responsiveness.
4. Separate the battery and control module from the moving parts to improve safety.
5. Control overall cost.

### Technical Specs

| Parameter | Value | Notes |
|---|---:|---|
| Linear speed range | 2 m/s | Not safely measurable |
| Angular speed range | 6 rad/s | Not safely measurable |
| Number of armor plates | 4 | Adjustable distribution |

In practice, the track is too short: on a 2 m track, the robot can finish the run in about 1 s and then hit the end frame.

### M3508 Performance Reference

Official DJI RoboMaster page for the M3508 motor: https://www.robomaster.com/zh-CN/products/components/general/M3508

Motor parameter screenshot: [M3508 motor data](./03-docs/Images/3508电机数据.png)

---

## Rotation Module

### Powertrain

Power source: DJI M3508 brushless DC geared motor × 1
Transmission: driven by the M3508; driving : driven = 50 : 130

Reference drawings:

- [Machining - Rotation Driving Gear - 45# Steel - v3.pdf](./03-docs/加工-旋转主动齿-45%23钢-v3.pdf)
- [Machining - Rotation Driven Gear - 6061 Aluminum.pdf](./03-docs/加工-旋转从动齿-6061铝.pdf)

---

## Translation Module

### Powertrain

Power source: DJI M3508 brushless DC geared motor × 2

### Transmission Design

The translation drive gear has 80 teeth per revolution.

Reference drawings:

- [Machining - Translation Drive Pulley - 6061 Aluminum - v3.pdf](./03-docs/加工-平动动力轮-6061铝-v3.pdf)
- [Translation assembly (STEP)](./01-hardware/移动部分总装.step)

It uses an HTD 5M timing-belt transmission. The HTD tooth profile has a larger tooth root height, giving a larger contact area.
Under high-load impacts it can “jump teeth” to provide overload protection and avoid damaging the motor and structural parts.

### Buffer Protection

Hydraulic shock absorbers are installed at both ends. When the robot reaches the end of travel, in addition to controller-side deceleration, the mechanics provide passive deceleration to protect the structure and improve reliability.
Model: ACA 2025-1

| Parameter | Value |
|---|---:|
| Max allowable speed | 4 m/s |
| Max effective weight | 260 kg |
| Max energy absorption | 65 Nm |

---

## Armor Plate Layout

The armor plates are fixed using an aluminum extrusion frame. The spacing and orientation of the four plates can be adjusted by reassembling the aluminum extrusion positions.
No new parts need to be machined, providing layout flexibility and allowing simulation of different RoboMaster robot armor distributions.

---

## BOM

Due to machining mistakes and trial-and-error, the total cost is about 4000 CNY.
See: [BOM](./01-hardware/BOM.csv)

# Firmware Notes

This is modified based on the 2024 Infantry codebase. There is a lot of redundant content; the only file that needs to be modified is the control file [Robo_Chassis.c](./02-firmware/SEML/App/Robo/Robo_Chassis.c).

Note: because the motors are mounted symmetrically, when setting the same linear speed for both motors, the directions must be opposite.

---

# Limitations & Improvements

## Design Phase

### Dual-track design is costly

The current dual-track design has relatively high structural cost. A theoretical improvement is switching to a single track + a friction-wheel approach.
However, the design considered the actual venue conditions: the friction-wheel approach requires a flat floor; if the venue does not meet this requirement, motion stability cannot be guaranteed. Therefore the dual-track solution is kept for now.

### No maintenance hole reserved for the locking bushing

A locking bushing is used to fasten the motor and pulley, but the pulley cover plate fully covers the shaft area. When the locking bushing needs tightening, the cover plate must be removed.

### No tolerance reserved for the pulley width

The pulley thickness equals the 5M belt width, causing excessive friction. It can be alleviated by slightly loosening the cover plate.
Later, this was solved perfectly with a 3D-printed spacer.

### Clarifying design goals

I spent a lot of time on experiments during the design phase but did not end up producing tangible outputs:

1. I planned a general-purpose rotation module configuration so one mechanism could be used to build both a target car and an outpost.
2. I tested a self-made 12-channel conductive slip ring; the brushes and 3D-printed structure were validated, but the work was shelved later due to lack of demand.

### Questioning unreasonable requirements

With a 2 m track, the linear speed is inherently limited. Whether a longer track can still use a timing belt, and whether the belt can be tensioned properly, both require consideration.

## Manufacturing Phase

### Rotation gear machining cost is too high

This was my first time designing and manufacturing parts, and I lacked experience.
The large gear of the rotation gimbal used wire EDM, and the cost per part was close to 1000 CNY.
Future versions should prioritize alternative processes to reduce cost.

## Assembly Phase

### Rivet nuts reduce flatness of part mating surfaces

When using rivet nuts, the mating surfaces are not perfectly flat.
This improved after switching to 3D-printed parts with embedded inserts.
An ideal solution is CNC-tapped embedded inserts, which gives the best fit but increases cost.

### Rail splicing accuracy is insufficient, limiting translation travel

Without precision tools such as a dial indicator, rail splicing and alignment is difficult.
After splicing two 10 cm rails, there was noticeable vibration when the slider crossed the joint.
Therefore, the splicing approach was abandoned in favor of a single-piece rail.
Due to shipping constraints, the maximum rail length is 2 m, which limits the current translation stroke.
How to extend the translation range without relying on splicing is a key topic for future versions.

## Debugging Phase

### M3508 motor suddenly reverses; repeated reversals trigger protection

In a single long test session with repeated reversals, the reversal response becomes noticeably slower than at the beginning. This may be motor protection being triggered.
An improvement direction is to use gentler direction changes ("soft stop, soft start") when not strictly necessary.

# Disclaimer

This project is intended for learning and research purposes only.
The author assumes no responsibility for any hardware damage, personal injury,
or other losses arising from the use of this project.
Please ensure safe operation and compliance with applicable laws and regulations.

This project is not affiliated with DJI (Da-Jiang Innovations) or the official RoboMaster program in any way.

This repository is shared as a reference only and is not intended for direct manufacturing or assembly.

















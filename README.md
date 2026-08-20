# Autonomous Garden Rover

This is an open source outdoor garden robot that can mow grass and water plants for under $400. It features a powerful 3000 watt C6374 motor for mowing grass with electronic height control, and a large Nema17 turret system for watering at target places that supports a normal garden hose. The rover currently uses a 9 axis IMU + wheel encoders for autonomous function. The brain of this rover is a powerful ESP32-P4, which gives this rover the capability to listen and respond to commands using its built in microphone and speaker.

## Inspiration

I spend hours each week gardening and mowing grass. As someone into robotics, I realized I could build a garden rover to automate some of these repetitive chores. This project gave me a reason to learn more about robotics while building something useful for my yard.

[**Watch the demo**](https://youtu.be/P-olpegfmmU) · [**Macondo journal**](https://macondo.hackclub.com/projects/9276) · [**See the assembly**](CAD/)

[![Autonomous Garden Rover demo](https://i.ytimg.com/vi/P-olpegfmmU/maxresdefault.jpg)](https://youtu.be/P-olpegfmmU)

## Features

- Under $400, anyone can build it
- All four wheels are motorized
- C6374 motor offers 3000W+ of power, more than most commercial systems
- NEMA 17 planetary motors lift and raise the mower using a rope system
- The turret uses high-reduction worm and dual-stage planetary drives to direct water
- Water flow can be controlled using a separate ESP32 controller
- Can be controlled from a phone or handheld controller
- Uses a 9-axis IMU and encoders on all four wheels for autonomous operation
- The aluminum chassis bends and acts as a suspension

# Building it yourself

See the [**mechanical, electrical, and firmware build instructions**](INSTRUCTIONS.md) for diagrams and the complete assembly guide.

# wavecatcher

https://www.anfractuosity.com/projects/wavecatcher/ - please see this website for more detail on the project.

The project will be used for acoustic cryptanalysis - "Acoustic cryptanalysis is a type of side channel attack that exploits sounds emitted by computers or other devices".

A number of aspects of computers emit ultrasound including SMPSs to inductors and capacitors present on the computer's motherboard itself.

You will likely require Kicad 5 to use the schematic/pcb layouts.

## Software

The software is now capable of extracting audio from the PDM microphone enabling playing of the audio with SoX.

I now need to swap the simple summing PDM to PCM converter, to enable better fidelity audio.

## Hardware

The PCB was manufactured and assembled using Macrofab.  I plan to eventually revise the PCB at some point
making use of line length matching of USB tracks and more curved PCB tracks.

I conducted a little benchmark of the STM32F3 chip's USB performance and get roughly 6Mbps received from it on my computer. The bandwidth
may vary a fair bit, in the order of megabits depending on your USB controller!

## Creative Commons Licence

Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)  

# Arbalest Rocketry Goose 4.2 Flight Computer Software by Ngoc Thach (Tom) Pham

This repository contains the code in Arduino and C++ for our 2 stages high power rocket to launch at Launch Canada 2025  

## Filter
The Kalman Filter Code was base on the information of "Application of the Kalman Filter to Rocket Apogee Detection" - by David W. Schultz, and the help from Leroy Musa  

## Stages

The rocket model contain 2 stages with 2 seperate flight computer, Stage 1 will be mount on the lower stage of the rocket and Stage 2 will be the upper stage. Each stage will have a different state machine, for stage 1, the state machine will begin from pre-launch and end when the lower stage land; stage 2 will also begin from pre-launch so we can check the mutual reading from stage 1, after seperation, stage 2 will ignite to the second stage apogee and end when it land.  

Stages 1:  
0: Pre-Launch  
1: Launch Detected  
2: First Burnout  
3: First Stage Apogee  
4: Stages Seperate  
5: Deploy Drouge Chute  
6: Deployed Main Chute  
7: Landing Detected  
8: Low Power Mode for recovery  

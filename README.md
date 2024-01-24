# Controlling a Robot with a Camera

This project demonstrates how to controll a robot with a camera by publishing the correct ros commands depending on user pose.

## Table of Contents

- [Project Name](#project-name)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Features](#features)
  - [Getting Started](#getting-started)
  - [Contributing](#contributing)
  - [Report](#report)

## Introduction

The purpouse of this project is to show advantages and disadvantage of using an AI approach to do this task as oppose to using the conventional approach.

## Features

This project will work with most rgb cameras to calculate user pose and thus send commands to a robot.

## Getting Started

The following will need to be installed via pip:
meadiapipe 
tensorflow
opencv
numpy
pandas - only needed for model creation

when installed you can clone the repo and run pose_command.py or model_command.py

## Contributing

Shimon Fiddler at Middlesex university

## Report
The two main approaches addressind in this project are conventional and AI based. The conventional approach is contained in the pose_command.py file. This works by taking an image of the user via the camera (using open cv) calculating the pose (using mediapipe)

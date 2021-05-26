# SocNavAssist


Last updated: May 25th 2021

## Introduction

SocNavAssist is a haptic shared autonomy framework for enabling socially-aware navigation assistance for mobile telepresence robots.

![demo](media/manual_demo.gif)

***
<br>

## Dependencies

* Tested on Ubuntu 18.04 and ROS Melodic
* `pygame_viz.py` depends on Python 3 and `pygame`
* Build and install `libnifalcon` (source files are already included)

<br>

## How to Run

1. To run individual scenarios:
```
roslaunch socially_aware_rvo trial.launch scenario:=<insert-scenario>
```
Currently available scenarios are:

* `approach_human`
* `crossing_human`
* `random_human`
* `approach_human_dense`
* `crossing_human_dense`
* `random_human_dense`

<br>

2. To run scenarios in a batch:
```
cd SocNavAssist/socially_aware_rvo/bash
./trial.sh
```
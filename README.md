# TI Car Cup
IDE Car code repo

## About
Come see live racing of autonomous 1/18-scale cars created by RIT Computer Engineering student teams. For this competition student teams build, program, and race a model car around a track for speed. The winning car is the fastest to complete the track without derailing. To produce these cars on a standard chassis, students blend electrical engineering, computer engineering, and mechanical engineering skills on circuits, electronics, control theories, interfacing, and embedded systems through team work. The teams will be racing during Imagine RIT.

## Authors
Jacob Gualtieri & Zeb Hollinger

## Steering Notes
Find the left and right sides of the track from the smoothed camera data

If the left edge is beyond threshold A from the center, turn left a little
If the left edge is beyond [64], turn left a lot

If the right edge is beyond threshold A from the center, turn right a little
If the right edge is beyond [64] , turn right a lot

These thresholds correspond to indexes in the [128] array

eye project
===========
Functionalities:
----------------
1. Interface for tabletop segmentation using Euclidean clustering (eye_v0.cpp)
2. Interface for same as (1) + primitive conversion (bounding boxes, superquadric, revolution and extrusion surfaces) (eye_v1.cpp)


NOTES
------
When creating the segmented-cloud channel, make it really big, since
the pointclouds segmented are not small.

ach mk -1 -o 666 segmented-cloud -m 10 -n 100000

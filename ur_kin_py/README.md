ur_kin_py
=========

Python wrappers for ur_kinematics

These kinematics find the tranfrom from the base link to the end effector.
Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
offset transforms are specified in this formulation.
To work with the raw D-H kinematics, use the inverses of the transforms below.

Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1

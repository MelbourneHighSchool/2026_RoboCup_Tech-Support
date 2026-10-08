// Separate translation unit prevents setuptools reusing the LIDAR object with
// different preprocessor flags when both extensions are built together.
#define SOCCER_TOF 1
#include "localisation.cpp"

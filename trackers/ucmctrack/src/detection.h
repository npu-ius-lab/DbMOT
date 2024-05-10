#pragma once

#include "dataType.h"

class Detection
{
public:
    double score;
    State2D boundingbox;
    PROJ_RESULT distribution;
    int trackid = 0;
};
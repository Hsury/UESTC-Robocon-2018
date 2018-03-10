#ifndef __MOVE_H
#define __MOVE_H

#include "includes.h"

#define MAX_SPEED    (2.4f)
#define SAFE_DIST    (0.8f)
#define COEF_LINEAR  (2.5f)
#define COEF_ANGULAR (2.5f)

uint32_t UniformPlusP(float GoalX, float GoalY, float GoalZ, uint32_t BlockTime);

#endif

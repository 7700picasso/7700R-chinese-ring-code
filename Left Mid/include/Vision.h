#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

/*vex-vision-config:begin*/
signature MOGO_RED = signature (1, 8017, 9845, 8931, 393, 1153, 773, 5.2, 0);
signature MOGO_BLUE = signature (2, -3139, -2349, -2744, 8913, 13233, 11073, 2.7, 0);
signature MOGO_YELLOW = signature (3, 1117, 2153, 1636, -2853, -2167, -2510, 4.3, 0);
//vision Vision = vision (PORT19, 50, MOGO_RED, MOGO_BLUE, MOGO_YELLOW);
vision VisionBack = vision (PORT12, 50, MOGO_RED, MOGO_BLUE, MOGO_YELLOW);
/*vex-vision-config:end*/
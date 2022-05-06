#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

/*vex-vision-config:begin*/
signature MOGO_RED = signature (1, 6429, 9347, 7888, -711, -173, -442, 3.2, 0);
signature MOGO_BLUE = signature (2, 47, 2271, 1158, -3693, -3251, -3472, 4, 0);
signature MOGO_YELLOW = signature (3, -2497, -1709, -2104, 7937, 10881, 9410, 3, 0);
vision Vision = vision (PORT19, 50, MOGO_RED, MOGO_BLUE, MOGO_YELLOW);
vision VisionBack = vision (PORT12, 50, MOGO_RED, MOGO_BLUE, MOGO_YELLOW);
/*vex-vision-config:end*/
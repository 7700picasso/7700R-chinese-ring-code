/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature MOGO_BLUE = vex::vision::signature (1, -2729, -1817, -2274, 9681, 11995, 10838, 5.5, 0);
vex::vision::signature MOGO_RED = vex::vision::signature (2, 7457, 11117, 9287, -1377, 1, -688, 2.5, 0);
vex::vision::signature MOGO_YELLOW = vex::vision::signature (3, 1533, 2561, 2047, -3341, -2735, -3038, 7.6, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision = vex::vision (vex::PORT19, 50, MOGO_BLUE, MOGO_RED, MOGO_YELLOW, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/
/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature SIG_1 = vex::vision::signature (1, 6429, 9347, 7888, -711, -173, -442, 3.2, 0);
vex::vision::signature SIG_2 = vex::vision::signature (2, 47, 2271, 1158, -3693, -3251, -3472, 4, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, -2497, -1709, -2104, 7937, 10881, 9410, 3, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT19, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/
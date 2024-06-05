#ifndef _ACCURACY_ALGORITHMS_H_
#define _ACCURACY_ALGORITHMS_H_

#include "I2C.h"
#include "macros.h"

#include <math.h>

#define MAX_SENSORS 6

// Data arrays for sensor data is ALWAYS 1st dimension for sensors, 2nd dimension for samples, starting with the earliest first
// E.g. 6 sensors 1 sample per: | A | B | C | D | E | F | = S[6][1]
//
// 4 sensors 3 samples per:     | A | B | C | D | x | x | = S[4][3]
//                              |A+1|B+1|C+1|D+1| x | x |
//                              |A+2|B+2|C+2|D+2| x | x |

#endif // !_ACCURACY_ALGORITHMS_H
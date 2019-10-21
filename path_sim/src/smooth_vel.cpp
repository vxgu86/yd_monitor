#include <iostream>
#include <sstream>
using namespace std;

//smooth velocity
// -Amax < a < Amax
// 0 < v < Vmax
// t = 1/hz = 0.1
// v = v + at
// delat_s = vt = vt + at *t

//constrain v

class yd_smooth
{
private:
    double accel;  //加速度
    double dccel;  //减速度
    double last_v; // 上一次记录的速度

public:
};

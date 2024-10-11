#include "SerialPort.h"
using namespace std;

void Bullet_Velocity_Control()
{
    if (ds.bullet_velocity > 20)
    {
        if (ds.car_id == 7)
        {
            ds.bullet_velocity = 27.5;
        }
        else
        {
            ds.bullet_velocity = 25.5;
        }
    }
} 
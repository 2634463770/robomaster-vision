#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include "SerialPort.h"
#include "Base.h"

class Geometry
{
public:
    // double Oxy_pre, Oxy_now, Oxy_hat;

    // double Xt_;

    double Geometry_Solution(double y1);

    double Geometry_Solution_New(double pnp_yaw, double armor_at, double armor_vt);

    void Angle_Solution(double y1, double y2);
};

#endif // GEOMETRY_H

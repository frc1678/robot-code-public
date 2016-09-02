#ifndef MUAN_UNITS_UNITS_H_
#define MUAN_UNITS_UNITS_H_

namespace muan {

namespace units {

using Time = double;
using Frequency = double;
using Length = double;
using Velocity = double;
using Acceleration = double;
using Mass = double;
using Angle = double;
using AngularVelocity = double;
using Current = double;
using Voltage = double;

// Unitless
constexpr double pi = 3.14159265358;

constexpr Time s = 1;
constexpr Time min = 60 * s;
constexpr Time ms = 1e-3 * s;
constexpr Time us = 1e-6 * s;

constexpr Frequency hz = 1 / s;

constexpr Length m = 1;
constexpr Length cm = 1e-2 * m;
constexpr Length in = 0.0254 * m;
constexpr Length ft = 0.3048 * m;

constexpr Velocity mps = 1 * m / s;
constexpr Velocity fps = 1 * ft / s;

constexpr Acceleration mps2 = 1 * m / s / s;
constexpr Acceleration fps2 = 1 * ft / s / s;

constexpr Mass kg = 1;
constexpr Mass g = 0.001 * kg;
constexpr Mass mg = 0.000001 * kg;

constexpr Angle rad = 1;
constexpr Angle deg = pi * rad / 180;
constexpr Angle rev = 2 * pi * rad;

constexpr AngularVelocity rps = 1 * rad / s;
constexpr AngularVelocity dps = 1 * deg / s;

constexpr Current A = 1;

constexpr Voltage V = kg * m * m / (A * s * s * s);
constexpr Voltage pwm = 12 * V;

constexpr double convert(double val, double unit) { return val / unit; }

}  // units

}  // muan

#endif

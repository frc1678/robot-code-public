#ifndef units
#define units

namespace muan {

namespace units {

// Unitless
constexpr double pi = 3.14159265358;

// Time
constexpr double s = 1 * s;
constexpr double min = 60 * s;
constexpr double ms = 1e-3 * s;
constexpr double us = 1e-6 * s;

// Lengths
constexpr double m = 1;
constexpr double cm = 1e-2 * m;
constexpr double in = 2.54 * cm;
constexpr double ft = 12 * in;

// Mass
constexpr double kg = 1;
constexpr double g = 0.001 * kg;
constexpr double mg = 0.001 * g;

// Angle
constexpr double rad = 1;
constexpr double deg = 1.74532925e-2 * rad;
constexpr double rev = 2 * pi * rad;

// Angular Velocity
constexpr double rpm = rev / min;

// Current
constexpr double A = 1;

// Voltage
constexpr double V = kg * m * m / (A * s * s * s);
constexpr double pwm = 12 * V;

}  // units

}  // muan

#endif

#ifndef units
#define units

namespace muan {

namespace units {

using Seconds = double;
using Meters = double;
using MetersPerSecond = double;
using Kilograms = double;
using Radians = double;
using RadiansPerSecond = double;
using Amperes = double;
using Volts = double;

// Unitless
constexpr double pi = 3.14159265358;

constexpr Seconds s = 1;
constexpr Seconds min = 60 * s;
constexpr Seconds ms = 1e-3 * s;
constexpr Seconds us = 1e-6 * s;

constexpr Meters m = 1;
constexpr Meters cm = 1e-2 * m;
constexpr Meters in = 0.0254 * m;
constexpr Meters ft = 0.3048 * m;

constexpr MetersPerSecond mps = 1 * m / s;
constexpr MetersPerSecond fps = 1 * ft / s;

constexpr Kilograms kg = 1;
constexpr Kilograms g = 0.001 * kg;
constexpr Kilograms mg = 0.000001 * kg;

constexpr Radians rad = 1;
constexpr Radians deg = 1.74532925e-2 * rad;
constexpr Radians rev = 2 * pi * rad;

constexpr RadiansPerSecond rps = 1 * rad / s;
constexpr RadiansPerSecond dps = 1 * deg / s;

constexpr Amperes A = 1;

constexpr Volts V = kg * m * m / (A * s * s * s);
constexpr Volts pwm = 12 * V;

double convert(double val, double unit) { return val / unit; }

}  // units

}  // muan

#endif

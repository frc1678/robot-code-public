'''
Generate a C++ include file that can be used for type-safe numerical
computation with units.  See the documentation file unitscpp.pdf.
 
This script is part of the unitscpp project at 
http://code.google.com/p/unitscpp/
 
---------------------------------------------------------------------------
Copyright (C) 2008 Don Peterson
Contact:  gmail.com@someonesdad1
 
Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
 
Except as contained in this notice, the name(s) of the above copyright
holders shall not be used in advertising or otherwise to promote the sale,
use or other dealings in this Software without prior written authorization.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
'''

import sys, getopt, os
from string import join

out = sys.stdout.write
err = sys.stderr.write

# Keys and map used to store strings used in output
Units       = "Units"
fundamental = "Fundamental"
derived     = "Derived"
header      = "header"
trailer     = "trailer"
constant    = "Constant"
constants   = "constants"
typedefs    = "typedefs"
typedefs_derived = "typedefs_derived"
needed_includes  = "needed_includes"
parameters  = "Parameters"
outstream   = "OUTSTREAM"
f_unit      = "Unit"
d_unit      = "DerivedUnit"
include_file_default_name = "IncludeDefaultFileName"
include_file_name = "IncludeFileName"
units_class       = "UnitsClass"
units_namespace   = "UnitsNamespace"
numerical_type    = "NumericalType"
class_identifier  = "class_identifier"

output_strings = {
    Units       : {},
    fundamental : [],
    derived     : [],
    constant    : [],
    header      : "",
    trailer     : "",
    include_file_default_name : "unitscpp.h",
    include_file_name : "",
    units_class : "Units",
    units_namespace: "",
    numerical_type: "double",
    class_identifier: "class",  # Gets around bug in ctags 5.7
    needed_includes: '''
#include <ostream>
#include <sstream>
#include <string>
'''

}

# Other string constants
header_begin = "HeaderBegin"
header_end = "HeaderEnd"
trailer_begin = "TrailerBegin"
trailer_end = "TrailerEnd"
unit_marker_begin = "\"<\" "
unit_marker_end   = "\">\""
indent = "    "
nl = "\n"

# Settings from command line

manpage = '''Usage:  %(name)s [options] [configuration_file]

  Writes an include file that can be used for type-safe numerical
  computations with units.  The configuration_file controls the contents
  of the include file, including such things as:
        * The fundamental number of dimensions
        * The names of each dimension (e.g., Length, Mass, etc.)
        * The symbols to be used for the constants
  Note that each unit winds up being a C++ type; the compiler won't let
  you mix them except in specified ways.  This will help you make your
  numerical calculations type-safe.

  Options

    -e      Causes an empty configuration file to be written to stdout.
            This will contain comments on how to construct the configuration
            file.

    -s      Same as -e, but includes the 7 fundamental SI units to be 
            added to the output along with common derived units and
            constants.  Feel free to edit this script to customize the
            output for your own needs.
'''

configuration_file_contents = '''# Empty configuration file for C++ units include file generation.  The
# method is based on the ideas in Barton and Nackman, "Scientific and
# Engineering C++", Addison-Wesley, 1994, ISBN 0201533936.  Also see
# W. Brown, "Applied Template Metaprogramming in SIunits:  the Library
# of Unit-Based Computation", 2001 (you'll have to search the web for
# this reference).
#
# The tokens that you specify in this file must be legitimate C++
# identifiers; otherwise, your code won't compile.  Note that the
# program that generates the units include file does no checking of
# the names you use.
#
#----------------------------------------------------------------------
# Header text (these lines are optional)
#
# All of the text between HeaderBegin and HeaderEnd will be inserted
# unchanged into the include file.  Use this for arbitrary preprocessor 
# code and C++ code.  Example:
#
#   HeaderBegin
#       #include "boost/rational.hpp"
#       #ifdef DEBUG
#       #include "my_debug.h"
#       #endif
#   HeaderEnd
#
# Note:  if you use a namespace via UnitsNamespace (see below), your
# code added here will not be in the namespace (contrast this to the
# Trailer section).

HeaderBegin
HeaderEnd


#----------------------------------------------------------------------
# Basic settings (these lines are optional)
#
# IncludeFileName is the name of the include file that will be
#   created.  The program _will_ overwrite an existing file.  If you
#   do not specify a name, the include file information will be
#   written to stdout.
# UnitsClass is the name of the templated class used for the Units
#   type.  Defaults to 'Units'.
# UnitsNamespace is the namespace that all the code will be under.
#   If this is the empty string, no namespace will be generated.
# NumericalType is the type you plan to use for numerical
#   calculations.  Defaults to 'double'.  You may use any type
#   that has the appropriate semantics.

IncludeFileName     =
UnitsClass          = Units
UnitsNamespace      =
NumericalType       = double

#----------------------------------------------------------------------
# Fundamental dimensions (these lines are required)
#
# This section determines the number and names of the fundamental
# unit dimensions.  Here's an example line that specifies length as a 
# fundamental unit:
#
#       Unit = Length
#
# 'Unit' is required, followed by an '=' character, then the type of
# the fundamental unit.
#
# This fundamental unit specification will create a type called Length
# with which you can declare variables.  You'll want to include in the
# Constants section below a line such as 
#
#       Constant const Length m = 1;
#
# to define the numerical magnitude of the fundamental base unit
# (here, the meter).
#
# This then lets you write variable definitions such as 
#
#   Length diameter = 2.3*m;
#
# In the Constants section below, you could also write lines such as
#
#   Constant const Length meter = 1;
#   Constant const Length metre = 1;
#
# which are effectively aliases of the fundamental base unit symbol.
# Thus, you don't have to worry about only having one symbol for a
# particular unit.
#
# You need at least one line containing a Unit statement.
#
# If you invoked the program with the -s option, you'll have the seven
# fundamental SI units defined for you.  This is a convenience so you
# don't have to define them here (but they'll show up in the include
# file). 

%(fundamental)s

#----------------------------------------------------------------------
# Derived units (these lines are optional)
#
# Similar to the fundamental units section, this section lets you
# define derived units.  You must write the derived unit in terms of
# the following items:
#       Any fundamental unit
#       Any derived unit defined on a previous line
#       Using the binary operators of * or /
#       Parentheses
#       Raising to an integer power with ** or ^
#
# You may also write expressions such as '1/Length'.  Note this won't
# work for any other integer or floating point number.
#
# Examples:
#   DerivedUnit = Velocity = Length/Time
#   DerivedUnit = Acceleration = Velocity/Time
#   DerivedUnit = Force = Mass*Acceleration
#   DerivedUnit = Energy = Force*Length
#
# The following are equivalent:
#   DerivedUnit = Area = 1/(Length*Length)
#   DerivedUnit = Area = 1/Length^2
#   DerivedUnit = Area = 1/Length**2
#   DerivedUnit = Area = Length**(-2)
#
# Note:  Length**0 is the same as 1.

%(derived)s

#----------------------------------------------------------------------
# Constants (these lines are optional)
#
# These lines specify verbatim code that will be inserted into the
# include file.  Here's an example that defines millimeters when the
# base unit of m has been defined:
#
#   Constant = const Length mm = m/1000;
#
# While you could add this verbatim code to the trailer section, using
# the Constant keyword documents things better and lets you e.g. grep
# for all your defined constants.
#
# You may omit the keyword const, which will mean you'll have a global
# variable whose value may be changed later.

%(constants)s

#----------------------------------------------------------------------
# Trailer text (these lines are optional)
#
# All of the text between TrailerBegin and TrailerEnd will be inserted
# unchanged into the include file.  Use this for arbitrary code you
# want in the include file.  
#
# Note if you define a namespace via UnitsNamespace, your code added
# here _will_ be in the namespace.

TrailerBegin
TrailerEnd
'''

# The following strings are used when the -s option is given.
# Customize them to your own needs.
s_fundamental = '''
Unit = Mass
Unit = Length
Unit = Time
Unit = Current
Unit = Temperature
Unit = Quantity
Unit = LuminousIntensity
Unit = Angle
Unit = SolidAngle
'''

s_derived = '''
DerivedUnit = Dimensionless = 1
DerivedUnit = Avogadro = 1/Quantity
DerivedUnit = Area = Length^2
DerivedUnit = Volume = Length^3
DerivedUnit = FlowRate = Length^3/Time
DerivedUnit = Density = Mass/Volume
DerivedUnit = Velocity = Length/Time
DerivedUnit = AngularVelocity = Angle/Time
DerivedUnit = Acceleration = Length/Time^2
DerivedUnit = AngularAcceleration = Angle/Time^2
DerivedUnit = Force = Mass*Acceleration
DerivedUnit = Weight = Force
DerivedUnit = Pressure = Force/Area
DerivedUnit = Stress = Pressure
DerivedUnit = Strain = Dimensionless
DerivedUnit = Wavelength = Length
DerivedUnit = WaveNumber = 1/Length
DerivedUnit = Frequency = 1/Time
DerivedUnit = KinematicViscosity = Area/Time
DerivedUnit = DynamicViscosity = Force*Time/Area
DerivedUnit = Energy = Force*Length
DerivedUnit = Work = Energy
DerivedUnit = Power = Energy/Time
DerivedUnit = GravitationalConstant = Force*Length^2/Mass^2
DerivedUnit = Action = Energy*Time

DerivedUnit = SpecificVolume = Volume/Mass
DerivedUnit = SpecificEnergy = Energy/Mass
DerivedUnit = SpecificHeatCapacity = Energy/(Mass*Temperature)
DerivedUnit = ThermalConductivity = Power/(Length*Temperature)
DerivedUnit = Entropy = Energy/Temperature

DerivedUnit = Charge = Current*Time
DerivedUnit = Voltage = Power/Current
DerivedUnit = ElectricFieldStrength = Voltage/Length
DerivedUnit = Resistance = Voltage/Current
DerivedUnit = Conductance = 1/Resistance
DerivedUnit = Capacitance = Current*Time/Voltage
DerivedUnit = MagneticFlux = Voltage*Time
DerivedUnit = MagneticFluxDensity = MagneticFlux/Area
DerivedUnit = MagneticFieldStrength = Current/Length
DerivedUnit = MagnetomotiveForce = Current
DerivedUnit = Inductance = MagneticFlux/Current
DerivedUnit = Permittivity = Capacitance/Length
DerivedUnit = Permeability = Force/Current^2
DerivedUnit = GasConstant = Energy/(Quantity*Temperature)
DerivedUnit = StefanBoltzmannLaw = Power/(Area*Temperature^4)
DerivedUnit = MagneticMoment = Energy/MagneticFluxDensity
DerivedUnit = ElectrochemicalConstant = Charge/Quantity
DerivedUnit = ChargeToMass = Charge/Mass

DerivedUnit = LuminousFlux = LuminousIntensity*SolidAngle
DerivedUnit = Luminance = LuminousIntensity/Area
DerivedUnit = Illuminance = LuminousFlux/Area
DerivedUnit = RadiantIntensity = Power/SolidAngle
'''

s_constants = '''
Constant = const NT tera = 1e12;
Constant = const NT giga = 1e9;
Constant = const NT mega = 1e6;
Constant = const NT kilo = 1e3;
Constant = const NT deci = 1e-1;
Constant = const NT centi = 1e-2;
Constant = const NT milli = 1e-3;
Constant = const NT micro = 1e-6;
Constant = const NT nano = 1e-9;
Constant = const NT pico = 1e-12;
Constant = const NT femto = 1e-15;
Constant = const NT atto = 1e-18;

Constant = const Mass kg = 1;
Constant = const Length m = 1;
Constant = const Time s = 1;
Constant = const Frequency Hz = 1/s;
Constant = const Current A = 1;
Constant = const Temperature K = 1;
Constant = const Quantity mol = 1;
Constant = const LuminousIntensity cd = 1;
Constant = const Angle radian = 1;
Constant = const Angle rad = 1;
Constant = const SolidAngle steradian = 1;
Constant = const SolidAngle sr = 1;
Constant = const Force N = 1*kg*m/(s*s);
Constant = const Pressure Pa = 1*N/(m*m);
Constant = const Energy J = 1*N*m;
Constant = const Power W = 1*J/s;
Constant = const Charge C = 1*A*s;
Constant = const Voltage V = 1*J/C;
Constant = const Resistance ohm = 1*V/A;
Constant = const Conductance S = 1/ohm;
Constant = const Capacitance F = 1*C/V;
Constant = const MagneticFlux Wb = 1*V*s;
Constant = const MagneticFluxDensity T = 1*Wb/(m*m);
Constant = const Inductance H = 1*Wb/A;

Constant = const Mass g = 1e-3*kg;
Constant = const Mass mg = milli*g;
Constant = const Mass ug = micro*g;
Constant = const Mass ng = nano*g;
Constant = const Mass lbm = 0.45359237*kg;
Constant = const Mass amu = 1.66053873e-27*kg;
Constant = const Mass ton = 2000*lbm;
Constant = const Mass tonne = 1000*kg;

Constant = const Length km = kilo*m;
Constant = const Length cm = centi*m;
Constant = const Length mm = milli*m;
Constant = const Length um = micro*m;
Constant = const Length nm = nano*m;
Constant = const Length Angstrom = 0.1*nm;
Constant = const Length in = 25.4*mm;
Constant = const Length inch = in;
Constant = const Length mil = milli*in;
Constant = const Length ft = 12*in;
Constant = const Length feet = ft;
Constant = const Length foot = ft;
Constant = const Length yd = 3*ft;
Constant = const Length mi = 5280*ft;
Constant = const Length mile = mi;
Constant = const Length nauticalmile = 1852*m;
Constant = const Length lightyear = 9.4607305e+15*m;
Constant = const Length au = 1.4959787e+11*m;

Constant = const Time ms = 1e-3*s;
Constant = const Time us = 1e-6*s;
Constant = const Time ns = 1e-9*s;
Constant = const Time minute = 60*s;
Constant = const Time hr = 3600*s;
Constant = const Time hour = hr;
Constant = const Time day = 24*hr;
Constant = const Time week = 7*day;
Constant = const Time year = 365.2422*day;
Constant = const Time month = 30.43685*day;

Constant = const Temperature degC = 1*K;
Constant = const Temperature degF = 5/9*K;

Constant = const Angle mradian = 1e-3*radian;
Constant = const Angle uradian = 1e-6*radian;
Constant = const Angle deg = 1.745329251994330e-02*radian;
Constant = const Angle degree = 1*deg;
Constant = const Angle degrees = 1*degree;

Constant = const Area m2 = 1*m*m;
Constant = const Area mm2 = 1*mm*mm;
Constant = const Area cm2 = 1*cm*cm;
Constant = const Area in2 = 1*in*in;
Constant = const Area ft2 = 1*ft*ft;
Constant = const Area hectare = 1e4*m2;
Constant = const Area acre = 4046.8726*m2;
Constant = const Area barn = 1e-28*m2;

Constant = const Volume m3 = 1*m*m2;
Constant = const Volume cm3 = 1*cm*cm2;
Constant = const Volume cc = 1*cm3;
Constant = const Volume mm3 = 1*mm*mm2;
Constant = const Volume in3 = 1*in*in2;
Constant = const Volume ft3 = 1*ft*ft2;
Constant = const Volume l = milli*m3;
Constant = const Volume liter = milli*m3;
Constant = const Volume ml = milli*liter;
Constant = const Volume ul = micro*liter;
Constant = const Volume gal = 0.0037854118*m3;
Constant = const Volume gallon = 1*gal;
Constant = const Volume quart = gallon/4;

Constant = const Velocity mph = mile/hour;
Constant = const Velocity knot = nauticalmile/hour;
Constant = const Velocity fps = foot/s;

Constant = const Frequency kHz = kilo*Hz;
Constant = const Frequency MHz = mega*Hz;
Constant = const Frequency GHz = giga*Hz;
Constant = const Frequency rpm = 1/minute;

Constant = const FlowRate gps = gal/s;
Constant = const FlowRate gpm = gal/minute;
Constant = const FlowRate gph = gal/hour;
Constant = const FlowRate cfs = ft3/s;
Constant = const FlowRate cfm = ft3/minute;
Constant = const FlowRate lps = l/s;
Constant = const FlowRate lpm = l/minute;

Constant = const Force lbf = 4.4482216*N;
Constant = const Force dyne = 10*micro*N;

Constant = const Pressure MPa = mega*Pa;
Constant = const Pressure GPa = giga*Pa;
Constant = const Pressure atm = 101325*Pa;
Constant = const Pressure mmHg = 133.32239*Pa;
Constant = const Pressure inHg = 3386.3886*Pa;
Constant = const Pressure psi = lbf/in2;
Constant = const Pressure kpsi = kilo*lbf/in2;
Constant = const Pressure Mpsi = mega*lbf/in2;
Constant = const Pressure psf = lbf/ft2;
Constant = const Pressure bar = 1e5*Pa;
Constant = const Pressure torr = 133.322368*Pa;
Constant = const Pressure inH2O = 249.08891*Pa;
Constant = const Pressure ftH2O = 12*inH2O;

Constant = const Energy erg = 1*dyne*cm;
Constant = const Energy cal = 4.1868*J;
Constant = const Energy kcal = kilo*cal;
Constant = const Energy eV = 1.6021765e-19*J;
Constant = const Energy keV = kilo*eV;
Constant = const Energy MeV = mega*eV;
Constant = const Energy GeV = giga*eV;
Constant = const Energy btu = 1055.0559*J;

Constant = const Current MA = mega*A;
Constant = const Current kA = kilo*A;
Constant = const Current mA = milli*A;
Constant = const Current uA = micro*A;
Constant = const Current nA = nano*A;
Constant = const Current pA = pico*A;

Constant = const Capacitance pF = pico*F;
Constant = const Capacitance nF = nano*F;
Constant = const Capacitance uF = micro*F;
Constant = const Capacitance mF = milli*F;

Constant = const Inductance mH = milli*H;
Constant = const Inductance uH = micro*H;

Constant = const MagneticFluxDensity gauss = 1e-4*T;

Constant = const Acceleration AccelerationOfGravity = 9.80665*m/(s*s);
Constant = const GravitationalConstant GravitationConstant = 6.67428e-11*N*m2/(kg*kg);
Constant = const Density DensityOfWater = 1*g/cc;
Constant = const Velocity SpeedOfLight = 2.9979246e8*m/s;
Constant = const Velocity SpeedOfSound = 331.46*m/s;
Constant = const Avogadro AvogadroNumber = 6.02214179e23/mol;
Constant = const Entropy BoltzmannConstant = 1.3806504e-23*J/K;
Constant = const GasConstant MolarGasConstant = 8.314472*J/(mol*K);
Constant = const Action PlanckConstant = 6.62606896e-34*J*s;
Constant = const Permittivity PermittivityOfVacuum = 8.854187817e-12*F/m;
Constant = const Permeability PermeabilityOfVacuum = 12.566370614e-7*N/(A*A);
Constant = const StefanBoltzmannLaw StefanBoltzmannConstant = 5.670400e-8*W/(m2*K*K*K*K);
Constant = const MagneticMoment BohrMagneton = 927.400915e-26*J/T;
Constant = const ElectrochemicalConstant FaradayConstant = 96485.3399*C/mol;
Constant = const Mass ElectronMass = 5.485799110e-4*amu;
Constant = const Charge ElectronCharge = 1.602176487e-19*C;
Constant = const Mass ProtonMass = 1.00727646688*amu;
Constant = const Mass NeutronMass = 1.00866491578*amu;
Constant = const Mass AlphaParticleMass = 4.0015061747*amu;
Constant = const Mass EarthMass = 5.9742e24*kg;
Constant = const Length EarthRadius = 6378136.49*m;
Constant = const Mass SunMass = 1.9891e30*kg;
Constant = const Length SunRadius = 6.96e8*m;
Constant = const Mass MoonMass = 7.3483e22*kg;
Constant = const Length MoonRadius = 1738*km;
Constant = const Length MoonDistance = 3.844e8*m;
'''

include_file = '''
typedef %(NumericalType)s NT;   // Number Type

template<%(IPAR)s>
%(class_identifier)s %(UnitsClass)s
{
    public:
        %(UnitsClass)s(NT value_=NT(0)) : value(value_) {}

        // This turns the class into a function object that allows
        // the user to easily get at the value.
        NT operator()() const { return value; }

        // Helper function to get a text representation of the
        // object's dimensions.  It is static because the
        // representation is known at compile time.
        static std::string dim(void)
        {
            std::stringstream s;
            s << %(OUTSTREAM)s;
            return s.str();
        }

        // Helper function for unit conversions.
        NT to(const %(UnitsClass)s & u) const
        {
            return value/u.value;
        }

        %(UnitsClass)s & operator=(const %(UnitsClass)s & rhs)
        {
            value = rhs.value;
            return *this;
        }

        // Arithmetic operators
        %(UnitsClass)s & operator+=(const %(UnitsClass)s & rhs)
        {
            value += rhs.value;
            return *this;
        }

        %(UnitsClass)s & operator-=(const %(UnitsClass)s & rhs)
        {
            value -= rhs.value;
            return *this;
        }

        Units & operator*=(const NT & rhs)
        {
            value *= rhs;
            return *this;
        }

        Units & operator/=(const NT & rhs)
        {
            value /= rhs;
            return *this;
        }

    private:
        NT value;
};


// Addition
template <%(IPAR)s>
const %(UNIT)s operator+(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return %(UNIT)s(lhs() + rhs());
}


// Subtraction
template <%(IPAR)s>
const %(UNIT)s operator-(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return %(UNIT)s(lhs() - rhs());
}


// Negation
template <%(IPAR)s>
const %(UNIT)s operator-(const %(UNIT)s & rhs)
{
    return %(UNIT)s(-rhs());
}


// Multiplication
template <%(IPAR)s>
const %(UNIT)s operator*(const NT & lhs, const %(UNIT)s & rhs)
{
    return %(UNIT)s(lhs*rhs());
}

template <%(IPAR)s>
const %(UNIT)s operator*(const %(UNIT)s & lhs, const NT & rhs)
{
    return rhs*lhs;
}

template <%(IPARmul)s>
const %(UnitsClass)s<%(PARaPb)s> operator*(const %(UnitsClass)s<%(PARa)s> & lhs, const %(UnitsClass)s<%(PARb)s> & rhs)
{
    return %(UnitsClass)s<%(PARaPb)s>(lhs()*rhs());
}


// Division
template <%(IPAR)s>
const %(UNIT)s operator/(const %(UNIT)s & lhs, const NT & rhs)
{
    return %(UNIT)s(lhs()/rhs);
}

template <%(IPAR)s>
const %(mUNIT)s operator/(const NT & lhs, const %(UNIT)s & rhs)
{
    return %(mUNIT)s(lhs/rhs());
}

template <%(IPARmul)s>
const %(UnitsClass)s<%(PARaMb)s> operator/(const %(UnitsClass)s<%(PARa)s> & lhs, const %(UnitsClass)s<%(PARb)s> & rhs)
{
    return %(UnitsClass)s<%(PARaMb)s>(lhs()/rhs());
}


// Comparisons
template <%(IPAR)s>
bool operator==(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return (lhs() == rhs());
}

template <%(IPAR)s>
bool operator==(const %(UNIT)s & lhs, const NT & rhs)
{
    return (lhs() == rhs);
}

template <%(IPAR)s>
bool operator==(const NT & lhs, const %(UNIT)s & rhs)
{
    return (lhs == rhs());
}

template <%(IPAR)s>
bool operator!=(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return not (lhs() == rhs());
}

template <%(IPAR)s>
bool operator!=(const %(UNIT)s & lhs, const NT & rhs)
{
    return not (lhs() == rhs);
}

template <%(IPAR)s>
bool operator!=(const NT & lhs, const %(UNIT)s & rhs)
{
    return not (lhs == rhs());
}


// Ordering
template <%(IPAR)s>
bool operator<=(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return lhs() <= rhs();
}

template <%(IPAR)s>
bool operator<=(const %(UNIT)s & lhs, const NT & rhs)
{
    return (lhs() <= rhs);
}

template <%(IPAR)s>
bool operator<=(const NT & lhs, const %(UNIT)s & rhs)
{
    return (lhs <= rhs());
}


template <%(IPAR)s>
bool operator>=(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return lhs() >= rhs();
}

template <%(IPAR)s>
bool operator>=(const %(UNIT)s & lhs, const NT & rhs)
{
    return (lhs() >= rhs);
}

template <%(IPAR)s>
bool operator>=(const NT & lhs, const %(UNIT)s & rhs)
{
    return (lhs >= rhs());
}


template <%(IPAR)s>
bool operator<(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return lhs() < rhs();
}

template <%(IPAR)s>
bool operator<(const %(UNIT)s & lhs, const NT & rhs)
{
    return (lhs() < rhs);
}

template <%(IPAR)s>
bool operator<(const NT & lhs, const %(UNIT)s & rhs)
{
    return (lhs < rhs());
}


template <%(IPAR)s>
bool operator>(const %(UNIT)s & lhs, const %(UNIT)s & rhs)
{
    return lhs() > rhs();
}

template <%(IPAR)s>
bool operator>(const %(UNIT)s & lhs, const NT & rhs)
{
    return (lhs() > rhs);
}

template <%(IPAR)s>
bool operator>(const NT & lhs, const %(UNIT)s & rhs)
{
    return (lhs > rhs());
}

template <%(IPAR)s>
std::ostream & operator<<(std::ostream & s, const %(UNIT)s & rhs)
{
    return s << rhs();
}

// operator>> is not provided because the unit type can not be
// created at runtime in any reasonable fashion.  This means there is
// no easy way to serialize unit objects.
//
// If you need to read in an object from a stream, read it into an NT
// variable and put it into an appropriate-type variable.  Example:
//
//      NT x;
//      cin >> x;
//      Length y = x*m;
//
// where the base unit m has already been defined.  This requires you
// to i) know the unit type at compile time and ii) assume its value
// is in terms of the base type.
'''

class U:
    '''Perform dimension arithmetic for arbitrary units.  This class is
    used to calculate the fundamental unit representation for derived units.
    '''
    def __init__(me, d):
        me.d = tuple(d)
    def __repr__(me):
        return "%s," % join(["%d" % int(i) for i in me.d], ",")
    def __mul__(me, b):
        return U([me.d[i] + b.d[i] for i in xrange(len(me.d))])
    def __div__(me, b):
        return U([me.d[i] - b.d[i] for i in xrange(len(me.d))])
    def __pow__(me, b):
        if (type(b) != type(0)) and (type(b) != type(0L)):
            raise TypeError("Bad exponent:  " + `b`)
        if b == 0: return U([0]*len(me.d))
        if b == 1:  return x
        if b < 0:
            x = U([-i for i in me.d])
            if b == -1:  return x
            for i in xrange(abs(b)-1): x /= me
        else:
            x = U(me.d)
            for i in xrange(b-1): x *= me
        return x

def ParseCommandLine():
    program_name = sys.argv[0]
    if len(sys.argv) < 2:
        out(manpage % {"name" : program_name})
        sys.exit(0)
    try:
        optlist, args = getopt.getopt(sys.argv[1:], "es")
    except getopt.error, str:
        err(str)
        sys.exit(1)
    for opt in optlist:
        if opt[0] == "-e":
            d = {"fundamental":"", "derived":"", "constants":""}
            out(configuration_file_contents % d)
            sys.exit(0)
        if opt[0] == "-s":
            d = {"fundamental":s_fundamental, 
                 "derived":s_derived, 
                 "constants":s_constants}
            out(configuration_file_contents % d)
            sys.exit(0)
    if len(args) != 1:
        Usage()
    return args[0]

def ReadConfigFile(config_file):
    try:
        lines = open(config_file).readlines()
    except:
        err("Error:  couldn't read config file '%s'%s" % (config_file, nl))
        sys.exit(1)
    return lines

def RemoveComments(lines):
    # Remove comments and empty lines by reading backwards through the file
    for ix in xrange(len(lines)-1, -1, -1):
        if not lines[ix].strip():
            del lines[ix]
            continue
        if lines[ix].strip()[0] == "#":
            del lines[ix]
    lines = [line.strip() for line in lines]
    return lines

def GetSection(lines, StartString, EndString):
    '''Find the text between lines with StartString and EndString on
    them.  Return the text between them and delete the associated lines
    from the lines list.
    '''
    StartString += nl  # The lines from the file have newlines at the end
    EndString += nl
    start = end = -1
    if StartString not in lines:
        if EndString in lines:
            err("Error:  Missing '%s' in config file%s" % (StartString, nl))
            sys.exit(1)
        return None
    if StartString in lines:
        start = lines.index(StartString)
        if EndString not in lines:
            err("Error:  Missing '%s' in config file%s" % (EndString, nl))
            sys.exit(1)
        end = lines.index(EndString)
        if start > end:
            err("Error:  '%s' comes before '%s' in config file%s" % \
                (EndString, StartString, nl))
            sys.exit(1)
    header = None
    if end - start > 1:
        header = lines[start+1:end]
    del lines[start:end+1]
    if header:
        return join(header, nl)
    else:
        return None

def Split(line, char="="):
    '''Split on the first indicated character.
    '''
    if char not in line:
        err("Error:  configuration file line missing '%s':%s%s%s" % 
            (char, nl, line, nl))
        sys.exit(1)
    position = line.find(char)
    key = line[:position].strip()
    value = line[position+1:].strip()
    return [key, value]

def ParseLine(line):
    '''Parse on the first '=' character on the line.
    '''
    if '=' not in line:
        err("Error:  configuration file line missing '=':%s%s%s" % 
            (nl, line, nl))
        sys.exit(1)
    position = line.find("=")
    key = line[:position].strip()
    value = line[position+1:].strip()
    return [key, value]

def ParseUnit(line, string):
    '''Make sure there's an '=' character and two tokens.
    '''
    if '=' not in string:
        err("Error:  configuration file line missing '=':%s%s%s" % 
            (nl, line, nl))
        sys.exit(1)
    name, symbol = Split(string, "=")
    return name + "=" + symbol

def ProcessLines(lines):
    '''Fill the output_strings global with the relevant information, 
    ready to be printed out to the include file.
    '''
    global output_strings
    AllowedKeys = (units_class, units_namespace, numerical_type, f_unit, 
        d_unit, constant, include_file_name)
    for line in lines:
        key, value = Split(line, "=")
        if key not in AllowedKeys:
            err("Error:  configuration file line with bad key '%s':%s%s%s" % 
                (key, nl, line, nl))
            sys.exit(1)
        if key == f_unit:
            output_strings[fundamental].append(value)
        elif key == d_unit:
            output_strings[derived].append(value)
        elif key == constant:
            output_strings[constant].append(value)
        else:
            output_strings[key] = value

def CheckData():
    # Check for duplicates in fundamental units.
    # Form is Unit = Length
    units = {}
    for name in output_strings[fundamental]:
        if units.has_key(name):
            err("Error:  '%s' is a duplicated fundamental unit%s" % (name, nl))
            sys.exit(1)
        else:
            units[name] = 0
    # Check for duplicates in derived units.
    # Form is DerivedUnit = Expression(fund. units, derived units)
    units = {}
    for item in output_strings[derived]:
        type, name = Split(item, "=")
        if units.has_key(type):
            err("Error:  '%s' is a duplicated derived unit%s" % (type, nl))
            sys.exit(1)
        else:
            units[type] = 0
    # Check for duplicates in constants units.
    # Form is Constant const Length mm = m/1000;
    units = {}
    for item in output_strings[constant]:
        if "=" not in item:
            err("Error:  constant '%s' is missing '='%s" % (item, nl))
            sys.exit(1)
        items = item.split()
        if len(items) < 4:
            err("Error:  constant '%s' is of improper form%s" % (item, nl))
            sys.exit(1)
        if items[0] == "const":
            key = items[2]
        else:
            key = items[1]
        if units.has_key(key):
            err("Error:  '%s' is a duplicated constant%s" % (key, nl))
            sys.exit(1)
        else:
            units[key] = 0

def MakeMap():
    '''Make a map that contains all the variables needed for outputting
    the include file.
    '''
    m = {}
    for key in output_strings[parameters]:
        m[key] = output_strings[parameters][key]
    m[numerical_type]   = output_strings[numerical_type]
    m[units_class]      = output_strings[units_class]
    m[outstream]        = output_strings[outstream]
    m[class_identifier] = output_strings[class_identifier]
    return m

def MakeTemplateParameters():
    def f(s, number_of_dimensions):
        string = range(1, number_of_dimensions + 1)
        string = [s % i for i in string]
        return join(string, ", ")
    def g(s, number_of_dimensions):
        string = range(1, number_of_dimensions + 1)
        string = [s % (i, i) for i in string]
        return join(string, ", ")
    global output_strings
    number_of_dimensions = len(output_strings[fundamental])
    d = {}
    d["PAR"]     = f("U%d", number_of_dimensions)
    d["PARa"]    = f("U%da", number_of_dimensions)
    d["PARb"]    = f("U%db", number_of_dimensions)
    d["MPAR"]    = f("-U%d", number_of_dimensions)
    d["IPAR"]    = f("int U%d", number_of_dimensions)
    d["IPARmul"] = f("int U%da", number_of_dimensions) + ", " + \
                   f("int U%db", number_of_dimensions)
    d["PARaPb"]  = g("U%da+U%db", number_of_dimensions)
    d["PARaMb"]  = g("U%da-U%db", number_of_dimensions)
    d["UNIT"]    = "%s<%s>" % (output_strings[units_class], d["PAR"])
    d["mUNIT"]    = "%s<%s>" % (output_strings[units_class], d["MPAR"])
    output_strings[parameters] = d

def BuildNeededStrings():
    global output_strings
    # String for dumping unit's dimensions
    number_of_dimensions = len(output_strings[fundamental])
    string = range(1, number_of_dimensions)
    string = ["<< U%d << \",\"" % i for i in string]
    string.append("<< U%d" % number_of_dimensions)
    s = unit_marker_begin + join(string) + " << " + unit_marker_end
    output_strings[outstream] = s
    # Build typedefs
    s = ""
    ns = output_strings[units_namespace]
    n = len(output_strings[fundamental])
    names_to_pos = {}  # Map dimension names to position for derived typedefs
    for ix in xrange(n):
        name = output_strings[fundamental][ix]
        tp = repr(output_strings[Units][name])
        if tp[-1] == ",": 
            tp = tp[:-1] # Remove comma
        if ns:
            s += "typedef %s::%s<%s> %s;%s" % \
                (ns, output_strings[units_class], tp, name, nl)
        else:
            s += "typedef %s<%s> %s;%s" % \
                (output_strings[units_class], tp, name, nl)
        names_to_pos[name] = ix + 1
    output_strings[typedefs] = s 
    # Build derived typedefs
    def Derived(derived_unit_string):
        name, expression = Split(derived_unit_string, "=")
        tp = repr(output_strings[Units][name])
        if tp[-1] == ",": 
            tp = tp[:-1] # Remove comma
        if ns:
            s = "typedef %s::%s<%s> %s;%s" % \
                (ns, output_strings[units_class], tp, name, nl)
        else:
            s = "typedef %s<%s> %s;%s" % \
                (output_strings[units_class], tp, name, nl)
        return s
    n = len(output_strings[derived])
    s = ""
    for item in output_strings[derived]:
        s += Derived(item)
    output_strings[typedefs_derived] = s
    # Build constants
    s = ""
    for string in output_strings[constant]:
        s += string + nl
    output_strings[constants] = s + nl

def WriteIncludeFile():
    s = output_strings[include_file_name]
    of = sys.stdout
    if s != "":
        # Note file will be overwritten if it is present
        of = open(s, "w")
    else:
        s = output_strings[include_file_default_name]
    out = of.write
    s = s.replace(".", "_")
    t = "#ifndef %s%s" % (s, nl)
    out(t)
    out("#define %s%s" % (s, nl))
    out(output_strings[needed_includes])
    if output_strings[header]:
        out(nl + "// From configuration file header" + nl)
        out(output_strings[header] + nl + nl)
    if output_strings[units_namespace]:
        out("namespace %s%s" % (output_strings[units_namespace], nl))
        out("{%s" % nl)
    out(include_file % MakeMap())
    out(nl + "// Typedefs for fundamental units" + nl)
    out(output_strings[typedefs])
    out(nl + "// Typedefs for derived units" + nl)
    out(output_strings[typedefs_derived])
    out(nl + "// Unit constants" + nl)
    out(output_strings[constants])
    if output_strings[trailer]:
        out("// From configuration file trailer" + nl)
        out(output_strings[trailer] + nl)
    if output_strings[units_namespace]:
        out("} // namespace %s%s%s" % (output_strings[units_namespace], nl, nl))
    out("#endif //" + t)
    if s:
        of.close()

def ConstructFundamentalUnits():
    '''Our basic procedure is to compile expressions that create the
    fundamental unit names objects in our local scope, then put the objects
    into the dictionary keyed by "Units" in the output_strings global 
    dictionary.  We'll use these to evaluate the expressions for derived
    units.
    '''
    global output_strings
    n = len(output_strings[fundamental])
    d = {}  # Dictionary for our fundamental objects
    for ix in xrange(n):
        name = output_strings[fundamental][ix]
        t = [0]*n
        t[ix] = 1
        # Construct the string to compile
        s = "%s = U((%s))" % (name, repr(U(t)))
        try:
            c = compile(s, "", "single")
            eval(c)  # Evaluate it in our local namespace
            # Get the object created and put into our dictionary
            d[name] = locals()[name]
        except:
            err("Fundamental unit '%s' has a problem.%s" % (name, nl))
            sys.exit(1)
    # Store into the global dictionary for use with evaluating derived
    # units.
    if len(d) == 0:
        err("Need at least one Unit statement in configuration file" + nl)
        sys.exit(1)
    output_strings[Units] = d

def ConstructDerivedUnits():
    '''The procedure here is analogous to what was done in
    ConstructFundamentalUnits, except we'll construct strings to be
    compile that contain the expressions the user put into the
    configuration file.  We'll then get the dimensions of the
    resulting unit, which is what we'll use later for the template
    parameters of the derived unit.  The beauty of this approach is
    that we use the python parser to evaluate the expressions and
    determine whether they are legitimate (i.e., we don't have to
    write a parser).  The U class captures the semantics of combining
    the units' dimensions.
    '''
    global output_strings
    num_fund = len(output_strings[fundamental])
    # First, put the fundamental units into our local namespace.  This
    # duplicates the work in ConstructFundamentalUnits(), but I wanted
    # to keep the tasks separate.
    for name in output_strings[Units]:
        try:
            s = "%s = U((%s))" % (name, output_strings[Units][name])
            c = compile(s, "", "single")
            eval(c)
        except:
            err("Internal error in ConstructDerivedUnits()" + nl)
            sys.exit(1)
    # Now evaluate the derived expressions
    n = len(output_strings[derived])
    d = output_strings[Units]  # Fundamental units are already here
    one = U([0]*num_fund)      # Allow things like '1/Length'
    for ix in xrange(n):
        name, expression = Split(output_strings[derived][ix], "=")
        if d.has_key(name):
            err("Derived unit '%s' is already the name of a unit.%s" % 
                (name, nl))
            sys.exit(1)
        # Construct the string to compile
        s = expression
        # Needed substitutions to make things work
        s = s.replace("1", "one")  # Allow things line '1/Length'
        s = s.replace("^", "**")   # Let's '^' be used for exponentiation
        try:
            c = compile(s, "", "eval")
            result = eval(c)
        except:
            err("Derived unit '%s' not a valid expression.%s" % (name, nl))
            sys.exit(1)
        d[name] = U(result.d)
        locals()[name] = d[name]
    output_strings[Units] = d

def main():
    config_file = ParseCommandLine()
    lines = ReadConfigFile(config_file)
    output_strings[header]  = GetSection(lines, header_begin, header_end)
    output_strings[trailer] = GetSection(lines, trailer_begin, trailer_end)
    RemoveComments(lines)
    ProcessLines(lines)
    CheckData()
    ConstructFundamentalUnits()
    ConstructDerivedUnits()
    BuildNeededStrings()
    MakeTemplateParameters()
    WriteIncludeFile()

main()

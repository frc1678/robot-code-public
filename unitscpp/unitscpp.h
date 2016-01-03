#ifndef unitscpp_h
#define unitscpp_h

#include <ostream>
#include <sstream>
#include <string>

typedef double NT;  // Number Type

template <int U1, int U2, int U3, int U4>
class Units {
 public:
  Units(NT value_ = NT(0)) : value(value_) {}

  // This turns the class into a function object that allows
  // the user to easily get at the value.
  NT operator()() const { return value; }

  static constexpr int u1 = U1, u2 = U2, u3 = U3, u4 = U4;

  // Helper function to get a text representation of the
  // object's dimensions.  It is static because the
  // representation is known at compile time.
  static std::string dim(void) {
    std::stringstream s;
    s << "<" << U1 << "," << U2 << "," << U3 << "," << U4 << ">";
    return s.str();
  }

  /** BEGIN CHANGED SECTION **/
  static int dim_length() { return U1; }

  static int dim_time() { return U2; }

  static int dim_mass() { return U3; }

  static int dim_current() { return U4; }
  /** END CHANGED SECTION **/

  // Helper function for unit conversions.
  NT to(const Units& u) const { return value / u.value; }

  Units& operator=(const Units& rhs) {
    value = rhs.value;
    return *this;
  }

  // Arithmetic operators
  Units& operator+=(const Units& rhs) {
    value += rhs.value;
    return *this;
  }

  Units& operator-=(const Units& rhs) {
    value -= rhs.value;
    return *this;
  }

  Units& operator*=(const NT& rhs) {
    value *= rhs;
    return *this;
  }

  Units& operator/=(const NT& rhs) {
    value /= rhs;
    return *this;
  }

 private:
  NT value;
};

// Addition
template <int U1, int U2, int U3, int U4>
const Units<U1, U2, U3, U4> operator+(const Units<U1, U2, U3, U4>& lhs,
                                      const Units<U1, U2, U3, U4>& rhs) {
  return Units<U1, U2, U3, U4>(lhs() + rhs());
}

// Subtraction
template <int U1, int U2, int U3, int U4>
const Units<U1, U2, U3, U4> operator-(const Units<U1, U2, U3, U4>& lhs,
                                      const Units<U1, U2, U3, U4>& rhs) {
  return Units<U1, U2, U3, U4>(lhs() - rhs());
}

// Negation
template <int U1, int U2, int U3, int U4>
const Units<U1, U2, U3, U4> operator-(const Units<U1, U2, U3, U4>& val) {
  return Units<U1, U2, U3, U4>(-val());
}

// Multiplication
template <int U1, int U2, int U3, int U4>
const Units<U1, U2, U3, U4> operator*(const NT& lhs,
                                      const Units<U1, U2, U3, U4>& rhs) {
  return Units<U1, U2, U3, U4>(lhs * rhs());
}

template <int U1, int U2, int U3, int U4>
const Units<U1, U2, U3, U4> operator*(const Units<U1, U2, U3, U4>& lhs,
                                      const NT& rhs) {
  return rhs * lhs;
}

template <int U1a, int U2a, int U3a, int U4a, int U1b, int U2b, int U3b,
          int U4b>
const Units<U1a + U1b, U2a + U2b, U3a + U3b, U4a + U4b> operator*(
    const Units<U1a, U2a, U3a, U4a>& lhs,
    const Units<U1b, U2b, U3b, U4b>& rhs) {
  return Units<U1a + U1b, U2a + U2b, U3a + U3b, U4a + U4b>(lhs() * rhs());
}

// Division
template <int U1, int U2, int U3, int U4>
const Units<U1, U2, U3, U4> operator/(const Units<U1, U2, U3, U4>& lhs,
                                      const NT& rhs) {
  return Units<U1, U2, U3, U4>(lhs() / rhs);
}

template <int U1, int U2, int U3, int U4>
const Units<-U1, -U2, -U3, -U4> operator/(const NT& lhs,
                                          const Units<U1, U2, U3, U4>& rhs) {
  return Units<-U1, -U2, -U3, -U4>(lhs / rhs());
}

template <int U1a, int U2a, int U3a, int U4a, int U1b, int U2b, int U3b,
          int U4b>
const Units<U1a - U1b, U2a - U2b, U3a - U3b, U4a - U4b> operator/(
    const Units<U1a, U2a, U3a, U4a>& lhs,
    const Units<U1b, U2b, U3b, U4b>& rhs) {
  return Units<U1a - U1b, U2a - U2b, U3a - U3b, U4a - U4b>(lhs() / rhs());
}

// Comparisons
template <int U1, int U2, int U3, int U4>
bool operator==(const Units<U1, U2, U3, U4>& lhs,
                const Units<U1, U2, U3, U4>& rhs) {
  return (lhs() == rhs());
}

template <int U1, int U2, int U3, int U4>
bool operator==(const Units<U1, U2, U3, U4>& lhs, const NT& rhs) {
  return (lhs() == rhs);
}

template <int U1, int U2, int U3, int U4>
bool operator==(const NT& lhs, const Units<U1, U2, U3, U4>& rhs) {
  return (lhs == rhs());
}

template <int U1, int U2, int U3, int U4>
bool operator!=(const Units<U1, U2, U3, U4>& lhs,
                const Units<U1, U2, U3, U4>& rhs) {
  return not(lhs() == rhs());
}

template <int U1, int U2, int U3, int U4>
bool operator!=(const Units<U1, U2, U3, U4>& lhs, const NT& rhs) {
  return not(lhs() == rhs);
}

template <int U1, int U2, int U3, int U4>
bool operator!=(const NT& lhs, const Units<U1, U2, U3, U4>& rhs) {
  return not(lhs == rhs());
}

// Ordering
template <int U1, int U2, int U3, int U4>
bool operator<=(const Units<U1, U2, U3, U4>& lhs,
                const Units<U1, U2, U3, U4>& rhs) {
  return lhs() <= rhs();
}

template <int U1, int U2, int U3, int U4>
bool operator<=(const Units<U1, U2, U3, U4>& lhs, const NT& rhs) {
  return (lhs() <= rhs);
}

template <int U1, int U2, int U3, int U4>
bool operator<=(const NT& lhs, const Units<U1, U2, U3, U4>& rhs) {
  return (lhs <= rhs());
}

template <int U1, int U2, int U3, int U4>
bool operator>=(const Units<U1, U2, U3, U4>& lhs,
                const Units<U1, U2, U3, U4>& rhs) {
  return lhs() >= rhs();
}

template <int U1, int U2, int U3, int U4>
bool operator>=(const Units<U1, U2, U3, U4>& lhs, const NT& rhs) {
  return (lhs() >= rhs);
}

template <int U1, int U2, int U3, int U4>
bool operator>=(const NT& lhs, const Units<U1, U2, U3, U4>& rhs) {
  return (lhs >= rhs());
}

template <int U1, int U2, int U3, int U4>
bool operator<(const Units<U1, U2, U3, U4>& lhs,
               const Units<U1, U2, U3, U4>& rhs) {
  return lhs() < rhs();
}

template <int U1, int U2, int U3, int U4>
bool operator<(const Units<U1, U2, U3, U4>& lhs, const NT& rhs) {
  return (lhs() < rhs);
}

template <int U1, int U2, int U3, int U4>
bool operator<(const NT& lhs, const Units<U1, U2, U3, U4>& rhs) {
  return (lhs < rhs());
}

template <int U1, int U2, int U3, int U4>
bool operator>(const Units<U1, U2, U3, U4>& lhs,
               const Units<U1, U2, U3, U4>& rhs) {
  return lhs() > rhs();
}

template <int U1, int U2, int U3, int U4>
bool operator>(const Units<U1, U2, U3, U4>& lhs, const NT& rhs) {
  return (lhs() > rhs);
}

template <int U1, int U2, int U3, int U4>
bool operator>(const NT& lhs, const Units<U1, U2, U3, U4>& rhs) {
  return (lhs > rhs());
}

template <int U1, int U2, int U3, int U4>
std::ostream& operator<<(std::ostream& s, const Units<U1, U2, U3, U4>& rhs) {
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

// Typedefs for fundamental units
typedef Units<1, 0, 0, 0> Length;
typedef Units<0, 1, 0, 0> Time;
typedef Units<0, 0, 1, 0> Mass;
typedef Units<0, 0, 0, 1> Current;

// Typedefs for derived units
typedef Units<0, 0, 0, 0> Angle;
typedef Units<1, -1, 0, 0> Velocity;
typedef Units<1, -2, 0, 0> Acceleration;
typedef Units<0, -1, 0, 0> AngularVelocity;
typedef Units<0, -2, 0, 0> AngularAcceleration;
typedef Units<1, -2, 1, 0> Force;
typedef Units<-1, -2, 1, 0> Pressure;
typedef Units<-1, -2, 1, 0> Stress;
typedef Units<2, -2, 1, 0> Energy;
typedef Units<0, -1, 0, 0> Frequency;
typedef Units<2, -3, 1, -1> Voltage;
typedef Units<0, 0, 0, 0> Unitless;

// Unit constants
const double pi = 3.14159265358;
const Length m = 1;
const Length km = 1e3 * m;
const Length cm = 1e-2 * m;
const Length mm = 1e-3 * m;
const Length in = 2.54 * cm;
const Length ft = 12 * in;
const Angle rad = 1;
const Angle deg = 1.74532925e-2 * rad;
const Angle rev = 2 * pi * rad;
const Mass kg = 1;
const Mass g = .001 * kg;
const Mass mg = .001 * g;
const Time s = 1;
const Time ms = 1e-3 * s;
const Time us = 1e-6 * s;
const Time hour = 3600 * s;
const Current A = 1;
const Voltage V = kg * m * m / (A * s * s * s);
const Length click = pi * 4 * in / 360;
const Length robotWidth = 1.5 * ft;
const Velocity maxHighRobotSpeed = 15 * ft / s;
const Velocity maxLowRobotSpeed = 5 * ft / s;
const AngularVelocity maxHighRobotAngularVelocity =
    2 * maxHighRobotSpeed / (robotWidth / 2) * rad;
const Unitless none = 1;
const Frequency hz = 1 / s;

#endif  //#ifndef unitscpp_h

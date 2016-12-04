#include "gyro_interface.h"

#include <cstring>
#include <inttypes.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GyroInterface::GyroInterface() : gyro_(new SPI(SPI::kOnboardCS0)) {
  // The gyro goes up to 8.08MHz.
  // The myRIO goes up to 4MHz, so the roboRIO probably does too.
  gyro_->SetClockRate(4e6);
  gyro_->SetChipSelectActiveLow();
  gyro_->SetClockActiveHigh();
  gyro_->SetSampleDataOnRising();
  gyro_->SetMSBFirst();
}

bool GyroInterface::InitializeGyro() {
  uint32_t result;
  if (!DoTransaction(0x20000003, &result)) {
    return false;
  }
  if (result != 1) {
    // We might have hit a parity error or something and are now retrying, so
    // this isn't a very big deal.
  }

  // Wait for it to assert the fault conditions before reading them.
  usleep(50000);

  if (!DoTransaction(0x20000000, &result)) {
    return false;
  }

  if (!DoTransaction(0x20000000, &result)) {
    return false;
  }
  if (ExtractStatus(result) != 2) {
    return false;
  }
  if (ExtractErrors(result) != 0x7F) {
    return false;
  }

  if (!DoTransaction(0x20000000, &result)) {
    return false;
  }
  if (ExtractStatus(result) != 2) {
    return false;
  }

  return true;
}

bool GyroInterface::DoTransaction(uint32_t to_write, uint32_t *result) {
  static const uint8_t kBytes = 4;
  static_assert(kBytes == sizeof(to_write),
                "need the same number of bytes as sizeof(the data)");

  if (__builtin_parity(to_write & ~1) == 0) to_write |= 1;

  uint8_t to_send[kBytes], to_receive[kBytes];
  const uint32_t to_write_flipped = __builtin_bswap32(to_write);
  memcpy(to_send, &to_write_flipped, kBytes);

  switch (gyro_->Transaction(to_send, to_receive, kBytes)) {
    case -1:
      return false;
    case kBytes:
      break;
    default:
      break;
  }

  memcpy(result, to_receive, kBytes);
  if (__builtin_parity(*result & 0xFFFF) != 1) {
    return false;
  }
  if (__builtin_parity(*result) != 1) {
    return false;
  }

  *result = __builtin_bswap32(*result);
  return true;
}

uint16_t GyroInterface::DoRead(uint8_t address) {
  // no idea how this works
  const uint32_t command = (0x8 << 28) | (address << 17);
  uint32_t response;
  while (true) {
    if (!DoTransaction(command, &response)) {
      continue;
    }
    // there's probably some checks in the high bits
    if ((response & 0xEFE00000) != 0x4E000000) {
      continue;
    }
    // data must be in bits 6 to 21
    return (response >> 5) & 0xFFFF;
  }
}

// gets angular velocity
double GyroInterface::ExtractAngle(uint32_t value) {
  const int16_t reading = -(int16_t)(value >> 10 & 0xFFFF);
  return ((static_cast<double>(reading) * 2.0 * M_PI / 360.0 / 80.0) - .0068);
}

uint32_t GyroInterface::ReadPartID() {
  return (DoRead(0x0E) << 16) | DoRead(0x10);
}

uint32_t GyroInterface::GetReading() {
  uint32_t result;
  if (!DoTransaction(0x20000000, &result)) {
    return 0;
  }
  return result;
}

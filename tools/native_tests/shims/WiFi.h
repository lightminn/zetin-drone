#pragma once

#include <cstdint>
#include <ostream>

class IPAddress {
public:
  constexpr IPAddress() = default;
  constexpr IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
      : octets_{a, b, c, d} {}

  constexpr uint8_t operator[](std::size_t index) const {
    return octets_[index];
  }

private:
  uint8_t octets_[4] = {0, 0, 0, 0};
};

inline std::ostream &operator<<(std::ostream &stream, const IPAddress &address) {
  return stream << static_cast<int>(address[0]) << '.'
                << static_cast<int>(address[1]) << '.'
                << static_cast<int>(address[2]) << '.'
                << static_cast<int>(address[3]);
}

constexpr int WIFI_POWER_19_5dBm = 78;

class WiFiClass {
public:
  bool softAP(const char *, const char *, int) { return true; }
  void setSleep(bool) {}
  void setTxPower(int) {}
};

inline WiFiClass WiFi;

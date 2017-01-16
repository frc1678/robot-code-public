#include "third_party/aos/vision/events/udp.h"

#include <string.h>

namespace aos {
namespace vision {

TXUdpSocket::TXUdpSocket(const char *ip_addr, int port)
    : fd_(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) {
  sockaddr_in destination_in;
  memset(&destination_in, 0, sizeof(destination_in));
  destination_in.sin_family = AF_INET;
  destination_in.sin_port = htons(port);
  inet_aton(ip_addr, &destination_in.sin_addr);

  connect(fd_.get(), reinterpret_cast<sockaddr *>(&destination_in),
                 sizeof(destination_in));
}

int TXUdpSocket::Send(const void *data, int size) {
  return send(fd_.get(), static_cast<const char *>(data), size, 0);
}

RXUdpSocket::RXUdpSocket(int port)
    : fd_(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) {
  sockaddr_in bind_address;
  memset(&bind_address, 0, sizeof(bind_address));

  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(port);
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);

  bind(fd_.get(), reinterpret_cast<sockaddr *>(&bind_address),
              sizeof(bind_address));
}

int RXUdpSocket::Recv(void *data, int size) {
  return recv(fd_.get(), static_cast<char *>(data), size, 0);
}

}  // namespace vision
}  // namespace aos

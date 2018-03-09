#ifndef MUAN_VISION_VIDEO_STREAM_H_
#define MUAN_VISION_VIDEO_STREAM_H_

#include <netinet/in.h>
#include <poll.h>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "muan/webdash/queue_types.h"
#include "muan/webdash/server.h"

namespace muan {
namespace webdash {

using VideoStreamQueue = muan::queues::MessageQueue<cv::Mat>;
class WebDashStreamer {
 public:
  explicit WebDashStreamer(WebDashRunner* runner);
  ~WebDashStreamer() = default;

  void AddQueue(std::string name, VideoStreamQueue* queue);

  void operator()();

  void Stop();

  void Start();

 private:
  struct ServerInfo {
    int max_backlog;
    sockaddr_in address;
    int addrlen;
  };

  void Update();
  void InitNetworking();
  void AcceptConnection();
  // Responds to event on connections_[connection_index].
  // Returns whether to keep the connection alive.
  bool HandleRequest(int connection_index);
  // Sends the image to all connections requesting the stream with the given
  // name
  void SendImage(std::string name, cv::Mat image);
  // "GET /foo.mjpeg HTTP/1.1\r\n...." -> "/foo.mjpeg". Returns "" on error.
  std::string ParseRequest(std::string request);
  void CloseConnection(int connection_index);

  ServerInfo info_;
  // First element is main socket, the rest are connections.
  // Connections has file descriptors and poll info, name of requested stream in
  // stream_requests
  std::vector<pollfd> connections_;
  // What resource is requested. Index corresponds to that of connections_.
  std::vector<std::string> stream_requests_;
  std::map<std::string, VideoStreamQueue*> streams_;
  std::vector<char> buffer_;
  bool running_;
  WebDashRunner* runner_;
};

}  // namespace webdash
}  // namespace muan

#endif  // MUAN_VISION_VIDEO_STREAM_H_

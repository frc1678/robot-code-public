#include "muan/vision/video_stream.h"
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "muan/logging/logger.h"
#include "third_party/aos/common/die.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/optional/optional.hpp"
namespace muan {
namespace webdash {

// MJPEG HTTP format:
//     HTTP/1.1 200 OK
//     Conent-Type: multipart/x-mixed-replace; boundary=boundary
//
//     --boundary
//     Content-Length: [JPEG size] (not actually necessary but there may be one
//     frame lag otherwise)
//
//     [JPEG binary data]--boundary
//     Content-Length: [JPEG size]
//
//     [JPEG binary data] ....

WebDashStreamer::WebDashStreamer(WebDashRunner* runner) {
  // Up to 3 connections can be waiting
  info_.max_backlog = 3;
  info_.address.sin_family = AF_INET;
  info_.address.sin_addr.s_addr = INADDR_ANY;
  // Port 5802
  info_.address.sin_port = htons(5802);
  info_.addrlen = sizeof(info_.address);
  // 1MiB initial buffer size
  buffer_ = std::vector<char>(1024 * 1024);
  runner_ = runner;
}

void WebDashStreamer::AddQueue(std::string name, VideoStreamQueue* queue) {
  std::string key = "/" + name + ".mjpeg";
  if (streams_.find(key) != streams_.end()) {
    LOG(FATAL, "Two video streams with same name");
    aos::Die("Two video streams with same name %s", name.c_str());
  }
  streams_.emplace(key, queue);
  if (runner_) {
    runner_->AddVideoStream(name);
  }
}

void WebDashStreamer::Stop() { running_ = false; }
void WebDashStreamer::Start() { running_ = true; }

void WebDashStreamer::operator()() {
  InitNetworking();
  running_ = true;
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(50));
  while (true) {
    phased_loop.SleepUntilNext();
    if (running_) {
      Update();
    }
  }
}

void WebDashStreamer::Update() {
  // Check for network activity
  if (poll(connections_.data(), connections_.size(), 0) == -1) {
    aos::Die("poll failed");
  }
  // Input on main socket means there is a connection pending
  if (connections_[0].revents & POLLIN) {
    AcceptConnection();
  }
  for (size_t i = 1; i < connections_.size(); i++) {
    // Input on an open connection is a HTTP request or a disconnection
    if (connections_[i].revents & POLLIN && connections_[i].fd != 0) {
      bool keep_alive = HandleRequest(i);
      LOG(INFO, "HTTP request");
      if (!keep_alive) {
        CloseConnection(i);
        LOG(ERROR, "Disconnection");
      }
    }
  }
  // Keep track of whether each stream has been sent. Initially, none are.
  static std::map<std::string, bool> stream_sent;
  for (const auto& stream : streams_) {
    stream_sent[stream.first] = false;
  }
  for (size_t i = 1; i < connections_.size(); i++) {
    // Is this connection requesting a stream that has not already been sent?
    // This prevents streams that aren't requested from being serialized
    // unnecessarily
    if (stream_requests_[i] != "" && !stream_sent[stream_requests_[i]]) {
      std::experimental::optional<cv::Mat> frame;
      if ((frame = streams_[stream_requests_[i]]->ReadLastMessage())) {
        SendImage(stream_requests_[i], *frame);
      }
      stream_sent[stream_requests_[i]] = true;
    }
  }
}

void WebDashStreamer::InitNetworking() {
  connections_.push_back(pollfd());
  stream_requests_.push_back("");
  // Get a TCP socket
  if ((connections_[0].fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    LOG(FATAL, "Socket failed");
    aos::Die("socket failed");
  }
  // Tell the socket to reuse resources from closed connections
  int opt = 1;
  if (setsockopt(connections_[0].fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt))) {
    LOG(FATAL, "Setsockopt Connection failed");
    aos::Die("setsockopt failed");
  }
  // Attach the socket to the specified port
  if (bind(connections_[0].fd, reinterpret_cast<sockaddr*>(&info_.address),
           info_.addrlen) < 0) {
    LOG(FATAL, "Bind connection failed");
    aos::Die("bind failed");
  }
  // Put the socket in server mode
  if (listen(connections_[0].fd, info_.max_backlog) < 0) {
    LOG(FATAL, "Listen Connection failed");
    aos::Die("listen failed");
  }
  // Poll for incoming connections
  connections_[0].events = POLLIN;
}

void WebDashStreamer::AcceptConnection() {
  int new_connection_fd =
      accept4(connections_[0].fd, reinterpret_cast<sockaddr*>(&info_.address),
              reinterpret_cast<socklen_t*>(&info_.addrlen), SOCK_NONBLOCK);
  if (new_connection_fd < 0) {
    LOG(FATAL, "New connection failed");
    aos::Die("accept4 failed");
  }
  // Store the new connection
  for (size_t i = 1; i < connections_.size(); i++) {
    // If fd is 0, the connection that was there is unused and the new
    // connection can be stored there
    if (connections_[i].fd == 0) {
      connections_[i].fd = new_connection_fd;
      LOG(INFO, "fd = 0, connection unused replacing with new");
      // Begin watching for input on the connection
      connections_[i].events = POLLIN;
      return;
    }
  }
  // If all connections are used, add a new connection to the vector
  pollfd new_connection;
  new_connection.fd = new_connection_fd;
  new_connection.events = POLLIN;
  connections_.push_back(new_connection);
  stream_requests_.push_back("");
}

bool WebDashStreamer::HandleRequest(int connection_index) {
  bool keep_alive = false;
  int lenread = recv(connections_[connection_index].fd, buffer_.data(),
                     buffer_.size(), MSG_DONTWAIT);
  if (lenread > 0) {
    // Set terminating null
    buffer_[lenread] = 0;
    std::string name = ParseRequest(buffer_.data());
    // Well-formed GET requests will set name
    if (name == "") {
      snprintf(buffer_.data(), buffer_.size(),
               "HTTP/1.1 400 Bad Request\r\n\r\n"
               "400 Bad Request or 501 Not Implemented\r\nRequest:\r\n%s",
               buffer_.data());
      LOG(ERROR, "Not a valid HTTP GET request: %s", buffer_.data());
    } else if (streams_.find(name) == streams_.end()) {
      snprintf(buffer_.data(), buffer_.size(),
               "HTTP/1.1 404 Not Found\r\n\r\n"
               "404 Not Found: %s\r\n\r\n",
               name.c_str());
      LOG(ERROR, "Streams not found: %s", name.c_str());
    } else {
      // MJPEG is a series of mixed-replace JPEGs
      snprintf(
          buffer_.data(), buffer_.size(),
          "HTTP/1.1 200 OK\r\n"
          "Content-Type: multipart/x-mixed-replace; boundary=boundary\r\n\r\n");
      stream_requests_[connection_index] = name;
      keep_alive = true;
    }
    if (send(connections_[connection_index].fd, buffer_.data(),
             strlen(buffer_.data()), MSG_NOSIGNAL | MSG_DONTWAIT) !=
        static_cast<ssize_t>(strlen(buffer_.data()))) {
      keep_alive = false;
      LOG(ERROR, "No signal");
    }
  }
  return keep_alive;
}

void WebDashStreamer::SendImage(std::string name, cv::Mat image) {
  // Serialize image
  static std::vector<uchar> jpeg_buffer;
  cv::imencode(".jpeg", image, jpeg_buffer);
  snprintf(buffer_.data(), buffer_.size(),
           "--boundary\r\nContent-Length: %d\r\n\r\n",
           static_cast<int>(jpeg_buffer.size()));
  int response_length = strlen(buffer_.data());
  // Make sure copying the JPEG data won't write past the end of the buffer
  if (response_length + jpeg_buffer.size() > buffer_.size()) {
    // Reserve 20% extra space to accomodate slightly larger images
    buffer_.reserve((response_length + jpeg_buffer.size()) * 1.2);
  }
  memcpy(buffer_.data() + response_length, jpeg_buffer.data(),
         jpeg_buffer.size());
  response_length += jpeg_buffer.size();
  // Send data to all connections requesting this stream
  for (size_t i = 0; i < connections_.size(); i++) {
    if (stream_requests_[i] == name) {
      send(connections_[i].fd, buffer_.data(), response_length,
           MSG_NOSIGNAL | MSG_DONTWAIT);
    }
  }
}

std::string WebDashStreamer::ParseRequest(std::string request) {
  size_t pos = 0;
  // Remove everything after first line
  if ((pos = request.find("\r\n")) == std::string::npos) {
    return "";
  }
  request.erase(pos, request.size() - pos);
  // Remove everything before resource name (only GET supported)
  if (request.find("GET ") != 0) {
    return "";
  }
  request.erase(0, 4);
  // Remove everything after resource name
  if ((pos = request.find(" HTTP/")) == std::string::npos) {
    return "";
  }
  request.erase(pos, request.size() - pos);
  // Only resource name is left
  return request;
}

void WebDashStreamer::CloseConnection(int connection_index) {
  if (close(connections_[connection_index].fd)) {
    LOG(FATAl, "Couldn't close connection");
    aos::Die("close failed");
  }
  // Mark the connection as unused so it can be recycled
  connections_[connection_index].fd = 0;
  // Stop watching it for input
  connections_[connection_index].events = 0;
  // Stop requesting the stream to be serialized
  stream_requests_[connection_index] = "";
  LOG(ERROR,
      "Connection unused, stopped receiving input and serializing stream");
}

}  // namespace webdash
}  // namespace muan

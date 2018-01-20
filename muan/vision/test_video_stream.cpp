#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include "muan/webdash/server.h"
#include "muan/vision/video_stream.h"

int main() {
  cv::VideoCapture cap(0);
  muan::webdash::VideoStreamQueue vision_input_feed(5);
  muan::webdash::VideoStreamQueue vision_output_feed(5);
  muan::webdash::WebDashRunner runner(muan::webdash::JETSON);
  muan::webdash::WebDashStreamer webdash_streamer(&runner);
  webdash_streamer.AddQueue("input", &vision_input_feed);
  webdash_streamer.AddQueue("output", &vision_output_feed);
  std::thread webdash_streamer_thread(webdash_streamer);
  std::thread webdash_runner_thread{std::ref(runner)};
  cv::Mat input, output;
  std::string display_object =
    "{"
    "  \"widgets\": ["
    "     {"
    "       \"name\": \"Raw Camera\","
    "       \"type\": \"image\","
    "       \"source\": \"input\","
    "       \"coordinates\": [0, 0],"
    "       \"should-title\": false"
    "     },"
    "     {"
    "       \"name\": \"Processed Camera\","
    "       \"type\": \"image\","
    "       \"source\": \"output\","
    "       \"coordinates\": [1, 0],"
    "       \"should-title\": false"
    "     }"
    "  ],"
    "  \"settings\": {"
    "    \"size\": [2, 1]"
    "  }"
    "}";
  runner.DisplayObjectMaker(display_object);
  while (true) {
    cap >> input;
    cv::cvtColor(input, output, CV_BGR2GRAY);
    vision_input_feed.WriteMessage(input);
    vision_output_feed.WriteMessage(output);
  }
  webdash_streamer_thread.join();
  webdash_runner_thread.join();
  return 0;
}

// MJPEG 스트림의 경계·파트 헤더 문자열. Arduino 헤더 없이 호스트에서도 빌드된다
// (tools/native_tests/test_camera_stream.cpp).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

constexpr const char* kStreamBoundary = "zetincamframe";

// "\r\n--zetincamframe\r\n" 을 쓰고 길이를 돌려준다. 넘치면 0.
inline size_t formatBoundary(char* buf, size_t cap) {
  const int n = snprintf(buf, cap, "\r\n--%s\r\n", kStreamBoundary);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

// 파트 헤더를 쓰고 길이를 돌려준다. 넘치면 0.
// captureUs 는 esp_timer 부팅 후 µs(64비트)라 "%d" 로 찍으면 안 된다 — CameraWebServer
// 예제가 그렇게 해서 형식이 어긋난다. 명시 캐스트로 long long / unsigned long 에 맞춘다.
inline size_t formatPartHeader(char* buf, size_t cap, uint32_t jpegLen,
                               int64_t captureUs, uint32_t seq) {
  const long long sec = (long long)(captureUs / 1000000);
  const long usec = (long)(captureUs % 1000000);
  const int n = snprintf(buf, cap,
                         "Content-Type: image/jpeg\r\n"
                         "Content-Length: %lu\r\n"
                         "X-Timestamp: %lld.%06ld\r\n"
                         "X-Frame-Seq: %lu\r\n\r\n",
                         (unsigned long)jpegLen, sec, usec, (unsigned long)seq);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

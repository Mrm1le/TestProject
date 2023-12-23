#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace cp {
struct MSDShape3f {
  float height;
  float width;
  float length;
};

struct MSDShape2f {
  float height;
  float width;
};

struct MSDRect4f {
  float left;
  float top;
  float right;
  float bottom;
};

struct MSDFrameMeta {
  uint64_t timestamp_us;
  uint64_t sequence;
  std::string tag;
};

struct MSDPoint2f {
  float x;
  float y;
};

struct MSDPoint2d {
  double x;
  double y;
};

struct MSDPoint3f {
  float x;
  float y;
  float z;
};

struct MSDPoint3d {
  double x;
  double y;
  double z;
};

struct MSDShape3d {
  double length;
  double width;
  double height;
};

struct MSDPolygon3f {
  std::vector<MSDPoint3f> points;
};

struct MSDPolyline3f {
  std::vector<MSDPoint3f> points;
};

struct MSDVector2f {
  float x;
  float y;
};

enum MSDStatus {
  MSD_OK,
  MSD_FAILURE,
};

struct MSDInterval {
  double begin;
  double end;
};

struct MSDMatrix2f {
  float x00;
  float x01;
  float x10;
  float x11;
};

} // namespace cp

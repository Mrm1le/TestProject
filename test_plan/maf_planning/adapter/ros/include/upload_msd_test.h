//
// Created by ros on 5/26/20.`
//
#pragma once

#include "pnc.h"

namespace msquare {
class UploadMsdTest {
public:
  UploadMsdTest() {}
  UploadMsdTest &operator=(const UploadMsdTest &upload_msd_test) = delete;

  void upload(const RecognizeInfo &recognize_info);
  ~UploadMsdTest() {}
};
} // namespace msquare

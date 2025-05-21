// Copyright 2020 UW-APL
// Authors: Aaron Marburg

#pragma once

#include <vector>

#include "liboculus/DataRx.h"

namespace oculus_sonar_driver {

class PublishingDataRx : public liboculus::DataRx {
 public:
  explicit PublishingDataRx(
      const liboculus::IoServiceThread::IoContextPtr &iosrv)
      : DataRx(iosrv), count_(0) {}

  ~PublishingDataRx() {}

  unsigned int count_;
};

}  // namespace oculus_sonar_driver

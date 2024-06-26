// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2023 Advanced Micro Devices, Inc. All rights reserved.

#ifndef __TestAiePl_h_
#define __TestAiePl_h_

#include "tools/common/TestRunner.h"

class TestAiePl : public TestRunner {
  public:
    boost::property_tree::ptree run(std::shared_ptr<xrt_core::device> dev);
    void runTest(std::shared_ptr<xrt_core::device> dev, boost::property_tree::ptree& ptree);

  public:
    TestAiePl();
};

#endif

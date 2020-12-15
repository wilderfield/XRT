/**
 * Copyright (C) 2016-2020 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <iostream>
#include <stdexcept>
#include <string>
#include <cstring>

// XRT includes
#include "experimental/xrt_xclbin.h"
#include "experimental/xrt_device.h"
#include "experimental/xrt_kernel.h"
#include "experimental/xrt_bo.h"

// This value is shared with worgroup size in kernel.cl
constexpr auto COUNT = 1024;

static void usage()
{
    std::cout << "usage: %s [options] -k <bitstream>\n\n";
    std::cout << "  -k <bitstream>\n";
    std::cout << "  [-d <index>]       (default: 0)\n";
    std::cout << "  [-v]\n";
    std::cout << "  [-h]\n\n";
    std::cout << "* Bitstream is required\n";
}

static int run(const xrt::device& device, const xrt::uuid& uuid, const std::string& cuName)
{
  const size_t DATA_SIZE = COUNT * sizeof(int);

  auto simple = xrt::kernel(device, uuid.get(), cuName);
  auto bo0 = xrt::bo(device, DATA_SIZE, simple.group_id(0));
  auto bo1 = xrt::bo(device, DATA_SIZE, simple.group_id(1));
  auto bo0_map = bo0.map<int*>();
  auto bo1_map = bo1.map<int*>();
  std::fill(bo0_map, bo0_map + COUNT, 0);
  std::fill(bo1_map, bo1_map + COUNT, 0);

  // Fill our data sets with pattern
  int foo = 0x10;
  int bufReference[COUNT];
  for (int i = 0; i < COUNT; ++i) {
    bo0_map[i] = 0;
    bo1_map[i] = i;
    bufReference[i] = i + i * foo;
  }

  bo0.sync(XCL_BO_SYNC_BO_TO_DEVICE, DATA_SIZE, 0);
  bo1.sync(XCL_BO_SYNC_BO_TO_DEVICE, DATA_SIZE, 0);

  auto run = simple(bo0, bo1, 0x10);
  run.wait();

  //Get the output;
  std::cout << "Get the output data from the device" << std::endl;
  bo0.sync(XCL_BO_SYNC_BO_FROM_DEVICE, DATA_SIZE, 0);

  // Validate our results
  if (std::memcmp(bo0_map, bufReference, DATA_SIZE))
    throw std::runtime_error("Value read back does not match reference");

  return 0;
}

int
run(int argc, char** argv)
{
  if (argc < 3) {
    usage();
    return 1;
  }

  std::string xclbin_fnm;
  unsigned int device_index = 0;

  std::vector<std::string> args(argv+1,argv+argc);
  std::string cur;
  for (auto& arg : args) {
    if (arg == "-h") {
      usage();
      return 1;
    }

    if (arg[0] == '-') {
      cur = arg;
      continue;
    }

    if (cur == "-k")
      xclbin_fnm = arg;
    else if (cur == "-d")
      device_index = std::stoi(arg);
    else
      throw std::runtime_error("Unknown option value " + cur + " " + arg);
  }

  if (xclbin_fnm.empty())
    throw std::runtime_error("FAILED_TEST\nNo xclbin specified");

  // Construct xclbin from fnm
  auto xclbin = xrt::xclbin(xclbin_fnm);

  std::cout << "Xclbin: " << xclbin_fnm << std::endl;
  
  // Exercise get_xsa_name()
  std::cout << "  XSAName: " << xclbin.get_xsa_name() << std::endl;
  
  // Exercise get_uuid()
  std::cout << "  UUID: " << xclbin.get_uuid().to_string() << std::endl;

  // Exercise kernel APIs */
  for (auto& kernel : xclbin.get_kernels()) {
    std::cout << "  Kernel: " << kernel.get_name() << std::endl;
    // Exercise kernel argument APIs */
    for (auto& karg : kernel.get_args()) {
      std::cout << "    Kernel Argument: " << karg.get_name() << std::endl;
    }
    // Exercise cu APIs */
    for (auto& cu :kernel.get_cus()) {
      std::cout << "    ComputeUnit: " << cu.get_name() << std::endl;
      std::cout << "      BaseAddress: " << "0x" << std::hex << cu.get_base_address() << std::endl;
      // Exercise cu argument APIs */
      for (auto& carg : cu.get_args()) {
        std::cout << "    CU Argument: " << carg.get_name() << std::endl;
        // Exercise mem APIs */
        for (auto& mem: carg.get_mems()) {
          std::cout << "      ConnectedMemory: " << mem.get_name() << std::endl;
          std::cout << "        BaseAddress: " << "0x" << std::hex << mem.get_base_address() << std::endl;
          std::cout << "        Size: " << mem.get_size() << std::endl;
          std::cout << "        Used: " << unsigned(mem.get_used()) << std::endl;
          std::cout << "        Type: " << unsigned(mem.get_type()) << std::endl;
          std::cout << "        Index: " << mem.get_index() << std::endl;
        }
      }
    }
  }

  // Can print object
  // std::cout << xclbin << std::endl;

  std::string cu_name = xclbin.get_kernels(0).get_name() + ":" + xclbin.get_kernels(0).get_cus(0).get_name();

  auto device = xrt::device(device_index);

  // Exercise device.get_xclbin()
  auto device_xclbin = device.get_xclbin();

  std::cout << device_xclbin << std::endl;

  auto uuid = device.load_xclbin(xclbin);
  auto luuid = device.get_xclbin_uuid();
  if (uuid != luuid)
    throw std::runtime_error("FAILED_TEST\nDevice does not have the correct xclbin loaded");

  run(device, uuid, cu_name);
  return 0;
}

int main(int argc, char** argv)
{
  try {
    auto ret = run(argc, argv);
    std::cout << "PASSED TEST\n";
    return ret;
  }
  catch (std::exception const& e) {
    std::cout << "Exception: " << e.what() << "\n";
    std::cout << "FAILED TEST\n";
    return 1;
  }

  std::cout << "PASSED TEST\n";
  return 0;
}

/*
 * Copyright (C) 2020, Xilinx Inc - All rights reserved
 * Xilinx Runtime (XRT) Experimental APIs
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

#ifndef _XRT_XCLBIN_H_
#define _XRT_XCLBIN_H_

#include "xrt.h"
#include "experimental/xrt_uuid.h"

#ifdef __cplusplus
# include <memory>
#include <utility>
# include <vector>
# include <string>
# include <bitset>
# include <iostream>
#endif

/**
 * typedef xrtXclbinHandle - opaque xclbin handle
 */
typedef void* xrtXclbinHandle;

#ifdef __cplusplus

// Utility to replicate pimpl
template<typename IMPL>
class Pimpl
{
public:
  Pimpl() = default;

  virtual ~Pimpl() = default;

  explicit Pimpl(std::shared_ptr<IMPL> handle) : handle(std::move(handle)) {}

  std::shared_ptr<IMPL>
  get_handle() const
  {
    return handle;
  }

protected:
  std::shared_ptr<IMPL> handle;
};

namespace xrt {

/**
 * class xclbin - xrt::xclbin represents an xclbin, and provides APIs for inspecting it.
 *
 * The xclbin object is typically constructed by the user with a file.
 *
 */
class xclbin_impl;

class xclbin : public Pimpl<xclbin_impl>
{
public:
  /**
   * class mem - xrt::xclbin::mem represents a physical device memory
   *
   * The class allows the user to associate cu arguments with device memories
   *
   */
  class mem_impl;
  class mem : public Pimpl<mem_impl>
  {
  public:
    explicit mem(std::shared_ptr<mem_impl> handle) : Pimpl(std::move(handle)) {}

    /**
     * get_name() - Get memory name
     *
     * Return: memory name
     *
     */
    XCL_DRIVER_DLLESPEC
    const std::string&
    get_name() const;

    /**
    * get_base_address() - Get the base address of the memory bank
    *
    * Return: Base address of the memory bank
    *
    */
    XCL_DRIVER_DLLESPEC
    uint64_t
    get_base_address() const;

    /**
    * get_size() - Get the size of the memory in KB
    *
    * Return: Size of memory in KB
    *
    */
    XCL_DRIVER_DLLESPEC
    uint64_t
    get_size() const;

    /**
     * get_used() - Get used status of the memory
     *
     * Return: Used status of the memory
     *
     */
    XCL_DRIVER_DLLESPEC
    uint8_t
    get_used() const;

    /**
     * get_type() - Get the type of the memory
     *
     * Return: Memory type
     *
     */
    XCL_DRIVER_DLLESPEC
    uint8_t
    get_type() const;

    /**
     * get_index() - Get the index of the memory
     *
     * Return: Index of the memory within the memory topology
     *
     */
    XCL_DRIVER_DLLESPEC
    int32_t
    get_index() const;
  };

  /**
   * class arg - xrt::xclbin::arg represents a kernel or cu argument
   *
   * Kernels must be provided arguments to operate on
   *   This class provides meta-data associated with a given argument
   */
  class arg_impl;
  class arg : public Pimpl<arg_impl>
  {
  public:
    explicit arg(std::shared_ptr<arg_impl> handle) : Pimpl(std::move(handle)) {}

    /**
     * get_name() - Get argument name
     *
     * Return: argument name
     *
     */
    XCL_DRIVER_DLLESPEC
    const std::string&
    get_name() const;

    /**
     * get_mems() - Get list of device memories from xclbin.
     *
     * Return: A list of memories from xclbin.
     */
    XCL_DRIVER_DLLESPEC
    std::vector<mem>
    get_mems() const;

    /**
     * get_mems() - Get a memory by index from an xclbin.
     *
     * Return: A mem from xclbin.
     */
    XCL_DRIVER_DLLESPEC
    mem
    get_mems(int32_t index) const;
  };

  /**
   * class cu - xrt::xclbin::cu represents a compute unit in an xclbin
   *
   * The cu handle is explicitly constructed by the xclbin object.
   *
   */
  class cu_impl;
  class cu : public Pimpl<cu_impl>
  {
  public:
    explicit cu(std::shared_ptr<cu_impl> handle) : Pimpl(std::move(handle)) {}

    /**
     * get_name() - Get CU name
     *
     * Return: CU Name
     *
     */
    XCL_DRIVER_DLLESPEC
    const std::string&
    get_name() const;

    /**
     * get_args() - Get list of arguments associated with this object.
     *
     * Return: A list of arguments.
     */
    XCL_DRIVER_DLLESPEC
    std::vector<arg>
    get_args() const;

    /**
     * get_args() - Get argument at index.
     *
     * Return: A list of arguments.
     */
    XCL_DRIVER_DLLESPEC
    arg
    get_args(int32_t index) const;

    /**
     * get_base_address() - Get the base address of the cu
     *
     * Return: Base address of the cu
     *
     */
    XCL_DRIVER_DLLESPEC
    uint64_t
    get_base_address() const;
  };

  /**
   * class kernel - xrt::kernel represents a kernel in an xclbin
   *
   * The kernel handle is explicitly constructed by the xclbin object.
   *
   */
  class kernel_impl;
  class kernel : public Pimpl<kernel_impl>
  {
  public:
    explicit kernel(std::shared_ptr<kernel_impl> handle) : Pimpl(std::move(handle)) {}

    /**
     * get_name() - Get kernel name
     *
     * Return: kernel name
     *
     */
    XCL_DRIVER_DLLESPEC
    const std::string&
    get_name() const;

    /**
     * get_cus() - Get list of cu from kernel.
     *
     * Return: A list of cu from kernel.
     */
    XCL_DRIVER_DLLESPEC
    std::vector<cu>
    get_cus() const;

    /**
     * get_cus() - Get cu from kernel at index.
     *
     * Return: A cu from kernel at index.
     */
    XCL_DRIVER_DLLESPEC
    cu
    get_cus(int32_t index) const;

    /**
     * get_args() - Get list of arguments associated with this object.
     *
     * Return: A list of arguments.
     */
    XCL_DRIVER_DLLESPEC
    std::vector<arg>
    get_args() const;

    /**
     * get_args() - Get argument at index.
     *
     * Return: An argument at index.
     */
    XCL_DRIVER_DLLESPEC
    arg
    get_args(int32_t index) const;
  };


public:
  xclbin() {}
  xclbin(std::shared_ptr<xclbin_impl> handle) : Pimpl(handle) {}

  /**
   * xclbin() - Constructor from an xclbin filename
   *
   * @param filename
   *  Path to the xclbin file
   *
   * The xclbin file must be accessible by the application. An
   * exception is thrown file not found
   */
  XCL_DRIVER_DLLESPEC
  explicit
  xclbin(const std::string& filename);

  /**
   * xclbin() - Constructor from raw data
   *
   * @param data
   *  Raw data of xclbin
   *
   * The raw data of the xclbin can be deleted after calling the constructor.
   */
  XCL_DRIVER_DLLESPEC
  explicit
  xclbin(const std::vector<char>& data);

  /**
   * xclbin() - Constructor from raw data
   *
   * @top: raw data of xclbin file as axlf*
   */
  XCL_DRIVER_DLLESPEC
  explicit
  xclbin(const axlf* top);

  /**
   * get_kernels() - Get list of kernels from xclbin.
   *
   * Return: A list of kernels from xclbin.
   */
  XCL_DRIVER_DLLESPEC
  std::vector<kernel>
  get_kernels() const;

  /**
   * get_kernels() - Get a kernel by index from an xclbin.
   *
   * @index: path to the xclbin file
   * Return: A kernels from xclbin.
   */
  XCL_DRIVER_DLLESPEC
  kernel
  get_kernels(int32_t index) const;

  /**
   * get_xsa_name() - Get Xilinx Support Archive (XSA) Name of xclbin
   *
   * @return 
   *  Name of XSA
   *
   * An exception is thrown if the data is missing.
   */
  XCL_DRIVER_DLLESPEC
  std::string
  get_xsa_name() const;

  /**
   * get_uuid() - Get the uuid of the xclbin
   *
   * @return 
   *  UUID of xclbin
   *
   * An exception is thrown if the data is missing.
   */
  XCL_DRIVER_DLLESPEC
  uuid
  get_uuid() const;

  /**
   * get_data() - Get the raw data of the xclbin
   *
   * @return 
   *  The raw data of the xclbin
   *
   * An exception is thrown if the data is missing.
   */
  XCL_DRIVER_DLLESPEC
  const std::vector<char>&
  get_data() const;
};

std::ostream& operator<<(std::ostream& os, const xclbin& xclbin);
std::ostream& operator<<(std::ostream& os, const xclbin::kernel& kernel);
std::ostream& operator<<(std::ostream& os, const xclbin::cu& cu);
std::ostream& operator<<(std::ostream& os, const xclbin::arg& arg);
std::ostream& operator<<(std::ostream& os, const xclbin::mem& mem);
} // namespace xrt

/// @cond
extern "C" {
#endif

/**
 * xrtXclbinAllocFilename() - Allocate a xclbin using xclbin filename
 *
 * @filename:      path to the xclbin file
 * Return:         xrtXclbinHandle on success or NULL with errno set
 */
XCL_DRIVER_DLLESPEC
xrtXclbinHandle
xrtXclbinAllocFilename(const char* filename);


/**
 * xrtXclbinAllocAxlf() - Allocate a xclbin using an axlf
 *
 * @top_axlf:      an axlf
 * Return:         xrtXclbinHandle on success or NULL with errno set
 */
XCL_DRIVER_DLLESPEC
xrtXclbinHandle
xrtXclbinAllocAxlf(const axlf* top_axlf);

/**
 * xrtXclbinAllocRawData() - Allocate a xclbin using raw data
 *
 * @data:          raw data buffer of xclbin
 * @size:          size (in bytes) of raw data buffer of xclbin
 * Return:         xrtXclbinHandle on success or NULL with errno set
 */
XCL_DRIVER_DLLESPEC
xrtXclbinHandle
xrtXclbinAllocRawData(const char* data, int size);

/**
 * xrtXclbinFreeHandle() - Deallocate the xclbin handle
 *
 * @xhdl:          xclbin handle
 * Return:         0 on success, -1 on error
 */
XCL_DRIVER_DLLESPEC
int
xrtXclbinFreeHandle(xrtXclbinHandle xhdl);

/**
 * xrtXclbinGetXSAName() - Get Xilinx Support Archive (XSA) Name of xclbin handle
 *
 * @xhdl:       Xclbin handle
 * @name:       Return name of XSA.
 *              If the value is nullptr, the content of this value will not be populated.
 *              Otherwise, the the content of this value will be populated.
 * @size:       size (in bytes) of @name.
 * @ret_size:   Return size (in bytes) of XSA name.
 *              If the value is nullptr, the content of this value will not be populated.
 *              Otherwise, the the content of this value will be populated.
 * Return:  0 on success or appropriate error number
 */
XCL_DRIVER_DLLESPEC
int
xrtXclbinGetXSAName(xrtXclbinHandle xhdl, char* name, int size, int* ret_size);

/**
 * xrtXclbinGetUUID() - Get UUID of xclbin handle
 *
 * @xhdl:     Xclbin handle
 * @ret_uuid: Return xclbin id in this uuid_t struct
 * Return:    0 on success or appropriate error number
 */
XCL_DRIVER_DLLESPEC
int
xrtXclbinGetUUID(xrtXclbinHandle xhdl, xuid_t ret_uuid);

/**
 * xrtXclbinGetData() - Get the raw data of the xclbin handle
 *
 * @xhdl:       Xclbin handle
 * @data:       Return raw data.
 *              If the value is nullptr, the content of this value will not be populated.
 *              Otherwise, the the content of this value will be populated.
 * @size:       Size (in bytes) of @data
 * @ret_size:   Return size (in bytes) of XSA name.
 *              If the value is nullptr, the content of this value will not be populated.
 *              Otherwise, the the content of this value will be populated.
 * Return:  0 on success or appropriate error number
 */
XCL_DRIVER_DLLESPEC
int
xrtXclbinGetData(xrtXclbinHandle xhdl, char* data, int size, int* ret_size);

/*
 * xrtGetXclbinUUID() - Get UUID of xclbin image running on device
 *
 * @dhdl:   Device handle
 * @out:    Return xclbin id in this uuid_t struct
 * Return:  0 on success or appropriate error number
 */
XCL_DRIVER_DLLESPEC
int
xrtXclbinUUID(xclDeviceHandle dhdl, xuid_t out);

/// @endcond
#ifdef __cplusplus
}
#endif

#endif

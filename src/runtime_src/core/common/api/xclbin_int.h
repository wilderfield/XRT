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

#ifndef _XRT_COMMON_XCLBIN_INT_H_
#define _XRT_COMMON_XCLBIN_INT_H_

// This file defines implementation extensions to the XRT XCLBIN APIs.
#include "core/include/experimental/xrt_xclbin.h"

namespace xrt_core {
namespace xclbin_int {

/**
 * get_xclbin_data() - Returns the data of the xrtXclbinHandle handle.
 *
 * @handle:        Xclbin handle
 * Return:         Data of the @handle
 *
 * Throws if @handle is invalid.
 */
const std::vector<char>&
get_xclbin_data(xrtXclbinHandle handle);

/**
 * get_axlf_section() - Get section from currently loaded axlf
 *
 * @my_xclbin:  Xclbin object to operate on
 * @section:    The desired section
 * @xclbin_id:  The uuid that corresponds to my_xclbin
 * Return:      Pair of section data and size in bytes
 */
std::pair<const char*, size_t>
get_axlf_section(const ::xrt::xclbin& my_xclbin, axlf_section_kind section, const uuid& xclbin_id = uuid());

/**
 * get_axlf_section() - Get section from currently loaded axlf, throws if section not found
 *
 * @my_xclbin:  Xclbin object to operate on
 * @section:    The desired section
 * @xclbin_id:  The uuid that corresponds to my_xclbin
 * Return:      Pair of section data and size in bytes
 */
std::pair<const char*, size_t>
get_axlf_section_or_error(const ::xrt::xclbin& my_xclbin, axlf_section_kind section, const uuid& xclbin_id = uuid());

/**
 * get_section_kinds() - Iterates over enum
 *
 * Return:     Vector of all possible section kinds
 *
 */
std::vector<axlf_section_kind>
get_section_kinds();

/**
 * create_xclbin() - Creates an xclbin object that only caches specified sections
 *  by default all present sections are cached
 *
 * @top:       raw data of xclbin
 * @kinds:     list of axlf_section_kind to cache
 * Return:     xclbin object
 *
 */
::xrt::xclbin
create_xclbin(const axlf* top, const std::vector<axlf_section_kind>& kinds = get_section_kinds());

/**
 * create_xclbin() - creates a xclbin object
 *  by default all present sections are cached
 *
 * @device:    ptr to xrt_core::device
 * @kinds:     list of axlf_section_kind to cache
 * Return:     xclbin object
 *
 */
::xrt::xclbin
create_xclbin(const xrt_core::device* device, const std::vector<axlf_section_kind>& kinds = get_section_kinds());

/**
 * read_xclbin() - Reads xclbin file as binary
 *
 * Return:     Vector of char
 *
 */
std::vector<char>
read_xclbin(const std::string& fnm);

/**
 * Utility to get uuid from xclbin -
 */
uuid
get_uuid(const axlf* top);

/**
 * header_section_iterator: iterator to traverse axlf_section_header
 */
class header_section_iterator // const behavior intended
{
public:
  explicit header_section_iterator(const axlf_section_header* ptr) : m_ptr(const_cast<axlf_section_header*>(ptr)) {}
  header_section_iterator operator++() { auto i = *this; m_ptr++; return i; }
  header_section_iterator operator++(int) { m_ptr++; return *this; }
  const axlf_section_header& operator*() { return *m_ptr; }
  const axlf_section_header* operator->() { return m_ptr; }
  bool operator==(const header_section_iterator& rhs) { return m_ptr == rhs.m_ptr; }
  bool operator!=(const header_section_iterator& rhs) { return m_ptr != rhs.m_ptr; }
private:
  axlf_section_header* m_ptr;
};

/**
 * axlf_header_container: header section container for axlf
 */
class axlf_header_container
{
public:
  explicit axlf_header_container(const axlf* top) : m_begin(top->m_sections), m_end(&(top->m_sections[top->m_header.m_numSections])) {}
  explicit axlf_header_container(const axlf& top) : m_begin(top.m_sections), m_end(&(top.m_sections[top.m_header.m_numSections])) {}
  header_section_iterator begin() { return m_begin; }
  header_section_iterator end() { return m_end; }
  header_section_iterator begin() const { return m_begin; }
  header_section_iterator end() const { return m_end; }
private:
  header_section_iterator m_begin;
  header_section_iterator m_end;
};

} //xclbin_int
}; // xrt_core

#endif

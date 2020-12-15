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

// This file implements XRT xclbin APIs as declared in
// core/include/experimental/xrt_xclbin.h
#define XCL_DRIVER_DLL_EXPORT  // exporting xrt_xclbin.h
#define XRT_CORE_COMMON_SOURCE // in same dll as core_common
#include "core/include/experimental/xrt_xclbin.h"

#include "core/common/system.h"
#include "core/common/device.h"
#include "core/common/message.h"
#include "core/common/query_requests.h"
#include "core/include/xclbin.h"
#include "core/common/xclbin_parser.h"
#include "xclbin_int.h"
#include <iostream>
#include <fstream>
#include <unordered_set>

#ifdef _WIN32
# include "windows/uuid.h"
# pragma warning( disable : 4244 4267 4996)
#else
# include <linux/uuid.h>
#endif

namespace xrt {

/**
 * class xclbin::mem_impl - xrt::xclbin::mem_impl provides the implementation
 *
 *   for a device memory object.
 *
 */
class xclbin::mem_impl
{
public:
  mem_impl(const struct mem_data& memdata, const int32_t memidx);

  const std::string&
  get_name() const{
    return m_name;
  }
  
  uint64_t
  get_base_address() const
  {
    return m_base_address;
  }
  
  uint64_t
  get_size() const
  {
    return m_size;
  }

  uint8_t
  get_used() const
  {
    return m_used;
  }

  uint8_t
  get_type() const
  {
    return m_type;
  }

  int32_t
  get_index() const
  {
    return m_index;
  }

protected:
  std::string m_name;
  uint64_t m_base_address;
  uint64_t m_size;
  uint8_t m_used;
  uint8_t m_type;
  int32_t m_index;
};
  
/**
 * class xclbin::mem - xrt::xclbin::mem provides public APIs
 *
 *   for a device memory object.
 *
 */
const std::string&
xclbin::mem::get_name() const{
    return handle->get_name();
}

uint64_t
xclbin::mem::get_base_address() const
{
	return handle->get_base_address();
}

uint64_t
xclbin::mem::get_size() const
{
	return handle->get_size();
}

uint8_t
xclbin::mem::get_used() const
{
	return handle->get_used();
}

uint8_t
xclbin::mem::get_type() const
{
	return handle->get_type();
}

int32_t
xclbin::mem::get_index() const
{
  return handle->get_index();
}

/**
 * class xclbin::arg_impl - xrt::xclbin::arg_impl provides the implementation
 *
 *   for a kernel/cu argument.
 *
 */
class xclbin::arg_impl
{
public:
  arg_impl(const struct xrt_core::xclbin::kernel_argument& karg);
  arg_impl(const struct xrt_core::xclbin::kernel_argument& karg, const struct ip_data& cu, const xclbin_impl* xclbin);
  
  const std::string&
  get_name() const {
    return m_name;
  }
  std::vector<xclbin::mem>
  get_mems() const {
    return m_mems;
  }

  xclbin::mem
  get_mems(int32_t index) const {
    return m_mems[index];
  }

protected:
  std::string m_name;
  std::vector<xclbin::mem> m_mems;
};
  
/**
 * class xclbin::arg - xrt::xclbin::arg provides public APIs
 *
 *   for an arg object.
 *
 */
const std::string&
xclbin::arg::get_name() const {
  return handle->get_name();
}

std::vector<xclbin::mem>
xclbin::arg::get_mems() const
{
  return handle->get_mems();
}
xclbin::mem
xclbin::arg::get_mems(int32_t index) const
{
  return handle->get_mems(index);
}

/**
 * class xclbin::cu_impl - xrt::xclbin::cu_impl provides the implementation
 *
 *   for a cu.
 *
 */
class xclbin::cu_impl
{
public:
  cu_impl(const ip_data* cu, const xclbin_impl* xclbin);

  const std::string&
  get_name() const {
    return m_name;
  }
    
  std::vector<xclbin::arg>
  get_args() const {
    return m_args;
  }
  
  xclbin::arg
  get_args(int32_t index) const {
    return m_args[index];
  }

  uint64_t
  get_base_address() const {
    return m_base_address;
  }

protected:
  std::string m_name;
  uint64_t m_base_address;
  std::vector<xclbin::arg> m_args;
};

/**
 * class xclbin::cu - xrt::xclbin::cu provides public APIs
 *
 *   for a cu object.
 *
 */
const std::string&
xclbin::cu::get_name() const
{
  return handle->get_name();
}
    
std::vector<xclbin::arg>
xclbin::cu::get_args() const {
  return handle->get_args();
}

xclbin::arg
xclbin::cu::get_args(int32_t index) const {
  return handle->get_args(index);
}

uint64_t
xclbin::cu::get_base_address() const {
  return handle->get_base_address();
}

/**
 * class xclbin::kernel_impl - xrt::xclbin::kernel_impl provides the implementation
 *
 *   for a kernel.
 *
 */
class xclbin::kernel_impl
{
public:
  kernel_impl(const struct xrt_core::xclbin::kernel_object& kernel, const xclbin_impl* xclbin);
  
  const std::string&
  get_name() const {
    return m_name;
  }

  std::vector<xclbin::cu>
  get_cus() const {
    return m_cus;
  }
  
  xclbin::cu
  get_cus(int32_t index) const {
    return m_cus[index];
  }
    
  std::vector<xclbin::arg>
  get_args() const {
    return m_args;
  }
  
  xclbin::arg
  get_args(int32_t index) const {
    return m_args[index];
  }

protected:
  std::string m_name;
  std::vector<xclbin::cu> m_cus;
  std::vector<xclbin::arg> m_args;
};

/**
 * class xclbin::kernel - xrt::xclbin::kernel provides public APIs
 *
 *   for a kernel object.
 *
 */
const std::string&
xclbin::kernel::get_name() const
{
  return handle->get_name();
}

std::vector<xclbin::cu>
xclbin::kernel::get_cus() const {
  return handle->get_cus();
}

xclbin::cu
xclbin::kernel::get_cus(int32_t index) const {
  return handle->get_cus(index);
}
    
std::vector<xclbin::arg>
xclbin::kernel::get_args() const {
  return handle->get_args();
}

xclbin::arg
xclbin::kernel::get_args(int32_t index) const {
  return handle->get_args(index);
}

/**
 * class xclbin_info - This is a shadow class
 *
 *   It allows an xclbin to be composed of kernels and memories
 *     if and only if it is necessary to compute them
 *
 */
class xclbin_info
{
public:
  xclbin_info(const xclbin_impl* xclbin);
  
  std::vector<xclbin::kernel>
  get_kernels() const;
  
  xclbin::kernel
  get_kernels(int32_t index) const;

protected:
  std::vector<xclbin::kernel> m_kernels;
  std::vector<xclbin::mem> m_mems;
};

std::vector<xclbin::kernel>
xclbin_info::get_kernels() const
{
  return m_kernels;
}

xclbin::kernel
xclbin_info::get_kernels(int32_t index) const
{
  return m_kernels[index];
}

/**
 * class xclbin_impl - xrt::xclbin_impl provides the implementation
 *   for an xclbin object. It is actually an abstract base class, 
 *   and it is extended by xclbin_full xclbin_filter and xclbin_dquery classes
 *   full - caches whole xclbin
 *   filter - caches sections of interest
 *   dquery - constructed by querying sections from the device driver
 *     necessary for processes that didn't load the xclbin
 *
 */
class xclbin_impl
{
public:

  virtual ~xclbin_impl() = default;

  virtual const std::vector<char>& get_data() const = 0;

  virtual
  std::pair<const char*, size_t>
  get_axlf_section(axlf_section_kind section, const uuid& xclbin_id) const = 0;
  
  template <typename SectionType>
  SectionType
  get_section(axlf_section_kind section, const uuid& uuid) const
  {
    return reinterpret_cast<SectionType>(get_axlf_section(section, uuid).first);
  }
  
  template <typename SectionType>
  SectionType
  get_section_or_error(axlf_section_kind section, const uuid& uuid) const
  {
    SectionType ptr = reinterpret_cast<SectionType>(get_axlf_section(section, uuid).first);
    if(ptr == nullptr)
      throw std::runtime_error("Section " + std::to_string(section) + " does not exist");
    return ptr;
  }

  virtual
  std::string
  get_xsa_name() const
  {
    return m_vbnv;
  }

  virtual
  uuid
  get_uuid() const
  {
    return m_uuid;
  }

  virtual
  const xclbin_info*
  get_info() const
  {
    if (m_info == nullptr)
      m_info = std::make_unique<xclbin_info>(this);
    return m_info.get();
  }

  virtual
  std::vector<xclbin::kernel>
  get_kernels() const
  {
    return get_info()->get_kernels();
  }
  
  virtual
  xclbin::kernel
  get_kernels(int32_t index) const
  {
    return get_info()->get_kernels(index);
  }
  
protected:
  mutable std::unique_ptr<xclbin_info> m_info;
  std::string m_vbnv;
  uuid m_uuid; // Must cache to secure get_axlf_sections()
};

/* xclbin_info constructor */
xclbin_info::xclbin_info(const xclbin_impl* xclbin)
{
  // xml section for kernel names
  auto xml_section = xclbin->get_axlf_section(EMBEDDED_METADATA, xclbin->get_uuid());
  if (!xml_section.first)
    return;
    // throw std::runtime_error("No xml metadata available");
  
  auto kernels = 
    xrt_core::xclbin::get_kernels(xml_section.first,xml_section.second);

  for (auto& kernel : kernels) {
    m_kernels.emplace_back(
      std::make_shared<xclbin::kernel_impl>(kernel,xclbin)
    );
  }
}

/* kernel_impl constructor */
xclbin::kernel_impl::kernel_impl(const struct xrt_core::xclbin::kernel_object& kernel, const xclbin_impl* xclbin)
  : m_name(kernel.name)
{
  // populate m_args
  for (auto& arg : kernel.args) {
    m_args.emplace_back(
      std::make_shared<xclbin::arg_impl>(arg)
    );
  }
 
  // ip_layout section for collecting CUs
  auto ip_layout = xclbin->get_section<const ::ip_layout*>(IP_LAYOUT, xclbin->get_uuid());
  if (!ip_layout)
    return;

  // populate m_cus
  // This will ignore streaming cus, is that okay?
  auto cus = 
    xrt_core::xclbin::get_cus(ip_layout,kernel.name);
  for (auto& cu : cus) {
    m_cus.emplace_back(
      std::make_shared<xclbin::cu_impl>(cu, xclbin)
    );
  }

  // Sort
  std::sort(m_cus.begin(), m_cus.end(), [] (xrt::xclbin::cu const& lhs, xrt::xclbin::cu const& rhs) -> bool
  {
      return lhs.get_base_address() < rhs.get_base_address();
  });
}

/* arg_impl constructor */
xclbin::arg_impl::arg_impl(const struct xrt_core::xclbin::kernel_argument& karg)
  : m_name(karg.name) {} 

/* arg_impl constructor */
xclbin::arg_impl::arg_impl(const struct xrt_core::xclbin::kernel_argument& karg, const struct ip_data& cu, const xclbin_impl* xclbin)
  : m_name(karg.name)
{
  auto ip_layout =
    xclbin->get_section_or_error<const ::ip_layout*>(IP_LAYOUT, xclbin->get_uuid());
  // connectivity section (details connections between CU arguments and memories)
  auto connectivity =
    xclbin->get_section_or_error<const ::connectivity*>(ASK_GROUP_CONNECTIVITY, xclbin->get_uuid());
  auto mem_topology =
    xclbin->get_section_or_error<const ::mem_topology*>(ASK_GROUP_TOPOLOGY, xclbin->get_uuid());

  // iterate connectivity and look for matching [cuaddr,arg] pair
  for (int32_t i=0; i<connectivity->m_count; ++i) {
    if (connectivity->m_connection[i].arg_index!=static_cast<int32_t>(karg.index))
      continue;
    auto ipidx = connectivity->m_connection[i].m_ip_layout_index;
    if (ip_layout->m_ip_data[ipidx].m_base_address!=cu.m_base_address)
      continue;

    // found the connection that match cuaddr,arg

    int32_t memidx = connectivity->m_connection[i].mem_data_index;
    assert(mem_topology->m_mem_data[memidx].m_used);
    m_mems.emplace_back (
      std::make_shared<xclbin::mem_impl>(mem_topology->m_mem_data[memidx], memidx)
      );
  }
}

/* mem_impl constructor */
xclbin::mem_impl::mem_impl(const struct mem_data& m, const int32_t memidx)
  : m_name(reinterpret_cast<const char*>(m.m_tag)),
    m_base_address(m.m_base_address),
    m_size(m.m_size),
    m_used(m.m_used),
    m_type(m.m_type),
    m_index(memidx) {}

/* cu_impl constructor */
xclbin::cu_impl::cu_impl(const ip_data* cu, const xclbin_impl* xclbin)
{

  std::string ip_name = reinterpret_cast<const char*>(cu->m_name);
  std::string kernel_name = ip_name.substr(0,ip_name.find(":"));
  m_name = ip_name.substr(ip_name.find(":")+1,ip_name.length());

  auto addr = cu->m_base_address;
  if (addr == static_cast<size_t>(-1))
    addr = std::numeric_limits<size_t>::max() & ~0xFF;

  m_base_address = addr;

  auto xml_section = xclbin->get_axlf_section(EMBEDDED_METADATA, xclbin->get_uuid());
  if (!xml_section.first)
    throw std::runtime_error("No xml metadata available");
 

  auto args = 
    xrt_core::xclbin::get_kernel_arguments(xml_section.first,xml_section.second,kernel_name);

  for (const auto& arg : args) {
    m_args.emplace_back(
      std::make_shared<xclbin::arg_impl>(arg,*cu,xclbin)
    );
  }
}

/* class xclbin_full - class for xclbin objects
 *   This implementation stores the full buffer
 * Life time of xclbin are managed through shared pointers.
 * A buffer is freed when last references is released.
 */
class xclbin_full : public xclbin_impl
{
public:

  explicit xclbin_full(const std::string& filename)
    : xclbin_full(::xrt_core::xclbin_int::read_xclbin(filename))
  {}

  explicit xclbin_full(std::vector<char>& data)
  {
    m_axlf = std::move(data);
    if (strncmp(get_top()->m_magic, "xclbin2", 7) != 0)
      throw std::runtime_error("Invalid xclbin");

    m_uuid = uuid(get_top()->m_header.uuid);

    auto vbnv = reinterpret_cast<const char*>(get_top()->m_header.m_platformVBNV);
    m_vbnv = std::string(vbnv, strnlen(vbnv, 64));

    // Build Map
    for (const auto& section_header : xrt_core::xclbin_int::axlf_header_container(get_top())) {
      auto kind = static_cast<axlf_section_kind>(section_header.m_sectionKind);
      auto section_data = reinterpret_cast<const char*>(get_top()) + section_header.m_sectionOffset;
      m_axlf_sections.emplace(kind, std::make_pair(section_data, section_header.m_sectionSize));
    }

    /* If group topology or group connectivity doesn't exists then use the
     * mem topology or connectivity respectively section for the same.
     */
    if (m_axlf_sections.find(ASK_GROUP_TOPOLOGY) == m_axlf_sections.end() &&
        m_axlf_sections.find(MEM_TOPOLOGY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_TOPOLOGY] = m_axlf_sections[MEM_TOPOLOGY];
    }
    if (m_axlf_sections.find(ASK_GROUP_CONNECTIVITY) == m_axlf_sections.end() &&
        m_axlf_sections.find(CONNECTIVITY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_CONNECTIVITY] = m_axlf_sections[CONNECTIVITY];
    }
  }

  explicit xclbin_full(const std::vector<char>& data)
  {
    m_axlf = data; // copy
    if (strncmp(get_top()->m_magic, "xclbin2", 7) != 0)
      throw std::runtime_error("Invalid xclbin");

    m_uuid = uuid(get_top()->m_header.uuid);

    auto vbnv = reinterpret_cast<const char*>(get_top()->m_header.m_platformVBNV);
    m_vbnv = std::string(vbnv, strnlen(vbnv, 64));

    // Build Map
    for (const auto& section_header : xrt_core::xclbin_int::axlf_header_container(get_top())) {
      auto kind = static_cast<axlf_section_kind>(section_header.m_sectionKind);
      auto section_data = reinterpret_cast<const char*>(get_top()) + section_header.m_sectionOffset;
      m_axlf_sections.emplace(kind, std::make_pair(section_data, section_header.m_sectionSize));
    }

    /* If group topology or group connectivity doesn't exists then use the
     * mem topology or connectivity respectively section for the same.
     */
    if (m_axlf_sections.find(ASK_GROUP_TOPOLOGY) == m_axlf_sections.end() &&
        m_axlf_sections.find(MEM_TOPOLOGY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_TOPOLOGY] = m_axlf_sections[MEM_TOPOLOGY];
    }
    if (m_axlf_sections.find(ASK_GROUP_CONNECTIVITY) == m_axlf_sections.end() &&
        m_axlf_sections.find(CONNECTIVITY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_CONNECTIVITY] = m_axlf_sections[CONNECTIVITY];
    }
  }

  explicit xclbin_full(const axlf* top)
  {
    if (strncmp(get_top()->m_magic, "xclbin2", 7))
      throw std::runtime_error("Invalid xclbin");

    auto length = top->m_header.m_length;
    m_axlf.resize(length);
    auto top_c = reinterpret_cast<const char*>(top);
    std::copy(top_c,top_c+length,m_axlf.begin());

    m_uuid = uuid(get_top()->m_header.uuid);

    auto vbnv = reinterpret_cast<const char*>(get_top()->m_header.m_platformVBNV);
    m_vbnv = std::string(vbnv, strnlen(vbnv, 64));

    // Build Map
    for (const auto& section_header : xrt_core::xclbin_int::axlf_header_container(get_top())) {
      auto kind = static_cast<axlf_section_kind>(section_header.m_sectionKind);
      auto section_data = reinterpret_cast<const char*>(get_top()) + section_header.m_sectionOffset;
      m_axlf_sections.emplace(kind, std::make_pair(section_data, section_header.m_sectionSize));
    }

    /* If group topology or group connectivity doesn't exists then use the
     * mem topology or connectivity respectively section for the same.
     */
    if (m_axlf_sections.find(ASK_GROUP_TOPOLOGY) == m_axlf_sections.end() &&
        m_axlf_sections.find(MEM_TOPOLOGY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_TOPOLOGY] = m_axlf_sections[MEM_TOPOLOGY];
    }
    if (m_axlf_sections.find(ASK_GROUP_CONNECTIVITY) == m_axlf_sections.end() &&
        m_axlf_sections.find(CONNECTIVITY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_CONNECTIVITY] = m_axlf_sections[CONNECTIVITY];
    }
  }

  const std::vector<char>& get_data() const override
  {
    return m_axlf;
  }

  std::pair<const char*, size_t>
  get_axlf_section(axlf_section_kind section, const uuid& xclbin_id) const override
  {
    if (xclbin_id && xclbin_id != m_uuid)
      throw std::runtime_error("xclbin id mismatch");
    auto itr = m_axlf_sections.find(section);
    return itr != m_axlf_sections.end()
           ? (*itr).second
           : std::make_pair(nullptr, size_t(0));
  }

protected:
  std::vector<char> m_axlf;
  std::map<axlf_section_kind, std::pair<const char*, size_t>> m_axlf_sections;

  axlf*
  get_top() const
  {
    return reinterpret_cast<axlf*>(const_cast<char*>(m_axlf.data()));
  }
};

/* class xclbin_filter - class for xclbin objects
 *   This version stores only sections of interest
 * Life time of xclbin are managed through shared pointers.
 * A buffer is freed when last references is released.
 */
class xclbin_filter : public xclbin_impl
{
public:
  explicit xclbin_filter(const axlf* top, const std::vector<axlf_section_kind>& kinds = ::xrt_core::xclbin_int::get_section_kinds())
  {
    if (strncmp(top->m_magic, "xclbin2", 7) != 0)
      throw std::runtime_error("Invalid xclbin");

    m_uuid = uuid(top->m_header.uuid);

    auto vbnv = reinterpret_cast<const char*>(top->m_header.m_platformVBNV);
    m_vbnv = std::string(vbnv, strnlen(vbnv, 64));

    m_axlf_sections.clear();

    // Convert kinds to set
    std::unordered_set<axlf_section_kind> kinds_set(kinds.begin(),kinds.end());

    // Copy all section data of interest
    for (const auto& section_header : xrt_core::xclbin_int::axlf_header_container(top)) {
      auto kind = static_cast<axlf_section_kind>(section_header.m_sectionKind);
      if (kinds_set.find(kind) != kinds_set.end()) {
        auto section_data = reinterpret_cast<const char*>(top) + section_header.m_sectionOffset;
        m_axlf_sections[kind] = std::vector<char>(section_data,section_data+section_header.m_sectionSize);
      }
    }

    /* If group topology or group connectivity doesn't exists then use the
     * mem topology or connectivity respectively section for the same.
     */
    if (m_axlf_sections.find(ASK_GROUP_TOPOLOGY) == m_axlf_sections.end() &&
        m_axlf_sections.find(MEM_TOPOLOGY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_TOPOLOGY] = m_axlf_sections[MEM_TOPOLOGY];
    }
    if (m_axlf_sections.find(ASK_GROUP_CONNECTIVITY) == m_axlf_sections.end() &&
        m_axlf_sections.find(CONNECTIVITY) != m_axlf_sections.end()) {
      m_axlf_sections[ASK_GROUP_CONNECTIVITY] = m_axlf_sections[CONNECTIVITY];
    }
  }

  const std::vector<char>& get_data() const override
  {
    throw std::runtime_error("get_data() is not implemented for this xclbin object");
  }

  std::pair<const char*, size_t>
  get_axlf_section(axlf_section_kind section, const uuid& xclbin_id) const override
  {
    if (xclbin_id && xclbin_id != m_uuid)
      throw std::runtime_error("xclbin id mismatch");
    auto itr = m_axlf_sections.find(section);
    return itr != m_axlf_sections.end()
           ? std::make_pair((*itr).second.data(),(*itr).second.size())
           : std::make_pair(nullptr, size_t(0));
  }

protected:
  std::map<axlf_section_kind, std::vector<char>> m_axlf_sections;
};

/* class xclbin_dquery - class for xclbin objects
 *   This version is constructed via device driver queries
 *   that occur after the xclbin has been loaded by another process
 * Life time of xclbin are managed through shared pointers.
 * A buffer is freed when last references is released.
 */
class xclbin_dquery : public xclbin_impl
{
public:
  explicit xclbin_dquery(const xrt_core::device* device, const std::vector<axlf_section_kind>& kinds = ::xrt_core::xclbin_int::get_section_kinds())
  {
    // Strive to match below
    /*
    const std::vector<axlf_section_kind> kinds =
      { EMBEDDED_METADATA, AIE_METADATA, IP_LAYOUT, CONNECTIVITY,
        ASK_GROUP_CONNECTIVITY, ASK_GROUP_TOPOLOGY,
        MEM_TOPOLOGY, DEBUG_IP_LAYOUT, SYSTEM_METADATA, CLOCK_FREQ_TOPOLOGY
      };
    */
    m_axlf_sections.clear();

    m_vbnv = xrt_core::device_query<xrt_core::query::rom_vbnv>(device); // Can't get xclbin vbnv
    m_uuid = uuid(xrt_core::device_query<xrt_core::query::xclbin_uuid>(device));

    m_axlf_sections[IP_LAYOUT] = xrt_core::device_query<xrt_core::query::ip_layout_raw>(device);

    m_axlf_sections[MEM_TOPOLOGY] = xrt_core::device_query<xrt_core::query::mem_topology_raw>(device);

    m_axlf_sections[CLOCK_FREQ_TOPOLOGY] = xrt_core::device_query<xrt_core::query::clock_freq_topology_raw>(device);

    m_axlf_sections[ASK_GROUP_TOPOLOGY] = xrt_core::device_query<xrt_core::query::group_topology>(device);

  }

  const std::vector<char>& get_data() const override
  {
    throw std::runtime_error("get_data() is not implemented for this xclbin object");
  }

  std::pair<const char*, size_t>
  get_axlf_section(axlf_section_kind section, const uuid& xclbin_id) const override
  {
    if (xclbin_id && xclbin_id != m_uuid)
      throw std::runtime_error("xclbin id mismatch");
    auto itr = m_axlf_sections.find(section);
    return itr != m_axlf_sections.end()
           ? std::make_pair((*itr).second.data(),(*itr).second.size())
           : std::make_pair(nullptr, size_t(0));
  }

protected:
  std::map<axlf_section_kind, std::vector<char>> m_axlf_sections;
};

/**
 * class xclbin - xrt::xclbin provides public APIs
 *
 *   for an xclbin object.
 *
 */
xclbin::xclbin(const std::string& filename)
  : Pimpl(std::make_shared<xclbin_full>(filename))
{}

xclbin::xclbin(const std::vector<char>& data)
  : Pimpl(std::make_shared<xclbin_full>(data))
{}

xclbin::xclbin(const axlf* top)
  : Pimpl(std::make_shared<xclbin_full>(top))
{}

std::vector<xclbin::kernel>
xclbin::get_kernels() const
{
  return handle->get_kernels();
}

xclbin::kernel
xclbin::get_kernels(int32_t index) const
{
  return handle->get_kernels(index);
}

std::string
xclbin::get_xsa_name() const
{
  return handle->get_xsa_name();
}

uuid
xclbin::get_uuid() const
{
  return handle->get_uuid();
}

const std::vector<char>&
xclbin::get_data() const
{
  return handle->get_data();
}

std::ostream& operator<<(std::ostream& os, const xclbin& xclbin)
{
  os << "Xclbin:" << std::endl;
  os << "  XSA Name: " << xclbin.get_xsa_name() << std::endl;
  os << "  UUID: " << xclbin.get_uuid().to_string();
  if (xclbin.get_kernels().size() > 0)
    os << std::endl;
  bool separate = false;
  for (const auto& kernel : xclbin.get_kernels()) {
    if (separate)
      os << std::endl;
    os << kernel;
    separate = true;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const xclbin::kernel& kernel)
{
  os << "Kernel: " << kernel.get_name();
  if (kernel.get_args().size() > 0)
    os << std::endl;
  bool separate = false;
  for (auto& arg : kernel.get_args()) {
    if (separate)
      os << std::endl;
    os << arg;
    separate = true;
  }

  if (kernel.get_cus().size() > 0)
    os << std::endl;
  separate = false;
  for (auto& cu : kernel.get_cus()) {
    if (separate)
      os << std::endl;
    os << cu;
    separate = true;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const xclbin::cu& cu)
{
  os << "Compute Unit: " << cu.get_name() << std::endl;
  os << "  BaseAddress: " << "0x" << std::hex << cu.get_base_address();
  if (cu.get_args().size() > 0)
    os << std::endl;
  bool separate = false;
  for (auto& arg : cu.get_args()) {
    if (separate)
      os << std::endl;
    os << arg;
    separate = true;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const xclbin::arg& arg)
{
  os << "Argument: " << arg.get_name();
  if (arg.get_mems().size() > 0)
    os << std::endl;
  bool separate = false;
  for (auto& mem : arg.get_mems()) {
    if (separate)
      os << std::endl;
    os << mem;
    separate = true;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const xclbin::mem& mem)
{
  os << "Memory: " << mem.get_name() << std::endl;
  os << "  Base Address: " << "0x" << std::hex << mem.get_base_address() << std::endl;
  os << "  Index: " << mem.get_index();
  return os;
}

} // namespace xrt

namespace {

// C-API handles that must be explicitly freed. Corresponding managed
// handles are inserted in this map.  When the unmanaged handle is
// freed, it is removed from this map and underlying object is
// deleted if no other shared ptrs exists for this xclbin object
static std::map<xrtXclbinHandle, std::shared_ptr<xrt::xclbin_impl>> xclbins;

static std::shared_ptr<xrt::xclbin_impl>
get_xclbin(xrtXclbinHandle handle)
{
  auto itr = xclbins.find(handle);
  if (itr == xclbins.end())
    throw xrt_core::error(-EINVAL, "No such xclbin handle");
  return itr->second;
}

static void
free_xclbin(xrtXclbinHandle handle)
{
  if (xclbins.erase(handle) == 0)
    throw xrt_core::error(-EINVAL, "No such xclbin handle");
}

inline void
send_exception_message(const char* msg)
{
  xrt_core::message::send(xrt_core::message::severity_level::error, "XRT", msg);
}

} // namespace

////////////////////////////////////////////////////////////////
// xrt_xclbin implementation of extension APIs not exposed to end-user
// 
// Utility function for device class to verify that the C xclbin
// handle is valid Needed when the C API for device tries to load an
// xclbin using C pointer to xclbin
////////////////////////////////////////////////////////////////
namespace xrt_core { namespace xclbin_int {

const std::vector<char>&
get_xclbin_data(xrtXclbinHandle handle)
{
  return get_xclbin(handle)->get_data();
}

std::pair<const char*, size_t>
get_axlf_section(const ::xrt::xclbin& my_xclbin, axlf_section_kind section, const uuid& xclbin_id)
{
  if (my_xclbin.get_handle() == nullptr)
	throw std::runtime_error("xclbin is not valid");
  if (xclbin_id && xclbin_id != my_xclbin.get_uuid())
    throw std::runtime_error("xclbin id mismatch");
  return my_xclbin.get_handle()->get_axlf_section(section, xclbin_id);
}

std::pair<const char*, size_t>
get_axlf_section_or_error(const ::xrt::xclbin& my_xclbin, axlf_section_kind section, const uuid& xclbin_id)
{
  auto ret = get_axlf_section(my_xclbin, section, xclbin_id);
  if (ret.first != nullptr)
    return ret;
  throw std::runtime_error("no such xclbin section");
}

::xrt::xclbin
create_xclbin(const axlf* top, const std::vector<axlf_section_kind>& kinds){
  return {std::make_shared<xrt::xclbin_filter>(top, kinds)};
}

::xrt::xclbin
create_xclbin(const device* device, const std::vector<axlf_section_kind>& kinds){
  return {std::make_shared<xrt::xclbin_dquery>(device, kinds)};
}

std::vector<axlf_section_kind>
get_section_kinds()
{
  std::vector<axlf_section_kind> all_kinds;
  for(int kind = BITSTREAM;kind != ASK_LAST_SECTION_KIND; ++kind)
    all_kinds.push_back(static_cast<axlf_section_kind>(kind));

  return all_kinds;
}

std::vector<char>
read_xclbin(const std::string& fnm)
{
  if (fnm.empty())
    throw std::runtime_error("No xclbin specified");

  // load the file
  std::ifstream stream(fnm);
  if (!stream)
    throw std::runtime_error("Failed to open file '" + fnm + "' for reading");

  stream.seekg(0, stream.end);
  size_t size = stream.tellg();
  stream.seekg(0, stream.beg);

  std::vector<char> binary(size);
  stream.read(binary.data(), size);
  return binary;
};

uuid
get_uuid(const axlf* top)
{
  return {top->m_header.uuid};
}

}} // namespace xclbin_int, core_core

////////////////////////////////////////////////////////////////
// xrt_xclbin C API implmentations (xrt_xclbin.h)
////////////////////////////////////////////////////////////////
xrtXclbinHandle
xrtXclbinAllocFilename(const char* filename)
{
  try {
    auto xclbin = std::make_shared<xrt::xclbin_full>(filename);
    auto handle = xclbin.get();
    xclbins.emplace(handle, std::move(xclbin));
    return handle;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    errno = ex.get();
  }
  catch (const std::exception& ex) {
    send_exception_message(ex.what());
    errno = 0;
  }
  return nullptr;
}

xrtXclbinHandle
xrtXclbinAllocRawData(const char* data, int size)
{
  try {
    std::vector<char> raw_data(data, data + size);
    auto xclbin = std::make_shared<xrt::xclbin_full>(raw_data);
    auto handle = xclbin.get();
    xclbins.emplace(handle, std::move(xclbin));
    return handle;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    errno = ex.get();
  }
  catch (const std::exception& ex) {
    send_exception_message(ex.what());
    errno = 0;
  }
  return nullptr;
}

xrtXclbinHandle
xrtXclbinAllocAxlf(const axlf* top_axlf)
{
  try {
    auto xclbin = std::make_shared<xrt::xclbin_full>(top_axlf);
    auto handle = xclbin.get();
    xclbins.emplace(handle, std::move(xclbin));
    return handle;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    errno = ex.get();
  }
  catch (const std::exception& ex) {
    send_exception_message(ex.what());
    errno = 0;
  }
  return nullptr;
}

int
xrtXclbinFreeHandle(xrtXclbinHandle handle)
{
  try {
    free_xclbin(handle);
    return 0;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    return ex.get();
  }
  catch (const std::exception& ex) {
    send_exception_message(ex.what());
    return -1;
  }
}

int
xrtXclbinGetXSAName(xrtXclbinHandle handle, char* name, int size, int* ret_size)
{
  try {
    auto xclbin = get_xclbin(handle);
    const std::string& xsaname = xclbin->get_xsa_name();
    // populate ret_size if memory is allocated
    if (ret_size)
      *ret_size = xsaname.size();
    // populate name if memory is allocated
    if (name)
      std::strncpy(name, xsaname.c_str(), size);
    return 0;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    // Set errno globally
    errno = ex.get();
    return ex.get();
  }
  catch (const std::exception& ex) {
    send_exception_message(ex.what());
    // Set errno globally
    errno = 0;
    return 0;
  }
}

int
xrtXclbinGetUUID(xrtXclbinHandle handle, xuid_t ret_uuid)
{
  try {
    auto xclbin = get_xclbin(handle);
    auto result = xclbin->get_uuid();
    uuid_copy(ret_uuid, result.get());
    return 0;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    // Set errno globally
    errno = ex.get();
    return ex.get();
  }
  catch (const std::exception& ex) {
    send_exception_message(ex.what());
    // Set errno globally
    errno = 0;
    return 0;
  }
}

int
xrtXclbinGetData(xrtXclbinHandle handle, char* data, int size, int* ret_size)
{
  try {
    auto xclbin = get_xclbin(handle);
    auto& result = xclbin->get_data();
    int result_size = result.size();
    // populate ret_size if memory is allocated
    if (ret_size)
      *ret_size = result_size;
    // populate data if memory is allocated
    if (data) {
      auto size_tmp = std::min(size, result_size);
      std::memcpy(data, result.data(), size_tmp);
    }
    return 0;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    // Set errno globally
    errno = ex.get();
    return ex.get();
  }
  catch (const std::exception& ex) {
    send_exception_message(ex.what());
    // Set errno globally
    errno = 0;
    return 0;
  }
}

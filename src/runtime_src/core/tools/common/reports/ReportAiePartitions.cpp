// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2023-2024 Advanced Micro Devices, Inc. All rights reserved.

// ------ I N C L U D E   F I L E S -------------------------------------------
// Local - Include Files
#include "ReportAiePartitions.h"

#include "core/common/query_requests.h"
#include "tools/common/Table2D.h"
#include "tools/common/XBUtilitiesCore.h"

#include <map>
#include <vector>

// Populate the AIE Mem tile information from the input XRT device
boost::property_tree::ptree
populate_aie_partition(const xrt_core::device* device)
{
  boost::property_tree::ptree pt;
  auto data = xrt_core::device_query_default<xrt_core::query::aie_partition_info>(device, {});
  // Group the hw contexts based on their which AIE partitions they use
  std::map<std::tuple<uint64_t, uint64_t>, boost::property_tree::ptree> pt_map;
  for (const auto& entry : data) {
    auto partition = pt_map.emplace(std::make_tuple(entry.start_col, entry.num_cols), boost::property_tree::ptree());

    boost::property_tree::ptree pt_entry;
    pt_entry.put("pid", entry.pid);
    pt_entry.put("context_id", entry.metadata.id);
    pt_entry.put("xclbin_uuid", entry.metadata.xclbin_uuid);
    pt_entry.put("command_submissions", entry.command_submissions);
    pt_entry.put("command_completions", entry.command_completions);
    pt_entry.put("migrations", entry.migrations);
    pt_entry.put("preemptions", entry.preemptions);
    pt_entry.put("errors", entry.errors);

    partition.first->second.push_back(std::make_pair("", pt_entry));
  }

  uint32_t partition_index = 0;
  boost::property_tree::ptree pt_data;
  for (const auto& entry : pt_map) {
    boost::property_tree::ptree pt_entry;
    pt_entry.put("start_col", std::get<0>(entry.first));
    pt_entry.put("num_cols", std::get<1>(entry.first));
    pt_entry.put("partition_index", partition_index++);
    pt_entry.add_child("hw_contexts", entry.second);
    pt.push_back(std::make_pair("", pt_entry));
  }
  return pt;
}

void
ReportAiePartitions::
getPropertyTreeInternal(const xrt_core::device* _pDevice, 
                        boost::property_tree::ptree &_pt) const
{
  // Defer to the 20202 format.  If we ever need to update JSON data, 
  // Then update this method to do so.
  getPropertyTree20202(_pDevice, _pt);
}

void 
ReportAiePartitions::
getPropertyTree20202(const xrt_core::device* _pDevice, 
                     boost::property_tree::ptree &_pt) const
{
  boost::property_tree::ptree pt;
  pt.put("description", "AIE Partition Information");
  pt.add_child("partitions", populate_aie_partition(_pDevice));
  _pt.add_child("aie_partitions", pt);
}

void 
ReportAiePartitions::
writeReport(const xrt_core::device* /*_pDevice*/,
            const boost::property_tree::ptree& _pt,
            const std::vector<std::string>& /*_elementsFilter*/,
            std::ostream & _output) const
{
  _output << "AIE Partitions\n";
  boost::property_tree::ptree empty_ptree;
  const boost::property_tree::ptree pt_partitions = _pt.get_child("aie_partitions.partitions", empty_ptree);
  if (pt_partitions.empty()) {
    _output << "  No hardware contexts running on device\n\n";
    return;
  }

  for (const auto& pt_partition : pt_partitions) {
    const auto& partition = pt_partition.second;

    const auto start_col = partition.get<uint64_t>("start_col");
    const auto num_cols = partition.get<uint64_t>("num_cols");
    std::string column_string;
    for (uint64_t i = 0; i < num_cols; i++) {
      column_string += std::to_string(start_col + i);
      if (i < num_cols - 1)
        column_string += ", ";
    }

    const std::vector<Table2D::HeaderData> table_headers = {
      {"PID", Table2D::Justification::left},
      {"Context ID", Table2D::Justification::left},
      {"Xclbin UUID", Table2D::Justification::left},
      {"Submissions", Table2D::Justification::left},
      {"Completions", Table2D::Justification::left},
      {"Migrations", Table2D::Justification::left},
      {"Preemptions", Table2D::Justification::left},
    };
    Table2D context_table(table_headers);

    std::vector<uint64_t> errors;
    for (const auto& pt_hw_context : partition.get_child("hw_contexts", empty_ptree)) {
      const auto& hw_context = pt_hw_context.second;

      errors.push_back(hw_context.get<uint64_t>("errors"));

      const std::vector<std::string> entry_data = {
        hw_context.get<std::string>("pid"),
        hw_context.get<std::string>("context_id"),
        hw_context.get<std::string>("xclbin_uuid"),
        hw_context.get<std::string>("command_submissions"),
        hw_context.get<std::string>("command_completions"),
        hw_context.get<std::string>("migrations"),
        hw_context.get<std::string>("preemptions")
      };
      context_table.addEntry(entry_data);
    }

    // Currently the error counts are per partition vs per context.
    // Future releases will enable per context error resolution.
    // In the meantime, each context within a partition must have the same error count.
    uint64_t error_count = errors.empty() ? 0 : errors[0];
    for (size_t i = 1; i < errors.size(); i++) {
      if (errors[i - 1] != errors[i])
        throw xrt_core::error("Invalid error counts within AIE partition");
    }

    _output << boost::str(boost::format("  Partition Index: %d\n") % partition.get<uint64_t>("partition_index"));
    _output << boost::str(boost::format("    Columns: [%s]\n") % column_string);
    _output << boost::str(boost::format("    Errors: %lu\n") % error_count);
    _output << "    HW Contexts:\n";
    _output << boost::str(boost::format("%s\n") % context_table.toString("      "));
  }

  if (XBUtilities::getVerbose()) {
    _output << "AIE Columns\n";

    const std::vector<Table2D::HeaderData> table_headers = {
      {"Column", Table2D::Justification::left},
      {"HW Context Slot", Table2D::Justification::left}
    };
    Table2D verbose_table(table_headers);

    for (const auto& pt_partition : pt_partitions) {
      const auto& partition = pt_partition.second;

      const auto start_col = partition.get<uint64_t>("start_col");
      const auto num_cols = partition.get<uint64_t>("num_cols");
      for (uint64_t i = 0; i < num_cols; i++) {
        uint64_t col = start_col + i;
        std::string context_string;
        for (const auto& pt_hw_context : partition.get_child("hw_contexts", empty_ptree))
          context_string += pt_hw_context.second.get<std::string>("context_id") + ", ";
        context_string.erase(context_string.end() - 2, context_string.end());
        const std::vector<std::string> entry_data = {
          std::to_string(col),
          boost::str(boost::format("[%s]") % context_string)
        };
        verbose_table.addEntry(entry_data);
      }
    }
    _output << boost::str(boost::format("%s\n") % verbose_table.toString("  "));
  }
}

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "aeris_msgs/msg/map_tile.hpp"

namespace aeris_map::tile_contract
{

// Validates tile ID format: "{zoom}/{x}/{y}" with bounds checking.
// Rejects invalid coordinates to prevent downstream tile generation errors.
bool is_valid_tile_id(const std::string & tile_id);

// Computes SHA-256 hash for tile integrity verification.
// Used by ground station to detect transmission corruption or cache staleness.
std::string sha256_hex(const std::vector<uint8_t> & bytes);

// Constructs a MapTile message with proper metadata for the tile streaming protocol.
// Layer IDs enable multi-layer composition (occupancy, elevation, imagery) in the UI.
aeris_msgs::msg::MapTile build_descriptor(
  const std::string & tile_id,
  const std::vector<std::string> & layer_ids,
  const std::string & hash_sha256,
  uint32_t byte_size);

}  // namespace aeris_map::tile_contract

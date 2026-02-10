#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "aeris_msgs/msg/map_tile.hpp"

namespace aeris_map::tile_contract
{

bool is_valid_tile_id(const std::string & tile_id);

std::string sha256_hex(const std::vector<uint8_t> & bytes);

aeris_msgs::msg::MapTile build_descriptor(
  const std::string & tile_id,
  const std::vector<std::string> & layer_ids,
  const std::string & hash_sha256,
  uint32_t byte_size);

}  // namespace aeris_map::tile_contract

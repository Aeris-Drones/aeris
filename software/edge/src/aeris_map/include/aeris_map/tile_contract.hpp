#pragma once

/**
 * @file tile_contract.hpp
 * @brief Map tile streaming protocol contract definitions
 *
 * This header defines the core contract for the Aeris map tile streaming
 * protocol. It provides validation, hashing, and message construction
 * utilities for tile-based map distribution from edge SLAM systems to
 * ground station operators.
 *
 * The tile contract ensures:
 * - Consistent tile ID format (Slippy Map convention)
 * - Content integrity via SHA-256 hashing
 * - Standardized message metadata for multi-layer composition
 *
 * @copyright Aeris Robotics 2024
 */

#include <cstdint>
#include <string>
#include <vector>

#include "aeris_msgs/msg/map_tile.hpp"

namespace aeris_map::tile_contract
{

/**
 * @brief Validates tile ID format and coordinate bounds.
 *
 * Tile IDs follow the Slippy Map convention: "{zoom}/{x}/{y}"
 * where zoom is 0-22 and x,y are within valid range for the zoom level.
 *
 * @param tile_id The tile identifier string to validate
 * @return true if the tile ID is valid, false otherwise
 *
 * @note Zoom levels above 22 are rejected to prevent integer overflow
 *       in coordinate calculations.
 */
bool is_valid_tile_id(const std::string & tile_id);

/**
 * @brief Computes SHA-256 hash of tile data.
 *
 * The hash is used by ground station clients to:
 * - Detect transmission corruption
 * - Validate cache entries
 * - Skip redundant tile downloads
 *
 * @param bytes Raw tile data bytes
 * @return Hexadecimal string representation of the SHA-256 digest
 */
std::string sha256_hex(const std::vector<uint8_t> & bytes);

/**
 * @brief Constructs a MapTile descriptor message.
 *
 * Creates a fully populated MapTile message with all metadata required
 * for the tile streaming protocol. Layer IDs enable multi-layer map
 * composition in the ground station UI (occupancy, elevation, imagery).
 *
 * @param tile_id Slippy map tile identifier (e.g., "18/12345/67890")
 * @param layer_ids Vector of layer identifiers for UI composition
 * @param hash_sha256 SHA-256 hex digest of tile payload
 * @param byte_size Size of tile payload in bytes
 * @return Populated MapTile message ready for publication
 */
aeris_msgs::msg::MapTile build_descriptor(
  const std::string & tile_id,
  const std::vector<std::string> & layer_ids,
  const std::string & hash_sha256,
  uint32_t byte_size);

}  // namespace aeris_map::tile_contract

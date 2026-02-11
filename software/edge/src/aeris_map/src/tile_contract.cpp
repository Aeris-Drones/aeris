/**
 * @file tile_contract.cpp
 * @brief Implementation of the map tile streaming protocol contract
 *
 * Implements tile ID validation, SHA-256 hashing, and MapTile message
 * construction as defined in the Aeris tile streaming protocol.
 *
 * @copyright Aeris Robotics 2024
 */

#include "aeris_map/tile_contract.hpp"

#include <cstdint>
#include <exception>
#include <iomanip>
#include <regex>
#include <sstream>

#include <openssl/sha.h>

namespace
{

/** @brief Regex pattern for Slippy Map tile ID format: "{zoom}/{x}/{y}" */
constexpr char kTileIdPattern[] = R"(^([0-9]{1,2})/([0-9]+)/([0-9]+)$)";

/** @brief MBTiles format version identifier */
constexpr char kFormat[] = "mbtiles-1.3";

/** @brief Minimum valid zoom level (world map) */
constexpr int kMinZoom = 0;

/** @brief Maximum valid zoom level (prevents integer overflow) */
constexpr int kMaxZoom = 22;

}  // namespace

namespace aeris_map::tile_contract
{

bool is_valid_tile_id(const std::string & tile_id)
{
  static const std::regex pattern(kTileIdPattern);
  std::smatch match;
  if (!std::regex_match(tile_id, match, pattern)) {
    return false;
  }

  try {
    // Validate zoom level is within supported range
    const int z = std::stoi(match[1].str());
    if (z < kMinZoom || z > kMaxZoom) {
      return false;
    }

    // Validate x,y coordinates are within valid range for zoom level
    const uint64_t x = std::stoull(match[2].str());
    const uint64_t y = std::stoull(match[3].str());
    const uint64_t max_index = (1ULL << static_cast<uint64_t>(z)) - 1ULL;
    return x <= max_index && y <= max_index;
  } catch (const std::exception &) {
    // stoi/stoull may throw on overflow
    return false;
  }
}

std::string sha256_hex(const std::vector<uint8_t> & bytes)
{
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(bytes.data(), bytes.size(), digest);

  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned char b : digest) {
    out << std::setw(2) << static_cast<int>(b);
  }
  return out.str();
}

aeris_msgs::msg::MapTile build_descriptor(
  const std::string & tile_id,
  const std::vector<std::string> & layer_ids,
  const std::string & hash_sha256,
  uint32_t byte_size)
{
  aeris_msgs::msg::MapTile descriptor;
  descriptor.tile_id = tile_id;
  descriptor.format = kFormat;
  descriptor.layer_ids = layer_ids;
  descriptor.hash_sha256 = hash_sha256;
  descriptor.byte_size = byte_size;
  return descriptor;
}

}  // namespace aeris_map::tile_contract

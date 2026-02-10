#include "aeris_map/tile_contract.hpp"

#include <cstdint>
#include <exception>
#include <iomanip>
#include <regex>
#include <sstream>

#include <openssl/sha.h>

namespace
{
constexpr char kTileIdPattern[] = R"(^([0-9]{1,2})/([0-9]+)/([0-9]+)$)";
constexpr char kFormat[] = "mbtiles-1.3";
constexpr int kMinZoom = 0;
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
    const int z = std::stoi(match[1].str());
    if (z < kMinZoom || z > kMaxZoom) {
      return false;
    }

    const uint64_t x = std::stoull(match[2].str());
    const uint64_t y = std::stoull(match[3].str());
    const uint64_t max_index = (1ULL << static_cast<uint64_t>(z)) - 1ULL;
    return x <= max_index && y <= max_index;
  } catch (const std::exception &) {
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

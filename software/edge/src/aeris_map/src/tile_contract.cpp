#include "aeris_map/tile_contract.hpp"

#include <iomanip>
#include <regex>
#include <sstream>

#include <openssl/sha.h>

namespace
{
constexpr char kTileIdPattern[] = R"(^([0-9]{1,2})/([0-9]+)/([0-9]+)$)";
constexpr char kFormat[] = "mbtiles-1.3";
}  // namespace

namespace aeris_map::tile_contract
{

bool is_valid_tile_id(const std::string & tile_id)
{
  static const std::regex pattern(kTileIdPattern);
  return std::regex_match(tile_id, pattern);
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

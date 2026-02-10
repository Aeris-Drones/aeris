#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "aeris_map/tile_contract.hpp"

namespace tc = aeris_map::tile_contract;

TEST(TileContract, ValidatesTileIdFormat)
{
  EXPECT_TRUE(tc::is_valid_tile_id("12/345/678"));
  EXPECT_TRUE(tc::is_valid_tile_id("0/0/0"));
  EXPECT_FALSE(tc::is_valid_tile_id("12-345-678"));
  EXPECT_FALSE(tc::is_valid_tile_id("12/345"));
  EXPECT_FALSE(tc::is_valid_tile_id("z/x/y"));
}

TEST(TileContract, ComputesKnownSha256)
{
  const std::vector<uint8_t> sample{'a', 'b', 'c'};
  EXPECT_EQ(
    tc::sha256_hex(sample),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(TileContract, PopulatesMapTileSchema)
{
  const auto descriptor = tc::build_descriptor(
    "10/123/456",
    std::vector<std::string>{"occupancy", "fetch-service:/map/get_tile_bytes"},
    "deadbeef",
    1024);

  EXPECT_EQ(descriptor.tile_id, "10/123/456");
  EXPECT_EQ(descriptor.format, "mbtiles-1.3");
  ASSERT_EQ(descriptor.layer_ids.size(), 2U);
  EXPECT_EQ(descriptor.layer_ids[0], "occupancy");
  EXPECT_EQ(descriptor.hash_sha256, "deadbeef");
  EXPECT_EQ(descriptor.byte_size, 1024U);
}

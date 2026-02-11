/**
 * @file test_tile_contract.cpp
 * @brief Unit tests for the map tile streaming protocol contract
 *
 * Tests tile ID validation, SHA-256 hashing, and MapTile message
 * construction to ensure protocol compliance.
 *
 * @copyright Aeris Robotics 2024
 */

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "aeris_map/tile_contract.hpp"

namespace tc = aeris_map::tile_contract;

/**
 * @test Validates Slippy Map tile ID format and bounds.
 *
 * Verifies:
 * - Valid format "z/x/y" is accepted
 * - Zoom level bounds (0-22) are enforced
 * - Coordinate bounds for zoom level are enforced
 * - Invalid formats are rejected
 */
TEST(TileContract, ValidatesTileIdFormat)
{
  // Valid tile IDs
  EXPECT_TRUE(tc::is_valid_tile_id("12/345/678"));
  EXPECT_TRUE(tc::is_valid_tile_id("0/0/0"));
  EXPECT_TRUE(tc::is_valid_tile_id("22/4194303/4194303"));  // Max at zoom 22

  // Invalid formats
  EXPECT_FALSE(tc::is_valid_tile_id("12-345-678"));  // Wrong separator
  EXPECT_FALSE(tc::is_valid_tile_id("12/345"));      // Missing component
  EXPECT_FALSE(tc::is_valid_tile_id("z/x/y"));       // Non-numeric

  // Out of bounds
  EXPECT_FALSE(tc::is_valid_tile_id("23/1/1"));      // Zoom too high
  EXPECT_FALSE(tc::is_valid_tile_id("1/2/0"));       // X exceeds max for zoom 1
  EXPECT_FALSE(tc::is_valid_tile_id("1/0/2"));       // Y exceeds max for zoom 1
}

/**
 * @test Verifies SHA-256 hash computation.
 *
 * Uses known test vector: SHA-256("abc") from NIST specification.
 */
TEST(TileContract, ComputesKnownSha256)
{
  const std::vector<uint8_t> sample{'a', 'b', 'c'};
  EXPECT_EQ(
    tc::sha256_hex(sample),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

/**
 * @test Verifies MapTile descriptor construction.
 *
 * Ensures all fields are correctly populated and the format
 * identifier matches the MBTiles 1.3 specification.
 */
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

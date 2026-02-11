"""Unit tests for vehicle identifier normalization utilities.

Verifies the normalize_vehicle_id function handles various input formats
correctly, including case variations, hyphen/underscore differences,
whitespace, and concatenated alphanumeric identifiers.
"""

import pytest

from aeris_orchestrator.vehicle_ids import normalize_vehicle_id


class TestNormalizeVehicleId:
    """Test cases for the normalize_vehicle_id function."""

    @pytest.mark.parametrize(
        "input_value,expected",
        [
            ("scout1", "scout_1"),
            ("scout_1", "scout_1"),
            ("Scout1", "scout_1"),
            ("SCOUT_1", "scout_1"),
            ("scout-1", "scout_1"),
            ("SCOUT-1", "scout_1"),
            ("  scout1  ", "scout_1"),
            ("scout01", "scout_01"),
            ("scout_01", "scout_01"),
            ("alpha_03", "alpha_03"),
            ("alpha__03", "alpha_03"),
            ("ranger2", "ranger_2"),
            ("RANGER-2", "ranger_2"),
        ],
    )
    def test_various_formats(self, input_value: str, expected: str) -> None:
        """Test normalization of various vehicle ID formats.

        Args:
            input_value: Raw vehicle identifier input.
            expected: Expected normalized output.
        """
        assert normalize_vehicle_id(input_value) == expected

    @pytest.mark.parametrize(
        "input_value",
        ["", "   ", "___", "---", "   ___   "],
    )
    def test_empty_and_whitespace_returns_empty(self, input_value: str) -> None:
        """Test that empty or whitespace-only inputs return empty string.

        Args:
            input_value: Input that should result in empty output.
        """
        assert normalize_vehicle_id(input_value) == ""

    def test_empty_string(self) -> None:
        """Test that empty string returns empty string."""
        assert normalize_vehicle_id("") == ""

    def test_only_underscores(self) -> None:
        """Test that only underscores return empty string."""
        assert normalize_vehicle_id("___") == ""

    def test_only_whitespace(self) -> None:
        """Test that only whitespace returns empty string."""
        assert normalize_vehicle_id("   ") == ""

#!/usr/bin/env python3
"""Map tile verification utility using Playwright browser automation.

This module provides automated verification of map tile rendering in the
AERIS viewer. It launches a headless browser, navigates to the viewer,
waits for tiles to load, and captures a screenshot for visual verification.

Dependencies:
    - playwright: Browser automation library

Usage:
    python verify_map.py

Prerequisites:
    - Viewer must be running at http://localhost:3000
    - Map tile publisher should be active (e.g., publish_dummy_tiles.py)

Verification Logic:
    1. Navigate to viewer at localhost:3000
    2. Wait for HUD element ("Map Status") to appear
    3. Wait 10 seconds for tile data to arrive and render
    4. Capture screenshot to /app/verification.png
    5. Check page content for "Tiles Loaded: 0" (failure indicator)

Exit Codes:
    0: Success (tiles loaded count > 0)
    1: Failure (tiles loaded count is 0 or error occurred)
"""

import time
from playwright.sync_api import sync_playwright, expect


def verify_map_rendering():
    """Verify map tile rendering in the AERIS viewer.

    Launches a headless Chromium browser, navigates to the viewer,
    waits for map tiles to load, and performs basic validation.

    Returns:
        bool: True if verification succeeded, False otherwise.

    Raises:
        Exception: If the HUD element is not found within timeout.
    """
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()

        print("Navigating to viewer...")
        try:
            page.goto("http://localhost:3000")
        except Exception as e:
            print(f"Failed to load page: {e}")
            return False

        print("Waiting for HUD...")
        try:
            expect(page.get_by_text("Map Status")).to_be_visible(timeout=10000)
        except Exception:
            print("HUD not found.")
            raise

        print("Waiting for tiles to load...")
        # Allow time for tile publisher to send data and viewer to render
        time.sleep(10)

        screenshot_path = "/app/verification.png"
        page.screenshot(path=screenshot_path)
        print(f"Screenshot saved to {screenshot_path}")

        # Validate tile loading by checking for non-zero count
        content = page.content()
        if "Tiles Loaded: 0" in content:
            print("FAILURE: Tiles Loaded count is 0.")
            success = False
        else:
            print("SUCCESS: Tiles Loaded count seems > 0 (or text changed).")
            success = True

        browser.close()
        return success


if __name__ == "__main__":
    success = verify_map_rendering()
    exit(0 if success else 1)

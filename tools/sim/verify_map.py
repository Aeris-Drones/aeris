import time
from playwright.sync_api import sync_playwright, expect

def verify_map_rendering():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()

        print("Navigating to viewer...")
        try:
            page.goto("http://localhost:3000")
        except Exception as e:
            print(f"Failed to load page: {e}")
            return

        print("Waiting for HUD...")
        try:
            expect(page.get_by_text("Map Status")).to_be_visible(timeout=10000)
        except Exception:
            print("HUD not found.")
            raise

        print("Waiting for tiles to load...")
        time.sleep(10) # Give plenty of time for publisher to send data

        screenshot_path = "/app/verification.png"
        page.screenshot(path=screenshot_path)
        print(f"Screenshot saved to {screenshot_path}")

        # Simple check for "Tiles Loaded" text in body, assumes format "Tiles Loaded: X"
        # We will read the screenshot to verify visually as well.
        content = page.content()
        # Check if "Tiles Loaded: 0" is NOT present (meaning we have some tiles)
        # Or check if "Tiles Loaded: [1-9]" matches
        if "Tiles Loaded: 0" in content:
            print("FAILURE: Tiles Loaded count is 0.")
        else:
            print("SUCCESS: Tiles Loaded count seems > 0 (or text changed).")

        browser.close()

if __name__ == "__main__":
    verify_map_rendering()

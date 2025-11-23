import time
from playwright.sync_api import sync_playwright

def verify_thermal_markers():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        try:
            page = browser.new_page()

            print("Navigating to viewer...")
            try:
                page.goto("http://localhost:3000", timeout=60000)
            except Exception as e:
                print(f"Navigation failed: {e}")
                # Check logs
                return

            print("Waiting for canvas...")
            page.wait_for_selector("canvas", timeout=30000)

            # Wait for markers to appear (ROS connection + message receipt)
            print("Waiting for markers...")
            time.sleep(10)

            # Take screenshot
            screenshot_path = "verification/thermal_markers.png"
            page.screenshot(path=screenshot_path)
            print(f"Screenshot saved to {screenshot_path}")

        except Exception as e:
            print(f"Verification failed: {e}")
        finally:
            browser.close()

if __name__ == "__main__":
    verify_thermal_markers()

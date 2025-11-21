from playwright.sync_api import sync_playwright
import time

def verify_vehicles():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        try:
            page = browser.new_page()

            # Wait for Next.js to start
            max_retries = 2
            for attempt in range(max_retries):
                try:
                    page.goto("http://localhost:3000", timeout=30000)
                    break
                except Exception as e:
                    if attempt < max_retries - 1:
                        print(f"Connection failed (attempt {attempt + 1}/{max_retries}), retrying...")
                        time.sleep(5)
                    else:
                        raise RuntimeError(f"Failed to connect to Next.js app after {max_retries} attempts") from e

            # Wait for Scene3D to load (canvas)
            page.wait_for_selector("canvas", timeout=10000)

            # Wait a bit for vehicles to move and trajectories to form
            print("Waiting for simulation...")
            time.sleep(5)

            # Take screenshot
            screenshot_path = "verification_vehicles.png"
            page.screenshot(path=screenshot_path)
            print(f"Screenshot saved to {screenshot_path}")
        finally:
            browser.close()

if __name__ == "__main__":
    verify_vehicles()

from playwright.sync_api import sync_playwright
import time

def verify_vehicles():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()

        # Wait for Next.js to start
        try:
            page.goto("http://localhost:3000", timeout=30000)
        except:
            print("Retrying connection...")
            time.sleep(5)
            page.goto("http://localhost:3000")

        # Wait for Scene3D to load (canvas)
        page.wait_for_selector("canvas", timeout=10000)

        # Wait a bit for vehicles to move and trajectories to form
        print("Waiting for simulation...")
        time.sleep(5)

        # Take screenshot
        screenshot_path = "verification_vehicles.png"
        page.screenshot(path=screenshot_path)
        print(f"Screenshot saved to {screenshot_path}")

        browser.close()

if __name__ == "__main__":
    verify_vehicles()

# ESP32-CAM OV3660 Telegram Photo Sender

An Arduino sketch for the AI-Thinker ESP32-CAM module paired specifically with the **OV3660** camera sensor. It periodically captures and sends photos to a Telegram chat using the `AsyncTelegram2` library, featuring a sophisticated auto/dark exposure control loop and rich telemetry captions.

---

## Features

* **Advanced Exposure Loop (Auto & Dark Modes):** Continuously monitors image average brightness and sensor gain. When ambient light drops below a threshold, it automatically switches to a closed-loop manual long exposure mode with frame-length adjustments.
* **Detailed Telegram Captions:** Each photo sent to Telegram includes rich diagnostic information, such as local timestamp, WiFi RSSI signal strength, exposure mode, average light levels, and JPEG dimensions/size.
* **Configurable Image Options:** Easily customize resolution (up to QXGA/1080p/720p/VGA), JPEG quality, white balance, orientation (horizontal mirror/vertical flip), and special effects right from the configuration block.
* **Status LED Indication:** Blinks rapidly while connecting to WiFi and stays solid once a stable connection is established.
* **Brownout Protection:** Disables the ESP32 brownout detector on startup to prevent resets during high-current camera operations.

---

## Hardware Requirements

* **Microcontroller:** AI-Thinker ESP32-CAM (with built-in PSRAM recommended for higher resolutions).
* **Camera Sensor:** OV3660 Camera Module.
* **Programmer:** USB-to-Serial (FTDI) adapter (3.3V logic) for flashing the code.
* **Power Supply:** Stable external 5V power source to prevent brownouts during wireless transmission.

---

## Library Dependencies

Make sure you install the required library via the Arduino IDE Library Manager:

* [AsyncTelegram2](https://github.com/cotestatnt/AsyncTelegram2) by cotestatnt
* Standard ESP32 board support packages (including `esp_camera`)

---

## Configuration & Setup

1. Open the `.ino` sketch in your Arduino IDE.
2. Fill in your network and Telegram credentials near the top of the file:
   ```cpp
   const char* ssid  = "YOUR_WIFI_SSID";
   const char* pass  = "YOUR_WIFI_PASSWORD";
   const char* token = "YOUR_TELEGRAM_BOT_TOKEN";
   int64_t userid = YOUR_TELEGRAM_USER_ID;

3. Adjust your preferred image resolution, JPEG quality, and orientation settings in the IMAGE OPTIONS section.

4. Select your board (AI Thinker ESP32-CAM), choose the correct COM port, and upload the code.
   

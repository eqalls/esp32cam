/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <iot-d_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include <string.h> // Include for strcmp
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>

//WIFI
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASS"

// Pin Definitions
#define TILT_SENSOR_PIN 13 
#define HARVEST_BUTTON_PIN 12  // New button pin for manual harvest logging

// API endpoints
#define VIBRATION_LOG_URL "http://YOUR_DOMAIN/api/vibration-log"
#define HARVEST_LOG_URL "http://YOUR_DOMAIN/api/harvest-log"

// Farmer and durian details
#define FARMER_ID 1
#define ORCHARD_ID 1
#define DURIAN_ID 1
#define DURIAN_TYPE "Musang King"

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture
int detectCount = 0;
const String FIREBASE_HOST = "FIREBASE_URL";
const String FIREBASE_AUTH = "FIREBASE_API";
unsigned long lastButtonDebounceTime = 0;
int lastButtonState = LOW;
int buttonState = LOW;
const unsigned long debounceDelay = 50;  // Debounce time in milliseconds

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function declarations ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
void fallDurianLog(int logType);
void sendHarvestLog();
void checkHarvestButton();
void reconnectWiFi();

/**
 * @brief      Arduino setup function
 */
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // Initialize pins
    pinMode(TILT_SENSOR_PIN, INPUT);       // Set tilt sensor pin as input
    pinMode(HARVEST_BUTTON_PIN, INPUT_PULLUP);  // Set harvest button pin as input with pull-up

    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nWi-Fi Connected!");

    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    } else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nWaiting for vibration detection or harvest button press...\n");
    Serial.println("Press the harvest button to log a harvest");
}

/**
 * @brief      Get data and run inferencing
 *
 * @param[in]  debug  Get debug info if true
 */
void loop() {
    // Check for harvest button press
    checkHarvestButton();
    
    // Check for vibration detection
    if (digitalRead(TILT_SENSOR_PIN) == HIGH) {  // Vibration detected
        Serial.println("Vibration detected! Capturing image...");
        
        snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

        if (snapshot_buf == nullptr) {
            ei_printf("ERR: Failed to allocate snapshot buffer!\n");
            return;
        }

        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_get_data;

        if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
            ei_printf("Failed to capture image\r\n");
            free(snapshot_buf);
            return;
        }

        // Run object detection
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            free(snapshot_buf);
            return;
        }

        // Print results
        ei_printf("Object detection bounding boxes:\r\n");
        bool objectDetected = false;
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) continue;

            // Check the label and print accordingly
            if (strcmp(bb.label, "earbud") == 0) {
                ei_printf("  earbud (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                          bb.value, bb.x, bb.y, bb.width, bb.height);
                detectCount++;
                objectDetected = true;
                
                // Update Firebase
                String url = FIREBASE_HOST + "sensors/sensor1/vibrationCount.json?auth=" + FIREBASE_AUTH;
                HTTPClient http;
                http.begin(url);
                http.addHeader("Content-Type", "application/json");
                int httpCode = http.PUT(String(detectCount));
                http.end();
                
                if (httpCode == HTTP_CODE_OK) {
                    Serial.printf("Total: %d\n", detectCount);
                } else {
                    Serial.printf("Firebase error: %s\n", http.errorToString(httpCode).c_str());
                }
                fallDurianLog(1); 
            }
            else if (strcmp(bb.label, "pokemon") == 0) {
                ei_printf("  pokemon (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                          bb.value, bb.x, bb.y, bb.width, bb.height);
                objectDetected = true;
                fallDurianLog(2);
            }
        }
        
        free(snapshot_buf);
        
        // Only delay if an object was detected
        if (objectDetected) {
            delay(3000);  // Prevent rapid retriggering when object is detected
        }
    }
    
    // Small delay to avoid excessive CPU usage
    delay(50);
}

/**
 * @brief   Check for harvest button press with debouncing
 */
void checkHarvestButton() {
    // Read the state of the button
    int reading = digitalRead(HARVEST_BUTTON_PIN);
    
    // Check if the button state has changed
    if (reading != lastButtonState) {
        lastButtonDebounceTime = millis();
    }
    
    // If the button state has been stable for the debounce delay
    if ((millis() - lastButtonDebounceTime) > debounceDelay) {
        // If the button state is different from the last stable state
        if (reading != buttonState) {
            buttonState = reading;
            
            // If button is pressed (LOW when using INPUT_PULLUP)
            if (buttonState == LOW) {
                Serial.println("Harvest button pressed - sending harvest log");
                sendHarvestLog();
            }
        }
    }
    
    lastButtonState = reading;
}

/**
 * @brief   Send harvest log to backend
 */
void sendHarvestLog() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        
        http.begin(HARVEST_LOG_URL);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        
        // Get current date (use current date or implement an RTC for more accuracy)
        String currentDate = "2025-05-03"; // Replace with actual date if RTC available
        
        // Prepare POST data with required fields and null values for optional fields
        String postData = "farmer_id=" + String(FARMER_ID) +
                          "&orchard_id=" + String(ORCHARD_ID) +
                          "&durian_id=" + String(DURIAN_ID) +
                          "&durian_type=" + String(DURIAN_TYPE) +
                          "&harvest_date=" + currentDate +
                          "&total_harvested=" + String(detectCount) +
                          "&status=pending" +
                          "&estimated_weight=null" +
                          "&grade=null" +
                          "&condition=null" +
                          "&storage_location=null" +
                          "&remarks=null" +
                          "&harvester_signature=null";
        
        // Send POST request
        int httpCode = http.POST(postData);
        
        // Print response code
        Serial.printf("[Harvest Log] HTTP code: %d | Total harvested: %d\n", httpCode, detectCount);
        
        if (httpCode == HTTP_CODE_OK) {
            Serial.println("Harvest logged successfully");
            // Reset detection count after successful harvest logging
            detectCount = 0;
        } else {
            Serial.printf("HTTP code: %d | Harvest data not saved\n", httpCode);
        }
        
        http.end();
    } else {
        Serial.println("Wi-Fi disconnected - harvest not logged");
        reconnectWiFi();
    }
}

/**
 * @brief   Send log to backend about durian fall detection
 * 
 * @param   logType 1 for earbud, 2 for pokemon (as per your object detection classes)
 */
void fallDurianLog(int logType) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        
        // Use the correct API URL
        http.begin(VIBRATION_LOG_URL);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");

        // Prepare POST data
        String postData = "vibrationCount=" + String(detectCount) + "&deviceID=ESP32&logType=" + String(logType);
        
        // Send POST request
        int httpCode = http.POST(postData);

        // Print response code
        Serial.printf("[Fall Detection] HTTP code: %d | Durians: %d\n", httpCode, detectCount);

        if (httpCode == HTTP_CODE_OK) {
            Serial.println("Fall detection logged successfully");
        } else {
            Serial.printf("HTTP code: %d | Data not saved\n", httpCode);
        }

        http.end();
    } else {
        Serial.println("Wi-Fi disconnected - fall detection not logged");
        reconnectWiFi();
    }
}

/**
 * @brief   Attempt to reconnect to WiFi if connection is lost
 */
void reconnectWiFi() {
    Serial.print("Reconnecting to Wi-Fi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Try to reconnect for up to 10 seconds
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        Serial.print(".");
        delay(1000);
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWi-Fi Reconnected!");
    } else {
        Serial.println("\nFailed to reconnect Wi-Fi");
    }
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1); // flip it back
        s->set_brightness(s, 1); // up the brightness just a bit
        s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK) {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

    esp_camera_fb_return(fb);

    if(!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif

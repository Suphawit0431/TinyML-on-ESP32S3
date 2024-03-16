#include <Arduino.h>
#include <openmvrpc.h>
#include <esp_camera.h>
#include <esp_log.h>

#define TAG "main"

// rpc
openmv::rpc_scratch_buffer<256> scratch_buffer;
openmv::rpc_callback_buffer<8> callback_buffer;
openmv::rpc_hardware_serial_uart_slave rpc_slave;

// camera hardware T-cameras3
// #define CAM_PIN_PWDN    -1
// #define CAM_PIN_RESET   39
// #define CAM_PIN_XCLK    38
// #define CAM_PIN_SIOD    5
// #define CAM_PIN_SIOC    4

// #define CAM_PIN_D7      9
// #define CAM_PIN_D6      10
// #define CAM_PIN_D5      11
// #define CAM_PIN_D4      13
// #define CAM_PIN_D3      21
// #define CAM_PIN_D2      48
// #define CAM_PIN_D1      47
// #define CAM_PIN_D0      14
// #define CAM_PIN_VSYNC   8
// #define CAM_PIN_HREF    18
// #define CAM_PIN_PCLK    12

// #define I2C_SDA          7
// #define I2C_SCL          6

// camera hardware T-simcam
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   18
#define CAM_PIN_XCLK    14
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5

#define CAM_PIN_D7      15
#define CAM_PIN_D6      16
#define CAM_PIN_D5      17
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

#define I2C_SDA          7
#define I2C_SCL          6

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

XPowersPMU  PMU;

uint8_t buffer[10000];
uint16_t buffer_sz = 0;
volatile bool frame_ready = false;

void camera_init(void) {

    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("Failed to initialize power.....");
        while (1) {
            delay(5000);
        }
    }
    //Set the working voltage of the camera, please do not modify the parameters
    PMU.setALDO1Voltage(1800);  // CAM DVDD  1500~1800
    PMU.enableALDO1();
    PMU.setALDO2Voltage(2800);  // CAM DVDD 2500~2800
    PMU.enableALDO2();
    PMU.setALDO4Voltage(3000);  // CAM AVDD 2800~3000
    PMU.enableALDO4();

    // TS Pin detection must be disable, otherwise it cannot be charged
    PMU.disableTSPinMeasure();

    static camera_config_t camera_config;
    // Wire.begin(I2C_SDA, I2C_SCL);

    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.ledc_timer = LEDC_TIMER_0;

    camera_config.pin_d0 = CAM_PIN_D0;
    camera_config.pin_d1 = CAM_PIN_D1;
    camera_config.pin_d2 = CAM_PIN_D2;
    camera_config.pin_d3 = CAM_PIN_D3;
    camera_config.pin_d4 = CAM_PIN_D4;
    camera_config.pin_d5 = CAM_PIN_D5;
    camera_config.pin_d6 = CAM_PIN_D6;
    camera_config.pin_d7 = CAM_PIN_D7;
    camera_config.pin_xclk = CAM_PIN_XCLK;
    camera_config.pin_pclk = CAM_PIN_PCLK;
    camera_config.pin_vsync = CAM_PIN_VSYNC;
    camera_config.pin_href = CAM_PIN_HREF;
    camera_config.pin_sccb_sda = CAM_PIN_SIOD;
    camera_config.pin_sccb_scl = CAM_PIN_SIOC;
    camera_config.pin_pwdn = CAM_PIN_PWDN;
    camera_config.pin_reset = CAM_PIN_RESET;
    camera_config.xclk_freq_hz = 20000000;

    camera_config.pixel_format = PIXFORMAT_JPEG; //YUV422,GRAYSCALE,RGB565,JPEG
    camera_config.frame_size = FRAMESIZE_240X240; //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    camera_config.jpeg_quality = 20; //0-63, for OV series camera sensors, lower number means higher quality
    camera_config.fb_count = 2; //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.grab_mode = CAMERA_GRAB_LATEST; //CAMERA_GRAB_LATEST. Sets when buffers should be filled

    esp_camera_deinit();
    delay(100);
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "status code: %d", s->init_status(s));
    
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);

    /*
    sensor_t *cam_sensor = esp_camera_sensor_get();
    cam_sensor->set_framesize(cam_sensor, FRAMESIZE_240X240);
    cam_sensor->set_brightness(cam_sensor, 0);     // -2 to 2
    cam_sensor->set_contrast(cam_sensor, 0);       // -2 to 2
    cam_sensor->set_saturation(cam_sensor, 0);     // -2 to 2
    cam_sensor->set_special_effect(cam_sensor, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    cam_sensor->set_whitebal(cam_sensor, 1);       // 0 = disable , 1 = enable
    cam_sensor->set_awb_gain(cam_sensor, 1);       // 0 = disable , 1 = enable
    cam_sensor->set_wb_mode(cam_sensor, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    cam_sensor->set_exposure_ctrl(cam_sensor, 1);  // 0 = disable , 1 = enable
    cam_sensor->set_aec2(cam_sensor, 0);           // 0 = disable , 1 = enable
    cam_sensor->set_ae_level(cam_sensor, 0);       // -2 to 2
    cam_sensor->set_aec_value(cam_sensor, 300);    // 0 to 1200
    cam_sensor->set_gain_ctrl(cam_sensor, 1);      // 0 = disable , 1 = enable
    cam_sensor->set_agc_gain(cam_sensor, 0);       // 0 to 30
    cam_sensor->set_gainceiling(cam_sensor, (gainceiling_t)0);  // 0 to 6
    cam_sensor->set_bpc(cam_sensor, 0);            // 0 = disable , 1 = enable
    cam_sensor->set_wpc(cam_sensor, 1);            // 0 = disable , 1 = enable
    cam_sensor->set_raw_gma(cam_sensor, 1);        // 0 = disable , 1 = enable
    cam_sensor->set_lenc(cam_sensor, 1);           // 0 = disable , 1 = enable
    cam_sensor->set_hmirror(cam_sensor, 0);        // 0 = disable , 1 = enable
    cam_sensor->set_vflip(cam_sensor, 0);          // 0 = disable , 1 = enable
    cam_sensor->set_dcw(cam_sensor, 1);            // 0 = disable , 1 = enable
    cam_sensor->set_colorbar(cam_sensor, 0);       // 0 = disable , 1 = enable
    */
}

uint32_t camera_snapshot(camera_fb_t *fb) {
    camera_fb_t *fb = NULL;

    // Take a photo with the camera
    while (fb == NULL) {

      fb = esp_camera_fb_get();

      delay(100);

      ESP_LOGI(TAG, "Checking frame buffer");

    }
    if (fb) {
        ESP_LOGI(TAG, "Camera capture success: %dx%d", fb->width, fb->height);
    } else {
        ESP_LOGE(TAG, "Camera capture failed %d", fb);
    }
    memcpy(buffer, fb->buf, fb->len);
    esp_camera_fb_return(fb); //return the frame buffer back to the driver for reuse
    return fb->len;
}

// callbacks
size_t digital_read_callback(void *out_data) {
    // Get what we want to return into a variable.
    uint32_t state = 5000;

    // Move that variable into a transmit buffer.
    memcpy(out_data, &state, sizeof(state));

    // Return how much we will send.
    return sizeof(state);
}

size_t jpeg_image_snapshot_callback(void *out_data) {
  buffer_sz = camera_snapshot(camera_fb_t *fb);
  memcpy(out_data, &buffer_sz, sizeof(buffer_sz));
  return sizeof(buffer_sz);
}

size_t jpeg_image_read_callback(void *out_data) {
    frame_ready = true;
    return 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // resoruce check
  ESP_LOGI(TAG, "Total heap: %u", ESP.getHeapSize());
  ESP_LOGI(TAG, "Free heap: %u", ESP.getFreeHeap());
  ESP_LOGI(TAG, "Total PSRAM: %u", ESP.getPsramSize());
  ESP_LOGI(TAG, "Free PSRAM: %d", ESP.getFreePsram());

  // camera init and test
  camera_init();

  // rpc define
  rpc_slave.register_callback(F("digital_read"), digital_read_callback);
  rpc_slave.register_callback(F("jpeg_image_snapshot"), jpeg_image_snapshot_callback);
  rpc_slave.register_callback(F("jpeg_image_read"), jpeg_image_read_callback);
  rpc_slave.begin();

  /*
  for (int i = 0; i < 10; i++) {
    uint16_t sz = camera_snapshot();
    ESP_LOGI(TAG, "Camera capture success: %d", sz);
    ESP_LOGI(TAG, "Free heap: %u", ESP.getFreeHeap());
    ESP_LOGI(TAG, "Free PSRAM: %d", ESP.getFreePsram());
    delay(1000);
  }
  */
}

void loop() {
  if (frame_ready) {
    rpc_slave.put_bytes(buffer, buffer_sz, 10000);
    frame_ready = false;
  }
  rpc_slave.loop();
}

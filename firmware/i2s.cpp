#include "i2s.h"

// ============================================================================
// ESP32 GPIO Bridge - I2S Audio Operations Implementation
// ============================================================================

void handleI2SInitTx(String bckStr, String wsStr, String dataStr, String rateStr) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = (uint32_t)rateStr.toInt(),
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = (gpio_num_t)bckStr.toInt(),
        .ws_io_num = (gpio_num_t)wsStr.toInt(),
        .data_out_num = (gpio_num_t)dataStr.toInt(),
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    // OK response removed (v0.1.4 optimization)
}

void handleI2SWrite(String parts[], int partCount) {
    size_t bytes_written = 0;
    int16_t* buffer = new int16_t[partCount - 1];
    for(int i=1; i < partCount; i++) {
        buffer[i-1] = (int16_t)parts[i].toInt();
    }
    i2s_write(I2S_NUM_0, buffer, (partCount-1) * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    delete[] buffer;
    // OK response removed (v0.1.4 optimization)
}

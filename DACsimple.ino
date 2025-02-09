// IMPORT IMPORTANT LIBRARY
#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"

BluetoothA2DPSink a2dp_sink;
#define mutepin 2
#define mosfet 4
// config your I2S PIN im using cs4344 so i need MCLK
i2s_pin_config_t pin_config = {
    .mck_io_num = 3,                 // MCLK pin
    .bck_io_num = 19,                // BCK pin
    .ws_io_num = 21,                 // LRCK pin
    .data_out_num = 18,              // DATA out pin
    .data_in_num = I2S_PIN_NO_CHANGE // Not used
};

// Custom I2S configuration
i2s_config_t i2s_config_stereo = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),                    // TX only
    .sample_rate = 44100,                                                   // Sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                           // Bits per sample
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           // Channel format
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S), // Communication format
    .intr_alloc_flags = 0,                                                  // Interrupt allocation
    .dma_buf_count = 8,                                                     // DMA buffer count
    .dma_buf_len = 1024,                                                    // DMA buffer length
    .use_apll = true,                                                       // Use APLL
    .tx_desc_auto_clear = true,                                             // Auto clear tx descriptor on underflow
    .fixed_mclk = 0                                                         // Konfigurasi MCLK ke 11.2896 MHz
};
//change 8bit format to 16bit format
void audio_data_callback(const uint8_t *data, uint32_t len)
{
  // len is in bytes, divide by 4 to get the number of 16-bit stereo samples
  size_t num_samples = len / 4;
  // Temporary buffer to store converted int16_t data
  int16_t i2s_data[num_samples * 2];

  for (size_t i = 0; i < num_samples; i++)
  {
    // Convert each stereo sample from uint8_t to int16_t
    int16_t left = (data[i * 4 + 1] << 8) | data[i * 4];      // Left channel
    int16_t right = (data[i * 4 + 3] << 8) | data[i * 4 + 2]; // Right channel

    // Store the converted data in the i2s_data buffer
    i2s_data[i * 2] = left;
    i2s_data[i * 2 + 1] = right;
  }
  size_t i2s_bytes_written;
  i2s_write(I2S_NUM_0, i2s_data, sizeof(i2s_data), &i2s_bytes_written, portMAX_DELAY);//sent to DAC
}

void setup()
{
  Serial.begin(115200);
  pinMode(mosfet, OUTPUT);
  pinMode(mutepin, OUTPUT);
  digitalWrite(mosfet, HIGH);
  digitalWrite(mutepin, HIGH);

  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  a2dp_sink.set_stream_reader(audio_data_callback, false);
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.start("Harman/Kardon Genie");
}

void loop()
{
  //empty, want more function? check github!
}

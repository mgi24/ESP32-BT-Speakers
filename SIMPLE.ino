// IMPORT IMPORTANT LIBRARY
#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"

#define BTLED 2
BluetoothA2DPSink a2dp_sink; // Initialize Library

// config your I2S PIN
i2s_pin_config_t pin_config = {
    .mck_io_num = 3,                 // MCLK pin, ONLY USE gpio 0,1,or 3 otherwise not work
    .bck_io_num = 19,                // BCK/SCK pin
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
// change 8bit format to 16bit format, easier to process later
void audio_data_callback(const uint8_t *data, uint32_t len) // BT data on 8bit format
{
  size_t num_samples = len / 4;      // LLSB, LMSB, RMSB, RLSB, so divide by 4 per point
  int16_t i2s_data[num_samples * 2]; // Create a temporary buffer for 16-bit stereo samples

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
  i2s_write(I2S_NUM_0, i2s_data, sizeof(i2s_data), &i2s_bytes_written, portMAX_DELAY); // sent to DAC
}

void setup()
{ 
  Serial.begin(115200); // disable if not used please, since pin RX/gpio 3 is used for MCLK

  pinMode(BTLED, OUTPUT); // Set BTLED as output

  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL); // install i2s config
  i2s_set_pin(I2S_NUM_0, &pin_config);                        // configure pin

  a2dp_sink.set_stream_reader(audio_data_callback, false); // if audio received, run this program
  a2dp_sink.set_auto_reconnect(true);                      // remember last device and try to connect
  a2dp_sink.start("Harman/Kardon Genie");                  // Speaker name on BT
}

void loop()
{
  if (a2dp_sink.is_connected())
  {
    digitalWrite(BTLED, HIGH); // Turn on LED when connected
  }
  else
  {
    digitalWrite(BTLED, LOW); // Turn off LED when disconnected
  }
}

// IMPORT IMPORTANT LIBRARY
#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"

// DEFINE YOUR PIN HERE
#define mutepin 2
#define mosfetpin 4
#define powerbtn 25
#define beatLED 32
#define btled 33
#define ppbtn 26
#define dcbtn 27
#define prev 14
#define nextbtn 13

BluetoothA2DPSink a2dp_sink;

// config your I2S PIN
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

i2s_config_t i2s_config_mono = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // TX only
    .sample_rate = 44100,                                // Sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,        // Bits per sample
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S), // Communication format
    .intr_alloc_flags = 0,                                                  // Interrupt allocation
    .dma_buf_count = 8,                                                     // DMA buffer count
    .dma_buf_len = 1024,                                                    // DMA buffer length
    .use_apll = true,                                                       // Use APLL
    .tx_desc_auto_clear = true,                                             // Auto clear tx descriptor on underflow
    .fixed_mclk = 0                                                         // Konfigurasi MCLK ke 11.2896 MHz
};

void audio_data_callback(const uint8_t *data, uint32_t len)
{
  // len is in bytes, divide by 4 to get the number of 16-bit stereo samples
  size_t num_samples = len / 4;
  // audiolen = num_samples * 2;
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
  // memcpy(audiobuff, i2s_data, sizeof(i2s_data));
  size_t i2s_bytes_written;

  i2s_write(I2S_NUM_0, i2s_data, sizeof(i2s_data), &i2s_bytes_written, portMAX_DELAY);
}

void setup()
{
  Serial.begin(115200);
  pinMode(ppbtn, INPUT);
  pinMode(dcbtn, INPUT);
  pinMode(prev, INPUT);
  pinMode(nextbtn, INPUT);

  pinMode(mosfetpin, OUTPUT);
  pinMode(mutepin, OUTPUT);
  pinMode(beatLED, OUTPUT);
  pinMode(btled, OUTPUT);

  pinMode(powerbtn, INPUT);

  digitalWrite(mosfetpin, HIGH);
  digitalWrite(mutepin, LOW);
  digitalWrite(beatLED, HIGH);
  digitalWrite(btled, LOW);

  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  a2dp_sink.set_stream_reader(audio_data_callback, false);
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.start("Harman/Kardon Genie");
  delay(1000);
}

int volume = 0;
long btdckeytimer = 0;
bool isdc = false;
bool isplay = true;
bool ignorepp = false;
bool isnext = false;
bool isprev = false;
bool isconnected = false;

void loop()
{
  // CONNECT
  if (a2dp_sink.is_connected())
  { isconnected = 1;
    volume = a2dp_sink.get_volume(); // get volume if changed from source
    digitalWrite(btled, LOW);
  }
  else
  { isconnected = 0;
    digitalWrite(btled, HIGH);
  }

  // MUTE
  if (a2dp_sink.get_audio_state() == 2)
  { 
    if (volume == 0 || isconnected == 0)
    {
      
      digitalWrite(mutepin, LOW);
    }
    else
    {
      digitalWrite(mutepin, HIGH);
    }
    
  }
  else
  {
    digitalWrite(mutepin, LOW);
  }

  // SHUTDOWN
  if (analogRead(powerbtn) < 1000)
  {
    analogWrite(mutepin, LOW);
    delay(800);
    analogWrite(mosfetpin, LOW);
  }

  // PLAY PAUSE
  if (analogRead(ppbtn) < 1000)
  {
    bool timer = millis();
    while (analogRead(ppbtn) < 1000)
    {
      if (millis() - timer > 100 && !ignorepp)
      {
        ignorepp = true;
        if (isplay)
        {
          a2dp_sink.pause();
          isplay = false;
          Serial.println("PAUSE");
        }
        else
        {
          a2dp_sink.play();
          isplay = true;
          Serial.println("PLAY");
        }
      }
    }
  }
  else
  {
    ignorepp = false;
  }

  // DISCONNECT
  if (analogRead(dcbtn) < 1000)
  {
    btdckeytimer = millis();
    while (analogRead(dcbtn) < 1000)
    {
      Serial.println(millis() - btdckeytimer);
      if (millis() - btdckeytimer > 1000)
      {
        Serial.println("DISCONNECT");
        a2dp_sink.disconnect();
        digitalWrite(btled, HIGH);
      }
    }
  }

  // NEXT
  if (analogRead(nextbtn) < 1000)
  {
    long nexttimer = millis();
    while (analogRead(nextbtn) < 1000)
    {
      if (millis() - nexttimer > 1000 && !isnext)
      {
        isnext = true;
        Serial.println("NEXT");
        a2dp_sink.next();
      }
    }
    if (millis() - nexttimer > 10 && !isnext)
    {

      Serial.println("VOL UP");
      if (volume < 130)
      {
        volume = volume + 10;
        if (volume > 130)
        {
          volume = 130;
        }
        a2dp_sink.set_volume(volume);
      }
    }
  }
  else
  {
    isnext = false;
  }

  // PREV
  if (analogRead(prev) < 1000)
  {
    long prevtimer = millis();
    while (analogRead(prev) < 1000)
    {
      if (millis() - prevtimer > 1000 && !isprev)
      {
        isprev = true;
        Serial.println("PREV");
        a2dp_sink.previous();
      }
    }
    if (millis() - prevtimer > 10 && !isprev)
    {
      Serial.println("VOL DOWN");
      if (volume > 0)
      {
        volume = volume - 10;
        if (volume <= 0)
        {
          volume = 0;
        }
        a2dp_sink.set_volume(volume);
      }
    }
  }
  else
  {
    isprev = false;
  }

  Serial.flush();
}

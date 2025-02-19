// IMPORT IMPORTANT LIBRARY
#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"
#include "audio_data.h"
#include "shutdown.h"
#include "connected.h"
#include "max.h"
#include "dc.h"
#include "arduinoFFT.h"

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
#define SAMPLE_RATE 44100
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
    .sample_rate = SAMPLE_RATE,                                             // Sample rate
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
    .sample_rate = SAMPLE_RATE,                          // Sample rate
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
#define BUFFLEN 4096
int16_t audiobuff[BUFFLEN];
size_t audiolen = 0;
bool buffgo = false;

void startup_sound()
{ // JBL START SOUND

  i2s_driver_install(I2S_NUM_0, &i2s_config_mono, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  delay(1000);
  digitalWrite(mutepin, HIGH);

  const size_t chunk_size = 1024; // Size of each chunk
  size_t bytes_written;

  for (size_t i = 0; i < startup_len; i += chunk_size)
  {
    // Calculate the number of bytes to write
    size_t bytes_to_write = (startup_len - i) < chunk_size ? (startup_len - i) * sizeof(int16_t) : chunk_size * sizeof(int16_t);

    // Write the chunk to I2S
    i2s_write(I2S_NUM_0, (const char *)&startup_data[i], bytes_to_write, &bytes_written, portMAX_DELAY);

    // Check if all bytes were written
    if (bytes_written != bytes_to_write)
    {
      Serial.println("Error: Not all bytes were written to I2S");
    }
  }
  // Stop I2S driver
  digitalWrite(mutepin, LOW);
  i2s_driver_uninstall(I2S_NUM_0);
}

void shutdown_sound()
{ // JBL SHUTDOWN SOUND
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config_mono, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  // delay(1000);
  digitalWrite(mutepin, HIGH);

  const size_t chunk_size = 1024; // Size of each chunk
  size_t bytes_written;

  for (size_t i = 0; i < shutdown_len; i += chunk_size)
  {
    // Calculate the number of bytes to write
    size_t bytes_to_write = (shutdown_len - i) < chunk_size ? (shutdown_len - i) * sizeof(int16_t) : chunk_size * sizeof(int16_t);

    // Write the chunk to I2S
    i2s_write(I2S_NUM_0, (const char *)&shutdown[i], bytes_to_write, &bytes_written, portMAX_DELAY);

    // Check if all bytes were written
    if (bytes_written != bytes_to_write)
    {
      Serial.println("Error: Not all bytes were written to I2S");
    }
  }
  // Stop I2S driver
  digitalWrite(mutepin, LOW);
  i2s_driver_uninstall(I2S_NUM_0);
}

void connect_sound()
{ // CONENCT BT
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config_mono, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  // delay(1000);
  digitalWrite(mutepin, HIGH);

  const size_t chunk_size = 1024; // Size of each chunk
  size_t bytes_written;

  for (size_t i = 0; i < conected_len; i += chunk_size)
  {
    // Calculate the number of bytes to write
    size_t bytes_to_write = (conected_len - i) < chunk_size ? (conected_len - i) * sizeof(int16_t) : chunk_size * sizeof(int16_t);

    // Write the chunk to I2S
    i2s_write(I2S_NUM_0, (const char *)&connected[i], bytes_to_write, &bytes_written, portMAX_DELAY);

    // Check if all bytes were written
    if (bytes_written != bytes_to_write)
    {
      Serial.println("Error: Not all bytes were written to I2S");
    }
  }
  // Stop I2S driver
  digitalWrite(mutepin, LOW);
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void max_sound()
{ // MAX VOL INDICATOR
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config_mono, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  // delay(1000);
  digitalWrite(mutepin, HIGH);

  const size_t chunk_size = 1024; // Size of each chunk
  size_t bytes_written;

  for (size_t i = 0; i < max_len; i += chunk_size)
  {
    // Calculate the number of bytes to write
    size_t bytes_to_write = (max_len - i) < chunk_size ? (max_len - i) * sizeof(int16_t) : chunk_size * sizeof(int16_t);

    // Write the chunk to I2S
    i2s_write(I2S_NUM_0, (const char *)&max_data[i], bytes_to_write, &bytes_written, portMAX_DELAY);

    // Check if all bytes were written
    if (bytes_written != bytes_to_write)
    {
      Serial.println("Error: Not all bytes were written to I2S");
    }
  }
  // Stop I2S driver
  digitalWrite(mutepin, LOW);
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void dc_sound()
{ // DC INDICATOR
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config_mono, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  // delay(1000);
  digitalWrite(mutepin, HIGH);

  const size_t chunk_size = 1024; // Size of each chunk
  size_t bytes_written;

  for (size_t i = 0; i < dc_len; i += chunk_size)
  {
    // Calculate the number of bytes to write
    size_t bytes_to_write = (dc_len - i) < chunk_size ? (dc_len - i) * sizeof(int16_t) : chunk_size * sizeof(int16_t);

    // Write the chunk to I2S
    i2s_write(I2S_NUM_0, (const char *)&dc_data[i], bytes_to_write, &bytes_written, portMAX_DELAY);

    // Check if all bytes were written
    if (bytes_written != bytes_to_write)
    {
      Serial.println("Error: Not all bytes were written to I2S");
    }
  }
  // Stop I2S driver
  digitalWrite(mutepin, LOW);
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void audio_data_callback(const uint8_t *data, uint32_t len)
{
  size_t num_samples = len / 4;      // uint8 is 4 bit(LSBL,MSBL,LSBR,MSBR)
  int16_t i2s_data[num_samples * 2]; // uint16 buffer is half length of uint8 buffer

  for (size_t i = 0; i < num_samples; i++)
  {
    int16_t left = (data[i * 4 + 1] << 8) | data[i * 4];      // Left channel
    int16_t right = (data[i * 4 + 3] << 8) | data[i * 4 + 2]; // Right channel

    i2s_data[i * 2] = left;
    i2s_data[i * 2 + 1] = right;
  }
  if (!buffgo)
  {
    memcpy(audiobuff, i2s_data, sizeof(i2s_data));
    audiolen = num_samples * 2;
    buffgo = true;
  }

  size_t i2s_bytes_written;

  i2s_write(I2S_NUM_0, i2s_data, sizeof(i2s_data), &i2s_bytes_written, portMAX_DELAY);
}
const uint16_t fft_samples = 512;
double vReal[fft_samples];
double vImag[fft_samples];
double simplifiedfft[fft_samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, fft_samples, SAMPLE_RATE);

void buffTask(void *pvParameters)
{
  while (true)
  {
    double ema_simplifiedfft0 = 0;
    const double alpha = 0.1; // Smoothing factor (0 < alpha <= 1)
    if (buffgo)
    {
      int pointer = 0;
      bool isdone = false;
      uint16_t totalcp = audiolen / 2;
      
      while (1)
      { 


        if (totalcp >= fft_samples)
        
        {
          for (int i = 0; i < fft_samples; i++)
          {
            vReal[i] = (double)audiobuff[(i + pointer) * 2];
            vImag[i] = 0;
          }
          totalcp -= fft_samples;
          pointer += fft_samples;
        }
        else{
          break;
        }


        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
        FFT.compute(FFTDirection::Forward);                       /* Compute FFT */
        FFT.complexToMagnitude();
        int ranges = 10;
        int range_size = 1000 / ranges;
        int scanindex = 0;
        for (int i = 0; i < ranges; i++)
        {//add value on range
          simplifiedfft[i] = 0;
          int tracker = 0;//buat bagi rata2
          while ((scanindex * SAMPLE_RATE) / fft_samples < range_size * (i + 1))
          {//avg value each range
            float freqnow = (scanindex * SAMPLE_RATE) / fft_samples;
            simplifiedfft[i] += vReal[scanindex];
            scanindex += 1;
            tracker += 1;
          }
          simplifiedfft[i] /= tracker; // Average the values in the range
        }
        ema_simplifiedfft0 = alpha * simplifiedfft[0] + (1 - alpha) * ema_simplifiedfft0;
        // Serial.println(ema_simplifiedfft0);
        int pwmvalue = map(ema_simplifiedfft0, 0, 300000, 0, 255);
        analogWrite(beatLED, pwmvalue);
        memset(vReal, 0, sizeof(vReal));
        if (pointer>=audiolen/2)
        {
          break;
        }
        Serial.flush();
      }
      

      buffgo = false;
    }
    else{
      digitalWrite(beatLED, LOW);
    }
    Serial.flush();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(ppbtn, INPUT);
  pinMode(dcbtn, INPUT);
  pinMode(prev, INPUT);
  pinMode(nextbtn, INPUT);
  pinMode(powerbtn, INPUT);

  pinMode(mosfetpin, OUTPUT);
  pinMode(mutepin, OUTPUT);
  pinMode(beatLED, OUTPUT);
  pinMode(btled, OUTPUT);

  digitalWrite(mosfetpin, HIGH);
  digitalWrite(mutepin, LOW);
  digitalWrite(beatLED, HIGH);
  digitalWrite(btled, LOW);

  startup_sound();

  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  a2dp_sink.set_stream_reader(audio_data_callback, false);
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.start("Harman/Kardon Genie");

  xTaskCreatePinnedToCore(
      buffTask,   // Task function
      "buffTask", // Name of the task
      10000,      // Stack size
      NULL,       // Task input parameter
      1,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run
  );
}

// GLOBAL VARIABLE
int volume = 0;
long btdckeytimer = 0;
bool isplay = true;
bool ignorepp = false;
bool isnext = false;
bool isprev = false;
bool isconnected = false;

void loop()
{
  // CONNECT
  if (a2dp_sink.is_connected())
  {
    if (!isconnected)
    {
      connect_sound();
    }
    isconnected = 1;
    if (a2dp_sink.get_volume() != volume)
    {
      volume = a2dp_sink.get_volume(); // get volume if changed from source
      Serial.println(volume);
    }

    digitalWrite(btled, LOW);
  }
  else
  {
    isconnected = 0;
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
    shutdown_sound();
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
        dc_sound();
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
        volume = volume + 8;
        if (volume > 130)
        {
          max_sound();
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

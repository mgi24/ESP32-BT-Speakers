import wave
import numpy as np

def wav_to_header(input_wav, output_header):
    # Buka file WAV
    with wave.open(input_wav, 'rb') as wav_file:
        # Pastikan file WAV adalah 16-bit mono atau stereo
        assert wav_file.getsampwidth() == 2, "File WAV harus 16-bit"
        assert wav_file.getnchannels() in [1, 2], "File WAV harus mono atau stereo"

        # Baca data WAV
        frames = wav_file.readframes(wav_file.getnframes())
        num_samples = wav_file.getnframes() * wav_file.getnchannels()

        # Konversi data ke array numpy int16
        audio_data = np.frombuffer(frames, dtype=np.int16)

        # Jika stereo, interleave channel kiri dan kanan
        if wav_file.getnchannels() == 2:
            audio_data = audio_data.reshape((-1, 2)).flatten()

        # Panjang total audio
        shutdown_len = len(audio_data)

        # Tulis ke file header
        with open(output_header, 'w') as header_file:
            header_file.write(f'const int dc_len = {shutdown_len};\n')
            header_file.write('const int16_t dc_data[] = {\n')
            for i, sample in enumerate(audio_data):
                if i % 10 == 0:
                    header_file.write('\n')
                header_file.write(f'{sample}, ')
            header_file.write('\n};\n')

# Contoh penggunaan
wav_to_header('dc.wav', 'dc.h')
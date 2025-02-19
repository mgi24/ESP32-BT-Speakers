import serial
import cv2
import numpy as np
import re
# Configure the serial port
ser = serial.Serial('COM3', 115200)  # Adjust 'COM3' to your serial port


# Lists to store the data
start_freqs = []
end_freqs = []
values = []

# Create a 1000x1000 3-channel empty image

image = np.zeros((1000, 1000, 3), dtype=np.uint8)
# Define the frequencies to be displayed
frequencies = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]
spacing = image.shape[1] // (len(frequencies))
while True:
    
    line = ser.readline().decode('utf-8').strip()
    try:
        parsed_values = list(map(float, line.split('/')))
        print(parsed_values)
        for i, value in enumerate(parsed_values):
            value /=1000
            x = int(i * spacing)
            y = int(image.shape[0] - 20 - value)
            cv2.rectangle(image, (x, y), (x+10, image.shape[1]-20), (0, 255, 0), -1)
            
            for i, freq in enumerate(frequencies):
                position = (i * spacing, image.shape[0] - 10)
                cv2.putText(image, str(freq), position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow('spectogram', image)
        image = np.zeros((1000, 1000, 3), dtype=np.uint8)
    except ValueError:
        print(f"Error: {line}")
    if cv2.waitKey(1) & 0xFF == 27:  # 27 is the ASCII code for the ESC key
        break

    

cv2.destroyAllWindows()
ser.close()
    


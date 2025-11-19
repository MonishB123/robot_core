import serial

ser = serial.Serial('COM3', 115200, timeout=1)

ser.write(b'\xA5\x20')

while True:
    header = ser.read(2)
    if len(header) < 2:
        continue
        
    if(header[0] & 1 ) != 1:
        continue
    
    data = ser.read(2)
    if len(data) < 3:
        continue

    S = header[0]
    Q = header[1]

    angle = ((data[1] << 8) | data[0]) / 64.0
    dist = ((data[3] << 8) | data[2]) / 4.0

    print(angle, dist)
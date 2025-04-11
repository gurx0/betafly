import time
import serial
import struct

# Настройки подключения
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200
UPDATE_FREQ = 50  # Hz
UPDATE_PERIOD = 1.0 / UPDATE_FREQ


def send_msp_request(command):
    header = b'$M<'
    size = 0
    checksum = size ^ command
    message = struct.pack('<3sBB', header, size, command) + struct.pack('<B', checksum)
    ser.write(message)


def read_msp_response():
    try:
        header = ser.read(3)
        if header != b'$M>':
            return None

        size = ord(ser.read(1))
        code = ord(ser.read(1))
        data = ser.read(size)
        checksum = ord(ser.read(1))

        return code, data
    except:
        return None


def get_attitude():
    send_msp_request(108)  # MSP_ATTITUDE
    response = read_msp_response()

    if response and response[0] == 108:
        try:
            roll, pitch, yaw = struct.unpack('<hhh', response[1])
            return roll / 10.0, pitch / 10.0, yaw
        except:
            return None
    return None


# Инициализация соединения
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

try:
    print("Starting attitude monitoring...")
    last_update = time.monotonic()

    while True:
        start_time = time.monotonic()

        # Получение данных
        attitude = get_attitude()

        # Вывод данных
        if attitude:
            roll, pitch, yaw = attitude
            print(f"Orientation: Roll: {roll:6.1f}°, Pitch: {pitch:6.1f}°, Yaw: {yaw:6.1f}°")
        else:
            print("Error: Failed to get attitude data")

        # Поддержание частоты обновления
        elapsed = time.monotonic() - start_time
        sleep_time = UPDATE_PERIOD - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print(f"Warning: Can't keep up! Delay: {-sleep_time:.3f}s")

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    ser.close()
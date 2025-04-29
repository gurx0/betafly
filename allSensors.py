import serial
import struct
import time

# Настройки подключения
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200
UPDATE_FREQ = 50  # Hz
UPDATE_PERIOD = 1.0 / UPDATE_FREQ

# Инициализация соединения
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# ====== MSP Utilities ======

def checksum(data):
    return sum(data) & 0xFF

def send_msp_request(command, data=[]):
    payload = struct.pack(f'<{len(data)}B', *data) if data else b''
    size = len(payload)
    chk = checksum([size, command] + list(payload))
    packet = b'$M<' + struct.pack('<B', size) + struct.pack('<B', command) + payload + struct.pack('<B', chk)
    ser.write(packet)

def read_msp_response():
    try:
        header = ser.read(3)
        if header != b'$M>':
            print("[Ошибка] Неверный заголовок MSP:", header)
            return None, None
        size = ord(ser.read(1))
        code = ord(ser.read(1))
        data = ser.read(size)
        checksum_byte = ord(ser.read(1))
        return code, data
    except Exception as e:
        print(f"[Ошибка чтения]: {e}")
        return None, None

# ====== Получение данных ======

def get_attitude():
    send_msp_request(108)  # MSP_ATTITUDE
    code, data = read_msp_response()
    if code == 108 and data and len(data) == 6:
        try:
            roll, pitch, yaw = struct.unpack('<hhh', data)
            return roll / 10.0, pitch / 10.0, yaw
        except:
            print("[Ошибка] Невозможно распарсить attitude данные")
            return None
    print("[Ошибка] Неверный код или данные attitude")
    return None

def get_baro_altitude():
    send_msp_request(109)  # MSP_ALTITUDE
    time.sleep(0.01)  # небольшой интервал перед чтением
    code, data = read_msp_response()
    if code == 109 and data and len(data) == 6:
        try:
            alt, var = struct.unpack('<ih', data)
            return alt / 100.0, var
        except:
            print("[Ошибка] Невозможно распарсить baro данные")
            return None
    print("[Ошибка] Неверный код или данные baro")
    return None

def get_gyro_rates():
    send_msp_request(102)  # MSP_RAW_IMU
    code, data = read_msp_response()
    if code == 102 and data and len(data) == 18:
        try:
            # Данные: acc[3], gyro[3], mag[3] — всего 9 int16 = 18 байт
            unpacked = struct.unpack('<hhhhhhhhh', data)
            gyroX, gyroY, gyroZ = unpacked[3:6]
            return gyroX, gyroY, gyroZ
        except:
            print("[Ошибка] Невозможно распарсить gyro данные")
            return None
    print("[Ошибка] Неверный код или данные gyro")
    return None

# ====== Основной цикл ======

try:
    print("Начинаем мониторинг данных...")

    while True:
        start_time = time.monotonic()

        # Получение данных с сенсоров
        attitude = get_attitude()
        altitude_data = get_baro_altitude()
        gyro_rates = get_gyro_rates()

        # Вывод данных
        if attitude:
            roll, pitch, yaw = attitude
            print(f"Roll: {roll:6.1f}, Pitch: {pitch:6.1f}, Yaw: {yaw:6.1f}")
        else:
            print("[!] Ошибка получения данных attitude")

        if altitude_data:
            altitude, variance = altitude_data
            print(f"alt: {altitude:.2f} м, var: {variance}")
        else:
            print("[!] Ошибка получения данных baro")

        if gyro_rates:
            gx, gy, gz = gyro_rates
            print(f"Gyro: X: {gx:5d}, Y: {gy:5d}, Z: {gz:5d}")
        else:
            print("[!] Ошибка получения данных gyro")

        # Поддержание частоты обновления
        elapsed = time.monotonic() - start_time
        sleep_time = UPDATE_PERIOD - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print(f"Задержка обновления: {-sleep_time:.3f}s")

except KeyboardInterrupt:
    print("\nОстановка")

finally:
    ser.close()
    print("Соединение закрыто")

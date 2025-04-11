import serial
import struct
import time

# Открываем соединение с полетным контроллером
ser = serial.Serial('COM8', 115200, timeout=1)  # Укажи нужный COM порт

# Вычисляем контрольную сумму для MSP-пакета
def checksum(data):
    return sum(data) & 0xFF

# Отправка MSP-команды
def send_msp(command, data=[]):
    payload = struct.pack(f'<{len(data)}B', *data) if data else b''
    size = len(payload)
    chk = checksum([size, command] + list(payload))
    packet = b'$M<' + struct.pack('<B', size) + struct.pack('<B', command) + payload + struct.pack('<B', chk)
    ser.write(packet)

# Чтение ответа от полетного контроллера
def read_msp_response():
    header = ser.read(3)
    if header != b'$M>':
        print("[Ошибка] Неверный заголовок MSP:", header)
        return None, None
    size = ord(ser.read(1))
    code = ord(ser.read(1))
    data = ser.read(size)
    checksum_byte = ord(ser.read(1))
    return code, data

def get_baro_altitude():
    send_msp(109)  # MSP_ALTITUDE
    time.sleep(0.1)
    code, data = read_msp_response()
    if code != 109 or data is None:
        print("[!] Нет ответа или неверный код")
        return

    # Debug: Вывод сырых данных и их длины
    print(f"[Debug] Raw data: {data}, data Length: {len(data)}")

    # Проверка длины данных
    if len(data) != 6:
        print(f"[Error] Ожидалось 6 байт, получено {len(data)} байт")
        return

    # Распаковка: int32 altitude, int16 variance
    alt, var = struct.unpack('<ih', data)
    print(f"[📡] Высота: {alt / 100.0} м")
    print(f"[📊] Дисперсия: {var}")

# Вызов функции
get_baro_altitude()

# Закрытие порта (по желанию)
ser.close()

import serial
import struct
import time

# Подключение к полетному контроллеру
ser = serial.Serial('COM8', 115200, timeout=2)  # Укажи нужный COM-порт

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
        print(f"[Ошибка] Неверный заголовок MSP: {header}")
        return None, None
    size_byte = ser.read(1)
    if not size_byte:
        print("[Ошибка] Не удалось прочитать размер")
        return None, None
    size = ord(size_byte)
    code_byte = ser.read(1)
    if not code_byte:
        print("[Ошибка] Не удалось прочитать код")
        return None, None
    code = ord(code_byte)
    data = ser.read(size)
    if len(data) != size:
        print(f"[Ошибка] Неполные данные: ожидалось {size} байт, получено {len(data)}")
        return None, None
    checksum_byte = ser.read(1)
    if not checksum_byte:
        print("[Ошибка] Не удалось прочитать контрольную сумму")
        return None, None
    received_checksum = ord(checksum_byte)
    calculated_checksum = checksum([size, code] + list(data))
    if received_checksum != calculated_checksum:
        print(f"[Ошибка] Неверная контрольная сумма: ожидалось {calculated_checksum}, получено {received_checksum}")
        return None, None
    return code, data

# Получение режима полета (MSP_STATUS)
def get_flight_mode():
    send_msp(101)  # MSP_STATUS
    time.sleep(0.1)
    code, data = read_msp_response()
    if code != 101 or data is None:
        print("[!] Нет ответа или неверный код")
        return None

    # Проверка длины данных
    if len(data) < 11:
        print(f"[Error] Ожидалось минимум 11 байт, получено {len(data)}")
        return None

    # Распаковка: cycleTime (uint16), i2cError (uint16), sensor (uint16), flag (uint32), globalConf (uint8)
    cycle_time, i2c_error, sensor, flag, global_conf = struct.unpack('<HHHIb', data[:11])
    modes = []

    # Интерпретация флагов режимов (на основе Betaflight)
    if flag & (1 << 0):
        modes.append("ARM")
    if flag & (1 << 1):
        modes.append("ANGLE")
    if flag & (1 << 2):
        modes.append("HORIZON")
    if flag & (1 << 3):
        modes.append("BARO")
    if flag & (1 << 4):
        modes.append("MAG")
    if flag & (1 << 7):
        modes.append("HEADFREE")
    if flag & (1 << 10):
        modes.append("GPS_HOME")
    if flag & (1 << 11):
        modes.append("GPS_HOLD")

    print(f"[📡] Режимы полета: {', '.join(modes) if modes else 'Нет активных режимов'}")
    print(f"[📊] Флаги: {bin(flag)}")
    return modes

# Вызов функции
try:
    get_flight_mode()
finally:
    ser.close()
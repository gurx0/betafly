import serial
import struct
import time

# Открываем соединение с полетным контроллером
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

# Арминг/дизарминг дрона (MSP_SET_RAW_RC)
def set_arm(arm=True):
    # Каналы: roll, pitch, yaw, throttle, aux1 (arming), aux2-4
    roll = 1500  # Центр (нейтраль)
    pitch = 1500
    yaw = 1500
    throttle = 1000  # Минимальный троттл
    aux1 = 2000 if arm else 1000  # AUX1: 2000 для арминга, 1000 для дизарминга
    aux2 = 1000
    aux3 = 1000
    aux4 = 1000

    # Формируем данные (8 каналов, 16 бит каждый)
    channels = [roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4]
    data = []
    for ch in channels:
        data.extend([ch & 0xFF, (ch >> 8) & 0xFF])  # Младший и старший байт

    send_msp(200, data)  # MSP_SET_RAW_RC
    time.sleep(0.1)
    code, _ = read_msp_response()
    if code != 200:
        print(f"[!] Ошибка {'арминга' if arm else 'дизарминга'}")
        return False
    print(f"[✔] Дрон {'армирован' if arm else 'дизармирован'}")
    return True

# Установка троттла
def set_throttle(throttle):
    if not 1000 <= throttle <= 2000:
        print("[!] Троттл должен быть в диапазоне 1000-2000")
        return False

    # Каналы: roll, pitch, yaw, throttle, aux1-4
    roll = 1500
    pitch = 1500
    yaw = 1500
    aux1 = 2000  # Предполагаем, что дрон армирован
    aux2 = 1000
    aux3 = 1000
    aux4 = 1000

    channels = [roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4]
    data = []
    for ch in channels:
        data.extend([ch & 0xFF, (ch >> 8) & 0xFF])

    send_msp(200, data)  # MSP_SET_RAW_RC
    time.sleep(0.1)
    code, _ = read_msp_response()
    if code != 200:
        print("[!] Ошибка установки троттла")
        return False
    print(f"[✔] Троттл установлен: {throttle}")
    return True

# Получение режима полета (MSP_STATUS)
def get_flight_mode():
    send_msp(101)  # MSP_STATUS
    time.sleep(0.1)
    code, data = read_msp_response()
    if code != 101 or data is None:
        print("[!] Нет ответа или неверный код")
        return None

    # Распаковка MSP_STATUS: cycleTime (uint16), i2cError (uint16), sensor (uint16), flag (uint32), globalConf (uint8)
    if len(data) < 11:
        print(f"[Error] Ожидалось минимум 11 байт, получено {len(data)}")
        return None

    cycle_time, i2c_error, sensor, flag, global_conf = struct.unpack('<HHHIb', data[:11])
    modes = []

    # Флаги режимов (пример для Betaflight)
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

# Получение данных с барометра (MSP_ALTITUDE)
def get_baro_altitude():
    send_msp(109)  # MSP_ALTITUDE
    time.sleep(0.1)
    code, data = read_msp_response()
    if code != 109 or data is None:
        print("[!] Нет ответа или неверный код")
        return

    print(f"[Debug] Raw data: {data}, Length: {len(data)}")
    if len(data) != 6:
        print(f"[Error] Ожидалось 6 байт, получено {len(data)} байт")
        return

    alt, var = struct.unpack('<ih', data)
    print(f"[📡] Высота: {alt / 100.0} м")
    print(f"[📊] Дисперсия: {var}")

# Пример использования
try:
    # Проверяем статус
    get_flight_mode()

    # Получаем высоту
    get_baro_altitude()

    # Армируем дрон
    if set_arm(True):
        time.sleep(1)
        # Устанавливаем троттл (например, 1200)
        set_throttle(1200)
        time.sleep(2)
        # Возвращаем троттл на минимум
        set_throttle(1000)
        time.sleep(1)
        # Дизармируем
        set_arm(False)

finally:
    ser.close()
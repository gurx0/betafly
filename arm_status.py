import time
import serial
import struct

SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

# Функция для вычисления правильной XOR контрольной суммы MSP
def msp_checksum(data):
    return sum(data) & 0xFF  # Временная заглушка, замените на правильную реализацию

# Исправленная функция для отправки MSP команд
def send_msp(command, data=[]):
    payload = struct.pack(f'<{len(data)}B', *data) if data else b''
    length = len(payload)
    
    # Правильный расчет контрольной суммы (XOR)
    checksum = length ^ command
    for b in data:
        checksum ^= b
    
    packet = b'$M<' + struct.pack('<BB', length, command) + payload + struct.pack('<B', checksum)
    ser.write(packet)

# Функция для чтения ответов MSP
def read_msp_response():
    try:
        header = ser.read(3)
        if header != b'$M>':
            return None
        
        size = ord(ser.read(1))
        code = ord(ser.read(1))
        data = ser.read(size) if size > 0 else b''
        received_checksum = ord(ser.read(1))
        
        # Проверка контрольной суммы
        calculated_checksum = size ^ code
        for b in data:
            calculated_checksum ^= b
        
        if calculated_checksum != received_checksum:
            return None
        
        return code, data
    except:
        return None

# Получение статуса арма
def get_arm_status():
    send_msp(101)  # MSP_STATUS
    response = read_msp_response()
    
    if response and response[0] == 101:
        try:
            flags = struct.unpack('<H', response[1][6:8])[0]
            armed = bool(flags & 0x01)  # Бит 0 - статус арма
            return "ARMED" if armed else "DISARMED"
        except:
            return "UNKNOWN"
    return "ERROR"

# Команда для арма/дизарма
def arm_disarm(arm_flag):
    send_msp(214, [1 if arm_flag else 0])  # MSP_SET_ARMING
    time.sleep(0.5)  # Даем время на обработку команды

# Основная процедура
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

try:
    # Первоначальный статус
    print(f"Initial status: {get_arm_status()}")
    
    # Выполняем арм
    print("Sending ARM command...")
    arm_disarm(True)
    
    # Проверяем статус после арма
    print(f"Status after ARM: {get_arm_status()}")
    
    # Необязательно: дизарм
    print("Sending DISARM command...")
    arm_disarm(False)
    print(f"Final status: {get_arm_status()}")

except Exception as e:
    print(f"Error: {str(e)}")
finally:
    ser.close()
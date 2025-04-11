import serial
import struct
import time

ser = serial.Serial('COM8', 115200, timeout=1)

def msp_checksum(data):
    """Правильная XOR контрольная сумма для MSP протокола"""
    crc = 0
    for b in data:
        crc ^= b
    return crc

def send_msp(command, data=[]):
    """Улучшенная функция отправки MSP команд"""
    payload = struct.pack(f'<{len(data)}B', *data) if data else b''
    length = len(payload)
    
    # Формируем данные для контрольной суммы
    crc_data = [length, command] + list(payload)
    checksum_value = msp_checksum(crc_data)
    
    # Собираем пакет
    packet = b'$M<' + struct.pack('<BB', length, command) + payload + struct.pack('<B', checksum_value)
    ser.write(packet)
    ser.flush()  # Важно: очищаем буфер

def read_msp_response():
    """Улучшенное чтение ответов с обработкой ошибок"""
    try:
        # Читаем заголовок
        header = ser.read(3)
        if header != b'$M>':
            print(f"[ERROR] Invalid header: {header}")
            return None
        
        # Читаем размер данных
        size = ord(ser.read(1))
        code = ord(ser.read(1))
        
        # Читаем данные
        data = ser.read(size) if size > 0 else b''
        
        # Читаем и проверяем контрольную сумму
        received_checksum = ord(ser.read(1))
        calculated_checksum = msp_checksum([size, code] + list(data))
        
        if calculated_checksum != received_checksum:
            print("[ERROR] Checksum mismatch")
            return None
            
        return (code, data)
        
    except Exception as e:
        print(f"[ERROR] Read error: {str(e)}")
        return None

def get_boxnames():
    """Получение списка режимов с обработкой ошибок"""
    send_msp(116)  # MSP_BOXNAMES
    time.sleep(0.2)  # Увеличиваем задержку
    
    response = read_msp_response()
    if not response:
        return []
        
    code, data = response
    if code != 116:
        print(f"[ERROR] Unexpected response code: {code}")
        return []
    
    try:
        names = data.split(b'\x00')
        return [n.decode('utf-8') for n in names if n]
    except Exception as e:
        print(f"[ERROR] Decoding error: {str(e)}")
        return []

def get_active_modes():
    """Получение активных режимов с проверками"""
    send_msp(113)  # MSP_BOX
    time.sleep(0.2)
    
    response = read_msp_response()
    if not response:
        return None
        
    code, data = response
    if code != 113:
        print(f"[ERROR] Unexpected response code: {code}")
        return None
    
    try:
        if len(data) % 4 != 0:
            print("[ERROR] Invalid data length")
            return None
            
        count = len(data) // 4
        return [struct.unpack('<I', data[i*4:(i+1)*4])[0] for i in range(count)]
    except Exception as e:
        print(f"[ERROR] Unpack error: {str(e)}")
        return None

def print_active_modes():
    """Безопасный вывод режимов"""
    names = get_boxnames()
    if not names:
        print("Failed to get mode names")
        return
        
    masks = get_active_modes()
    if not masks:
        print("Failed to get active modes")
        return
    
    print("\n[INFO] Active Modes:")
    for i, name in enumerate(names):
        if i >= 32:  # Ограничение по битовой маске
            break
        if masks[0] & (1 << i):
            print(f" - {name}")

# Перед использованием проверяем соединение
if __name__ == "__main__":
    try:
        # Тест соединения
        if not ser.is_open:
            ser.open()
            
        # Сбрасываем буфер
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print("Connection established. Reading modes...")
        print_active_modes()
        
    except Exception as e:
        print(f"Fatal error: {str(e)}")
    finally:
        ser.close()
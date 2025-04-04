import serial
import struct

port_name = "COM8"  # Используйте найденный порт


# Функция вычисления контрольной суммы
def checksum(data):
    return sum(data) & 0xFF


# Функция для отправки команды MSP
def send_msp(command, data=[]):
    payload = struct.pack(f'<{len(data)}B', *data) if data else b''
    length = len(payload)
    checksum_value = checksum([length, command] + list(data))

    packet = b'$M<' + struct.pack('<BB', length, command) + payload + struct.pack('<B', checksum_value)
    return packet


try:
    # Открываем соединение
    ser = serial.Serial(port_name, 115200, timeout=2)
    print(f"Соединение установлено с {port_name}")

    # Отправляем команду MSP_STATUS
    ser.write(send_msp(101))

    # Ждем ответ
    response = ser.read(20)  # Читаем первые 20 байт ответа
    print(f"Получен ответ: {response}")

except serial.SerialException as e:
    print(f"Ошибка соединения: {e}")

finally:
    if ser.is_open:
        ser.close()

import serial
import struct
import time

# Открываем соединение с контроллером полета
ser = serial.Serial('COM8', 115200, timeout=1)  # Замените 'COM3' на ваш порт

# Функция для вычисления контрольной суммы MSP
def checksum(data):
    return sum(data) & 0xFF


# Функция для отправки команды MSP
def send_msp(command, data=[]):
    payload = struct.pack(f'<{len(data)}B', *data) if data else b''
    length = len(payload)
    checksum_value = checksum([length] + [command] + list(data))

    packet = b'$M<' + struct.pack('<BB', length, command) + payload + struct.pack('<B', checksum_value)
    ser.write(packet)


# **Команда ARM (включение моторов)**
def arm():
    send_msp(214, [1])  # MSP_SET_ARMING_FLAG


# **Команда THROTTLE (установка газа)**
def set_throttle(value):
    send_msp(200, [100, value, 100, 100])  # MSP_SET_RAW_RC


# Основной цикл управления
try:
    print("Arming the drone...")
    arm()
    time.sleep(2)  # Ожидаем активации

    print("Setting throttle to 1500...")
    set_throttle(1500)
    time.sleep(5)  # Ждём 5 секунд

    print("Disarming...")
    send_msp(214, [0])  # MSP_DISARM
except KeyboardInterrupt:
    send_msp(214, [0])  # MSP_DISARM перед выходом
    print("\nDisarmed the drone.")
finally:
    ser.close()

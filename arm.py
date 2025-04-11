import struct
import serial
import time

# Пример инициализации порта
ser = serial.Serial("COM8", baudrate=115200, timeout=1)

def send_msp(code, data=None):
    header = b'$M<'
    data_bytes = struct.pack('<' + 'H' * len(data), *data) if data else b''
    size = len(data_bytes)
    checksum = size ^ code
    for b in data_bytes:
        checksum ^= b
    packet = header + bytes([size]) + bytes([code]) + data_bytes + bytes([checksum])
    ser.write(packet)

def arm():
    send_msp(214, [1])  # MSP_SET_ARM (может отличаться, зависит от Betaflight настроек)

def set_throttle(throttle_value):
    roll = 1500
    pitch = 1500
    yaw = 1500
    aux1 = 1000
    send_msp(200, [roll, pitch, throttle_value, yaw, aux1, 1000, 1000, 1000])  # MSP_SET_RAW_RC

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

def disarm():
    send_msp(214, [0])  # MSP_DISARM

# **Команда THROTTLE (установка газа)**
def set_throttle(value):
    send_msp(200, [220, value, 200, 200])  # MSP_SET_RAW_RC


# Основной цикл управления
try:
    print("Arming the drone...")
    arm()
    time.sleep(2)  # Ожидаем активации

    print("Setting throttle to 1500...")
    set_throttle(20)
    time.sleep(3)  # Ждём 5 секунд

    disarm()
    print("Disarming...")
except KeyboardInterrupt:
    send_msp(214, [0])  # MSP_DISARM перед выходом
    print("\nDisarmed the drone.")
finally:
    ser.close()

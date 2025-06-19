#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Windows Uyumlu INAV SpeedyBee F405 Test Programı
Unicode sorunları çözülmüş, detaylı hata analizi eklenmiş
JSON dosyasına veri kaydetme özelliği eklenmiş
"""

import serial
import serial.tools.list_ports
import struct
import time
import threading
import logging
import sys
import os
import json
from datetime import datetime

# Windows konsol UTF-8 desteği
if sys.platform == "win32":
    os.system("chcp 65001 > nul")

# Logging ayarları - Unicode karakterler kaldırılmış
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('speedybee_test.log', encoding='utf-8')
    ]
)
logger = logging.getLogger(__name__)


class MSPProtocol:
    """MSP Protokol sınıfı - INAV uyumlu"""

    # MSP Komutları
    MSP_IDENT = 100
    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_ALTITUDE = 109
    MSP_ATTITUDE = 108
    MSP_ANALOG = 110
    MSP_SET_RAW_RC = 200
    MSP_ARM = 151
    MSP_DISARM = 152

    def __init__(self, port, baudrate=115200, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.armed = False
        self.connected = False

    def connect(self):
        """Seri porta bağlan ve test et"""
        try:
            logger.info(f"SpeedyBee baglantisi kuruluyor: {self.port} @ {self.baudrate}")

            # Serial bağlantı kur
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                rtscts=False,
                dsrdtr=False
            )

            # Kısa bekle
            time.sleep(1)

            # Buffer'ı temizle
            self.serial.flushInput()
            self.serial.flushOutput()

            print(f"Seri port acildi: {self.port}")

            # Bağlantı testi
            if self.test_connection():
                self.connected = True
                logger.info("SpeedyBee baglantisi basarili!")
                return True
            else:
                logger.error("SpeedyBee baglanti testi basarisiz")
                return False

        except serial.SerialException as e:
            logger.error(f"Seri port hatasi: {e}")
            print(f"Seri port hatasi: {e}")
            return False
        except Exception as e:
            logger.error(f"Baglanti hatasi: {e}")
            print(f"Baglanti hatasi: {e}")
            return False

    def test_connection(self):
        """Bağlantıyı test et"""
        print("Baglanti test ediliyor...")

        # Birkaç farklı komut dene
        test_commands = [self.MSP_IDENT, self.MSP_STATUS, self.MSP_ATTITUDE]

        for cmd in test_commands:
            try:
                print(f"Komut test ediliyor: {cmd}")
                self.send_command(cmd)

                # Yanıt bekle (daha uzun timeout)
                start_time = time.time()
                while time.time() - start_time < 3:  # 3 saniye bekle
                    if self.serial.in_waiting > 0:
                        response = self.read_response()
                        if response:
                            print(f"Basarili yanit alindi! Komut: {cmd}")
                            return True
                    time.sleep(0.1)

                print(f"Komut {cmd} icin yanit alinamadi")

            except Exception as e:
                print(f"Test hatasi: {e}")
                continue

        # Manuel veri gönderme testi
        print("Manuel veri gonderme testi...")
        try:
            # Basit MSP_IDENT komutu gönder
            test_packet = b'$M<\x00\x64\x64'  # MSP_IDENT
            self.serial.write(test_packet)
            time.sleep(0.5)

            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                print(f"Ham veri alindi: {data.hex()}")
                return True

        except Exception as e:
            print(f"Manuel test hatasi: {e}")

        return False

    def calculate_checksum(self, data):
        """MSP checksum hesaplama"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def send_command(self, command, data=None):
        """MSP komutu gönder"""
        if not self.serial or not self.serial.is_open:
            return False

        if data is None:
            data = []

        try:
            # MSP header: $M<
            header = [ord('$'), ord('M'), ord('<')]

            # Data length
            data_length = len(data)

            # Command
            cmd = command

            # Checksum hesapla
            checksum_data = [data_length, cmd] + data
            checksum = self.calculate_checksum(checksum_data)

            # Paketi oluştur
            packet = header + [data_length, cmd] + data + [checksum]

            # Gönder
            self.serial.write(bytes(packet))
            logger.debug(f"Komut gonderildi: {command}, Data: {data}")
            return True

        except Exception as e:
            logger.error(f"Komut gonderme hatasi: {e}")
            return False

    def read_response(self, timeout=2):
        """MSP yanıtını oku"""
        if not self.serial or not self.serial.is_open:
            return None

        try:
            start_time = time.time()

            # Header bekle: $M>
            while time.time() - start_time < timeout:
                if self.serial.in_waiting > 0:
                    byte = self.serial.read(1)
                    if byte == b'$':
                        if self.serial.read(1) == b'M':
                            direction = self.serial.read(1)
                            if direction == b'>':
                                break
                else:
                    time.sleep(0.01)
            else:
                return None

            # Data length
            length_byte = self.serial.read(1)
            if not length_byte:
                return None
            data_length = ord(length_byte)

            # Command
            cmd_byte = self.serial.read(1)
            if not cmd_byte:
                return None
            command = ord(cmd_byte)

            # Data
            data = []
            for _ in range(data_length):
                data_byte = self.serial.read(1)
                if not data_byte:
                    return None
                data.append(ord(data_byte))

            # Checksum
            checksum_byte = self.serial.read(1)
            if not checksum_byte:
                return None
            received_checksum = ord(checksum_byte)

            # Checksum doğrula
            calculated_checksum = self.calculate_checksum([data_length, command] + data)

            if received_checksum != calculated_checksum:
                logger.warning(f"Checksum hatasi! Beklenen: {calculated_checksum}, Alinan: {received_checksum}")
                return None

            logger.debug(f"Yanit alindi: Komut={command}, Data uzunlugu={data_length}")
            return {'command': command, 'data': data}

        except Exception as e:
            logger.error(f"Yanit okuma hatasi: {e}")
            return None

    def get_status(self):
        """Drone durumunu al"""
        if not self.send_command(self.MSP_STATUS):
            return None

        response = self.read_response()

        if response and len(response['data']) >= 11:
            data = response['data']

            try:
                # Cycle time
                cycle_time = struct.unpack('<H', bytes(data[0:2]))[0]

                # I2C errors
                i2c_errors = struct.unpack('<H', bytes(data[2:4]))[0]

                # Sensors
                sensors = struct.unpack('<H', bytes(data[4:6]))[0]

                # Flight mode flags
                flight_mode = struct.unpack('<L', bytes(data[6:10]))[0]

                # Profile
                profile = data[10]

                # ARM durumu kontrol et
                self.armed = bool(flight_mode & (1 << 0))  # ARM bayrağı

                return {
                    'cycle_time': cycle_time,
                    'i2c_errors': i2c_errors,
                    'sensors': sensors,
                    'flight_mode': flight_mode,
                    'profile': profile,
                    'armed': self.armed
                }
            except Exception as e:
                logger.error(f"Status parsing hatasi: {e}")
                return None
        return None

    def get_attitude(self):
        """Açısal pozisyon verilerini al"""
        if not self.send_command(self.MSP_ATTITUDE):
            return None

        response = self.read_response()

        if response and len(response['data']) >= 6:
            try:
                # Roll, Pitch, Yaw (0.1 derece cinsinden)
                roll = struct.unpack('<h', bytes(response['data'][0:2]))[0] / 10.0
                pitch = struct.unpack('<h', bytes(response['data'][2:4]))[0] / 10.0
                yaw = struct.unpack('<h', bytes(response['data'][4:6]))[0]

                return {'roll': roll, 'pitch': pitch, 'yaw': yaw}
            except Exception as e:
                logger.error(f"Attitude parsing hatasi: {e}")
                return None
        return None

    def get_raw_imu(self):
        """Ham IMU verilerini al"""
        if not self.send_command(self.MSP_RAW_IMU):
            return None

        response = self.read_response()

        if response and len(response['data']) >= 18:
            try:
                data = response['data']

                # Accelerometer (X, Y, Z)
                acc_x = struct.unpack('<h', bytes(data[0:2]))[0]
                acc_y = struct.unpack('<h', bytes(data[2:4]))[0]
                acc_z = struct.unpack('<h', bytes(data[4:6]))[0]

                # Gyroscope (X, Y, Z)
                gyro_x = struct.unpack('<h', bytes(data[6:8]))[0]
                gyro_y = struct.unpack('<h', bytes(data[8:10]))[0]
                gyro_z = struct.unpack('<h', bytes(data[10:12]))[0]

                # Magnetometer (X, Y, Z)
                mag_x = struct.unpack('<h', bytes(data[12:14]))[0]
                mag_y = struct.unpack('<h', bytes(data[14:16]))[0]
                mag_z = struct.unpack('<h', bytes(data[16:18]))[0]

                return {
                    'accelerometer': {'x': acc_x, 'y': acc_y, 'z': acc_z},
                    'gyroscope': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
                    'magnetometer': {'x': mag_x, 'y': mag_y, 'z': mag_z}
                }
            except Exception as e:
                logger.error(f"IMU parsing hatasi: {e}")
                return None
        return None

    def get_altitude(self):
        """Barometrik irtifa verilerini al"""
        if not self.send_command(self.MSP_ALTITUDE):
            return None

        response = self.read_response()

        if response and len(response['data']) >= 6:
            try:
                # Barometric altitude (cm cinsinden)
                altitude = struct.unpack('<l', bytes(response['data'][0:4]))[0]
                # Vertical velocity (cm/s cinsinden)
                vario = struct.unpack('<h', bytes(response['data'][4:6]))[0]

                return {'altitude_cm': altitude, 'altitude_m': altitude / 100.0, 'vario': vario}
            except Exception as e:
                logger.error(f"Barometric altitude parsing hatasi: {e}")
                return None
        return None

    def get_analog(self):
        """Analog verilerini al (voltaj, akım, RSSI)"""
        if not self.send_command(self.MSP_ANALOG):
            return None

        response = self.read_response()

        if response and len(response['data']) >= 7:
            try:
                data = response['data']

                # Voltage (0.1V cinsinden)
                voltage = data[0] / 10.0

                # mAh used
                mah_drawn = struct.unpack('<H', bytes(data[1:3]))[0]

                # RSSI
                rssi = struct.unpack('<H', bytes(data[3:5]))[0]

                # Amperage (0.01A cinsinden)
                amperage = struct.unpack('<h', bytes(data[5:7]))[0] / 100.0

                return {
                    'voltage': voltage,
                    'mah_drawn': mah_drawn,
                    'rssi': rssi,
                    'amperage': amperage
                }
            except Exception as e:
                logger.error(f"Analog parsing hatasi: {e}")
                return None
        return None

    def close(self):
        """Bağlantıyı kapat"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("Baglanti kapatildi")


class JSONLogger:
    """JSON dosyasına veri kaydetme sınıfı - tek okuma formatı"""

    def __init__(self, filename="speedybee_data.json"):
        self.filename = filename
        self.session_start = datetime.now().isoformat()

    def save_reading(self, status_data, attitude_data, imu_data, altitude_data):
        """Tek veri okumasını kaydet - her seferinde dosyayı güncelle"""
        current_data = {
            "session_start": self.session_start,
            "readings": [
                {
                    "timestamp": datetime.now().isoformat(),
                    "status": status_data,
                    "attitude": attitude_data,
                    "imu": imu_data,
                    "altitude": altitude_data
                }
            ]
        }

        try:
            with open(self.filename, 'w', encoding='utf-8') as f:
                json.dump(current_data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error(f"JSON kaydetme hatasi: {e}")


def find_com_ports():
    """Kullanılabilir COM portlarını bul"""
    ports = serial.tools.list_ports.comports()
    available_ports = []

    print("Kullanilabilir COM Portlari:")
    print("-" * 50)

    for port in ports:
        print(f"Port: {port.device}")
        print(f"Aciklama: {port.description}")
        print(f"Donanim ID: {port.hwid}")
        print("-" * 30)
        available_ports.append(port.device)

    return available_ports


def test_imu_continuous():
    """Sürekli IMU ve durum testi"""
    print("=== Surekli IMU ve Durum Testi ===")
    print("Ctrl+C ile cikabilirsiniz")
    print("Veriler 'speedybee_data.json' dosyasina kaydedilecek (her okumada guncellenecek)")
    print()

    # JSON logger başlat
    json_logger = JSONLogger()
    print(f"JSON kayit dosyasi hazir: {json_logger.filename}")

    # Port seçimi
    ports = find_com_ports()
    if not ports:
        print("Hicbir COM portu bulunamadi!")
        return

    print(f"Bulunan portlar: {', '.join(ports)}")
    port = input("Kullanmak istedigin portu gir (orn: COM3) veya 'auto' yazarak otomatik tespit: ")

    if port.lower() == 'auto':
        # USB-Serial adaptör ara
        usb_ports = [p for p in ports if 'USB' in p or 'Serial' in p]
        if usb_ports:
            port = usb_ports[0]
            print(f"Otomatik secilen port: {port}")
        else:
            port = ports[0]
            print(f"Ilk port secildi: {port}")

    # Bağlantı kur
    msp = MSPProtocol(port)

    try:
        if not msp.connect():
            print("Baglanti kurulamadi!")
            print("Sorun giderme:")
            print("1. USB kablosu kontrol edin")
            print("2. SpeedyBee guc aliyor mu?")
            print("3. INAV Configurator kapali mi?")
            print("4. Dogru COM portu secildi mi?")
            return

        print("Baglanti basarili! Veri okuma basliyor...")
        print("Her okuma JSON dosyasinda guncellenecek...")
        print("=" * 80)

        reading_count = 0

        while True:
            reading_count += 1

            # Durum bilgisi
            status = msp.get_status()
            if status:
                arm_status = "ARMED" if status['armed'] else "DISARMED"
                print(f"Durum: {arm_status:8} | Cycle: {status['cycle_time']:4}us | I2C Err: {status['i2c_errors']:3}")
            else:
                print("Durum: ALINAMADI")

            # Açısal pozisyon
            attitude = msp.get_attitude()
            if attitude:
                print(
                    f"Acilar: Roll={attitude['roll']:6.1f}° Pitch={attitude['pitch']:6.1f}° Yaw={attitude['yaw']:6.1f}°")
            else:
                print("Acilar: ALINAMADI")

            # IMU verileri
            imu = msp.get_raw_imu()
            if imu:
                acc = imu['accelerometer']
                gyro = imu['gyroscope']
                print(f"ACC: X={acc['x']:6} Y={acc['y']:6} Z={acc['z']:6}")
                print(f"GYRO: X={gyro['x']:6} Y={gyro['y']:6} Z={gyro['z']:6}")
            else:
                print("IMU: ALINAMADI")

            # İrtifa (Barometrik)
            altitude = msp.get_altitude()
            if altitude:
                print(f"Barometre: {altitude['altitude_m']:6.2f}m | Dikey hiz: {altitude['vario']:4}cm/s")
            else:
                print("Barometre: ALINAMADI")

            # JSON dosyasına kaydet - her okumada üzerine yaz
            try:
                json_logger.save_reading(status, attitude, imu, altitude)
                print(f"[JSON] Guncel veri kaydedildi")
            except Exception as e:
                print(f"[JSON] Kaydetme hatasi: {e}")

            print("-" * 80)
            time.sleep(1)

    except KeyboardInterrupt:
        print(f"\nTest durduruldu. Son okuma '{json_logger.filename}' dosyasinda sakli.")
    except Exception as e:
        print(f"Hata: {e}")
        logger.error(f"Test hatasi: {e}")
    finally:
        msp.close()


def main():
    """Ana program"""
    print("Windows SpeedyBee F405 Test Programi - JSON Kayit Ozelligi")
    print("GUVENLIK: Pervaneleri cikarin!")
    print("=" * 70)

    try:
        test_imu_continuous()
    except Exception as e:
        print(f"Program hatasi: {e}")
        logger.error(f"Ana program hatasi: {e}")


if __name__ == "__main__":
    main()
import serial
import pandas as pd
import sys
import os
import msvcrt

# 시리얼 포트와 보드레이트를 환경에 맞게 수정하세요
SERIAL_PORT = 'COM20'  # 예: 'COM3', '/dev/ttyACM0'
BAUDRATE = 115200
CSV_FILENAME = 'idle.csv'

# 데이터 저장용 리스트
data_rows = []
logging = False
print('스페이스바를 누르면 녹화 시작/종료가 토글됩니다.')

def get_new_csv_filename(base_name):
    name, ext = os.path.splitext(base_name)
    i = 1
    while True:
        new_name = f"{name}_{i}{ext}"
        if not os.path.exists(new_name):
            return new_name
        i += 1

try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"Serial port {SERIAL_PORT} opened. Waiting for data...")
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

try:
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b' ':
                logging = not logging
                if logging:
                    print('녹화 시작!')
                else:
                    print('녹화 종료!')
                    break  # 녹화 종료 시 루프 탈출
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue
        values = line.split(',')
        if len(values) != 6:
            print(f"Invalid data: {line}")
            continue
        if logging:
            data_rows.append(values)
            print(f"Logging: {values}")
except KeyboardInterrupt:
    print("Interrupted by user. Exiting...")
finally:
    ser.close()
    print("Serial port closed.")
    if data_rows:
        new_csv = get_new_csv_filename(CSV_FILENAME)
        df = pd.DataFrame(data_rows)
        df.to_csv(new_csv, index=False, header=False)
        print(f"{new_csv} 파일로 저장 완료.")
    else:
        print("저장할 데이터가 없습니다.")

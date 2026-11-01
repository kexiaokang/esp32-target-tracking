"""抓 COM10 串口日志：先硬复位板子，再读 N 秒，存到文件。

用法：
  python serial_log.py <端口> <秒数> <输出文件>
"""
import sys
import time
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".pylib"))
import serial  # noqa: E402

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
out = sys.argv[3] if len(sys.argv) > 3 else "serial.log"

ser = serial.Serial(port, 115200, timeout=0.2)
# 经典 ESP32 复位时序：DTR 控制 IO0，RTS 控制 EN
ser.setDTR(False)   # IO0 = HIGH（正常启动，不进下载模式）
ser.setRTS(True)    # EN = LOW
time.sleep(0.15)
ser.setRTS(False)   # EN = HIGH -> 运行
time.sleep(0.05)
ser.reset_input_buffer()

t0 = time.time()
buf = bytearray()
while time.time() - t0 < secs:
    chunk = ser.read(4096)
    if chunk:
        buf.extend(chunk)
ser.close()

text = buf.decode("utf-8", errors="replace")
with open(out, "w", encoding="utf-8") as f:
    f.write(text)
print(text)

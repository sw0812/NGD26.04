import serial, time

ser = serial.Serial('/dev/ttyTHS0', 115200, timeout=1)

tx_data = [10, 20, 30, 255, 7]
data = bytes(tx_data)

ser.write(data)
print("TX에서 보낸 값[" + ",".join(str(v) for v in tx_data) + "]")

time.sleep(0.1)
rx_data = list(ser.read(len(tx_data)))
print("RX에서 받은 값[" + ",".join(str(v) for v in rx_data) + "]")

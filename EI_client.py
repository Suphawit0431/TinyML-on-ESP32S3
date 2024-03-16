import serial
import rpc


rpc_master = rpc.rpc_usb_vcp_master("/dev/cu.usbmodem1401")
# result = rpc_master.call("digital_read", recv_timeout=10000)
result = rpc_master.call("jpeg_image_snapshot", recv_timeout=10000)

# print(result)
result = int.from_bytes(result.tobytes(), "little")
print(result)


# import serial.tools.list_ports

# # List available serial ports
# ports = list(serial.tools.list_ports.comports())
# for port in ports:
#     print(port)

# # Replace "/dev/cu.usbserial" with the actual port name you found
# port_name = "/dev/cu.usbserial-018BF9F6"

# # Set the Latency Timer value
# ser = serial.Serial(port_name)
# ser.setLatencyTimer(1)  # Replace 1 with your desired latency value in milliseconds
# ser.close()

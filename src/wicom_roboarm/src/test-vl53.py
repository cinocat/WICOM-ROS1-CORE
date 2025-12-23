import time
import board
import busio
import adafruit_tca9548a
import adafruit_vl53l1x
import adafruit_vl53l0x

# Create I2C bus and TCA9548A multiplexer
i2c = busio.I2C(board.SCL, board.SDA)
tca = adafruit_tca9548a.TCA9548A(i2c)

# Initialize VL53L1X (on channel 1)
print("Initializing VL53L1X on channel 1...")
try:
    vl53l1x = adafruit_vl53l1x.VL53L1X(tca[1])
    vl53l1x.start_ranging()
    print("VL53L1X initialized!")
except Exception as e:
    print(f"VL53L1X init error: {e}")
    vl53l1x = None

# Initialize VL53L0X (on channel 0)
print("Initializing VL53L0X on channel 0...")
try:
    vl53l0x = adafruit_vl53l0x.VL53L0X(tca[0])
    print("VL53L0X initialized!")
except Exception as e:
    print(f"VL53L0X init error: {e}")
    vl53l0x = None

# Offset (tự đo thực tế)
VL53L0X_OFFSET = 32   # mm

while True:
    try:
        # VL53L1X trả về cm, đổi sang mm
        if vl53l1x and vl53l1x.data_ready:
            distance1 = vl53l1x.distance * 10
        else:
            distance1 = None

        # VL53L0X trả về mm nhưng sai lệch, bù offset
        if vl53l0x:
            d2 = vl53l0x.range - VL53L0X_OFFSET
            distance2 = max(d2, 0)
        else:
            distance2 = None

        print(f"VL53L1X (channel 1): {distance1} mm | VL53L0X (channel 0): {distance2} mm")

    except Exception as e:
        print(f"Reading error: {e}")

    time.sleep(0.2)


import time
import smbus2
import threading

MOTOR_ADDRESS = 31
ENCODER_MOTOR_MODULE_ADDRESS = 0x34


class Motor:
    def __init__(self,
                 i2c_port,
                 motor_id):
        self.i2c_port = i2c_port
        self.motor_id = motor_id
        self.address = MOTOR_ADDRESS + self.motor_id - 1
        self.speed = 0
        self.lock = threading.Lock()

    def set_speed(self, speed):
        speed = 100 if speed > 100 else speed
        speed = -100 if speed < -100 else speed
        with self.lock:
            with smbus2.SMBus(self.i2c_port) as bus:
                bus.write_i2c_block_data(MCU_ADDRESS, self.address, [speed])
                self.speed = speed

class EncoderMotorController:
    def __init__(self, i2c_port, motor_type=3):
        self.i2c_port = i2c_port
        self.motor_type = motor_type
        self.lock = threading.Lock()
        self.initialize()

    def initialize(self, retries=3):
        for attempt in range(retries):
            try:
                with smbus2.SMBus(self.i2c_port) as bus:
                    bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDRESS, 20, self.motor_type)
                    return True
            except (TimeoutError, IOError) as e:
                print(f"初始化失败 (尝试 {attempt+1}): {e}")
                if attempt == retries - 1:
                    return False
                time.sleep(0.2)

    def set_speed(self, speed, motor_id=None, offset=0):
        with self.lock:
            if motor_id is None:
                return self._set_multi_speed(speed, offset)
            return self._set_single_speed(motor_id, speed)

    def _set_single_speed(self, motor_id, speed):
        if not 0 < motor_id < 5:
            raise ValueError("电机ID需在1-4范围内")
        
        speed_val = max(min(127, speed), -128)
        return self._i2c_retry_write(
            register=50 + motor_id,
            data=[speed_val],
            operation=f"设置电机{motor_id}速度"
        )

    def _set_multi_speed(self, speeds, offset):
        validated = [max(min(127, s), -128) for s in speeds]
        return self._i2c_retry_write(
            register=51 + offset,
            data=validated,
            operation=f"批量设置速度 offset:{offset}"
        )

    def _i2c_retry_write(self, register, data, operation="", retries=3):
        for attempt in range(retries):
            try:
                with smbus2.SMBus(self.i2c_port) as bus:
                    bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, register, data)
                    return True
            except (TimeoutError, IOError) as e:
                print(f"{operation} 失败 (尝试 {attempt+1}): {e}")
                
                if attempt == retries - 1:
                    print("尝试重新初始化...")
                    if self.initialize():
                        print("重新初始化成功，重试操作")
                        return self._i2c_retry_write(register, data, operation, retries=1)
                    return False
                
                time.sleep(0.1 * (attempt + 1))

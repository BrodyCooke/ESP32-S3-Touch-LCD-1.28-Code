from machine import I2C, Pin
import time


def twos_complement_16bit(value):
    """Converts a 16-bit two's complement number to a signed decimal integer."""
    if value & 0x8000:  # Check if the sign bit (15th bit) is set
        return value - 0x10000  # Convert to negative value
    return value

# Define constants for the QMI8658
QMI8658_SLAVE_ADDR_L = 0x6A
QMI8658_SLAVE_ADDR_H = 0x6B
ONE_G = 9.80665


# Constants for registers (you will need to update these with actual register values)
QMI8658Register_Ctrl1 = 0b00000010
QMI8658Register_Ctrl2 = 0b00000011
QMI8658Register_Ctrl3 = 0b00000100
QMI8658Register_Ctrl5 = 0b00000110
QMI8658Register_Ctrl6 = 0b00000111
QMI8658Register_Ctrl7 = 0b00001000
QMI8658Register_Tempearture_L = 0b00110011
QMI8658Register_Ax_L = 0b00110101
QMI8658Register_Gx_L = 0b00111011
QMI8658Register_dVX_L = 0b01010001

# Scale factors
acc_lsb_div = 1 << 12  # Example for 8g range
gyro_lsb_div = 64  # Example for 512dps range

# flags
hpf_enable = 0
lpf_enable = 1

# Initialize I2C
i2c = I2C(1, scl=Pin(7), sda=Pin(6))  # Adjust pins and frequency as per your board

class QMI8658:
    def __init__(self, address=QMI8658_SLAVE_ADDR_H):
        self.address = address

    def write_reg(self, reg, value):
        #print(bin(value))
        #print(bytes([value]))
        i2c.writeto_mem(self.address, reg, bytes([value]))
        #print(self.read_reg(reg,1))

    def read_reg(self, reg, length=1):
        return i2c.readfrom_mem(self.address, reg, length)
    
    def enable_sensors(self):
        self.write_reg(QMI8658Register_Ctrl1, 0x60)
        self.write_reg(QMI8658Register_Ctrl7, 0x0B)
        print("Ctrl7: ",bin(0x0B))
        
        
        

    def configure_acc(self, range_setting, odr):
        global acc_lsb_div
        if range_setting == 0x00:
            acc_lsb_div = 1 << 14
        elif range_setting == 0x10:
            acc_lsb_div = 1 << 13
        elif range_setting == 0x20:
            acc_lsb_div = 1 << 12
        elif range_setting == 0x30:
            acc_lsb_div = 1 << 11
        else:
            range_setting = 0x20
            acc_lsb_div = 1 << 12
        if hpf_enable:
            self.write_reg(QMI8658Register_Ctrl2, range_setting | odr | 0x80)
        else:
            print("Ctrl2: ", bin(range_setting | odr))
            self.write_reg(QMI8658Register_Ctrl2, range_setting | odr)
                
        # set LPF & HPF
        ctl_data = self.read_reg(QMI8658Register_Ctrl5, 1)[0]
        #ctl_data = self.read_reg(QMI8658Register_Ctrl5, 1)
        ctl_data &= 0xf0
        if lpf_enable:
            ctl_data |= 0x03 << 1
            ctl_data |= 0x01  # Enable LPF
        else:
            ctl_data &= ~0x01
        self.write_reg(QMI8658Register_Ctrl5, ctl_data)

    def configure_gyro(self, range_setting, odr):
        global gyro_lsb_div
        
        # Ensure range_setting is valid
        if range_setting == 0x00:
            gyro_lsb_div = 1024
        elif range_setting == 0x10:
            gyro_lsb_div = 512
        elif range_setting == 0x20:
            gyro_lsb_div = 256
        elif range_setting == 0x30:
            gyro_lsb_div = 128
        elif range_setting == 0x40:
            gyro_lsb_div = 64
        elif range_setting == 0x50:
            gyro_lsb_div = 32
        elif range_setting == 0x60:
            gyro_lsb_div = 16
        elif range_setting == 0x70:
            gyro_lsb_div = 8
        else:
            range_setting = 64  # Default to 512dps
            gyro_lsb_div = 64

        if hpf_enable:
            self.write_reg(QMI8658Register_Ctrl3, range_setting | odr | 0x80)
        else:
            print("Ctrl3: ", bin(range_setting | odr))
            self.write_reg(QMI8658Register_Ctrl3, range_setting | odr)
                
        # set LPF & HPF
        ctl_data = self.read_reg(QMI8658Register_Ctrl5, 1)[0]
        #ctl_data = self.read_reg(QMI8658Register_Ctrl5, 1)
        ctl_data &= 0x0f
        if lpf_enable:
            ctl_data |= 0x03 << 5
            ctl_data |= 0x10  # Enable LPF
        else:
            ctl_data &= ~0x10
        self.write_reg(QMI8658Register_Ctrl5, ctl_data)
        print("Ctrl5: ",bin(ctl_data))
        
    def configure_vel(self, odr):
        print("Ctrl6: ", bin(0x80 | odr))
        self.write_reg(QMI8658Register_Ctrl6, (0x80 | odr))

    def read_acc(self):
        # Read 6 bytes of accelerometer data
        raw_data = i2c.readfrom_mem(self.address, QMI8658Register_Ax_L, 6)
        
        # Combine bytes for X, Y, and Z
        raw_acc_x = (raw_data[1] << 8) | raw_data[0]
        raw_acc_y = (raw_data[3] << 8) | raw_data[2]
        raw_acc_z = (raw_data[5] << 8) | raw_data[4]
        
        raw_acc_x = twos_complement_16bit(raw_acc_x)
        raw_acc_y = twos_complement_16bit(raw_acc_y)
        raw_acc_z = twos_complement_16bit(raw_acc_z)
        
        acc_x = (raw_acc_x * ONE_G) / acc_lsb_div
        acc_y = (raw_acc_y * ONE_G) / acc_lsb_div
        acc_z = (raw_acc_z * ONE_G) / acc_lsb_div

        #print("Acceleration:", acc_x, acc_y, acc_z)
        return acc_x, acc_y, acc_z

    def read_gyro(self):
        # Read 6 bytes of gyroscope data
        raw_data = i2c.readfrom_mem(self.address, QMI8658Register_Gx_L, 6)
        
        # Combine bytes for X, Y, and Z
        raw_gyro_x = (raw_data[1] << 8) | raw_data[0]
        raw_gyro_y = (raw_data[3] << 8) | raw_data[2]
        raw_gyro_z = (raw_data[5] << 8) | raw_data[4]

        gyro_x = (raw_gyro_x * ONE_G) / gyro_lsb_div
        gyro_y = (raw_gyro_y * ONE_G) / gyro_lsb_div
        gyro_z = (raw_gyro_z * ONE_G) / gyro_lsb_div
        #print("Gyroscope:", gyro_x, gyro_y, gyro_z)
        return gyro_x, gyro_y, gyro_z
    
    def read_vel(self):
        # Read 6 bytes of accelerometer data
        raw_data = i2c.readfrom_mem(self.address, QMI8658Register_dVX_L, 6)
        
        # Combine bytes for X, Y, and Z
        raw_v_x = (raw_data[1] << 8) | raw_data[0]
        raw_v_y = (raw_data[3] << 8) | raw_data[2]
        raw_v_z = (raw_data[5] << 8) | raw_data[4]
        
        raw_v_x = twos_complement_16bit(raw_v_x)
        raw_v_y = twos_complement_16bit(raw_v_y)
        raw_v_z = twos_complement_16bit(raw_v_z)
        
        v_x = (raw_v_x)
        v_y = (raw_v_y)
        v_z = (raw_v_z)

        #print("Acceleration:", acc_x, acc_y, acc_z)
        return v_x, v_y, v_z


# Example usage
qmi8658 = QMI8658()

# Configure accelerometer and gyroscope
qmi8658.configure_acc(0x20, 0x03)  # 8g, 1000Hz
qmi8658.configure_gyro(0x50, 0x03)  # 512dps, 940hz
qmi8658.configure_vel(0x06)  # 64hz
qmi8658.enable_sensors()
#print(qmi8658.read_reg(QMI8658Register_Ctrl7,1))
time.sleep(1)

# Read sensor data


while True:
    #print(qmi8658.read_reg(0x2f,1))
    acc = qmi8658.read_acc()
    gyro = qmi8658.read_gyro()
    vel = qmi8658.read_vel()
    #print("Acceleration:", acc)
    #print("Gyroscope:", gyro)
    print("Delta Velocity:",vel)
    time.sleep(.25)

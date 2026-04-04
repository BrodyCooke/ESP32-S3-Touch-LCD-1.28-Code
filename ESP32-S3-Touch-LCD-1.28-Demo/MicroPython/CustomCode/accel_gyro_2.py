from machine import I2C, Pin, SPI
import ujson
import time
import math
from madgwickahrs import MadgwickAHRS
import gc9a01
from bitmap import vga2_8x16 as font

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
        self.calfile = "calibration.json"
        self.calib = None
        self.gyro_cal = None
        self.mps_threshhold = .2 #tune the mps threshold, used for determining if stationary
        self.sample_rate = 1/50  # 50 Hz loop rate
        self.beta= .1 #Tune beta: smaller (e.g. 0.05) = smoother but slower(trust gyro more), larger = faster but noisier(trust accel).
        self.alpha = .995  # tweak between 0.9-0.99
        self.spin_cuttof = 1200  # deg/s threshold — tune for your disc
        self.trust_gyro_threshold = 10 #basically never trust the gyro
        self.vel = [0,0,0]
        self.pos = [0,0,0]
        self.time = 0

    def write_reg(self, reg, value):
        #print(bin(value))
        #print(bytes([value]))
        i2c.writeto_mem(self.address, reg, bytes([value]))
        #print(self.read_reg(reg,1))

    def read_reg(self, reg, length=1):
        return i2c.readfrom_mem(self.address, reg, length)
    
    def enable_sensors(self):
        self.write_reg(QMI8658Register_Ctrl1, 0x60)
        self.write_reg(QMI8658Register_Ctrl7, 0x03)
        #self.write_reg(QMI8658Register_Ctrl7, 0xB)
        #self.write_reg(QMI8658Register_Ctrl7, 0x0B)
        #print("Ctrl1: ",bin(0x60))
        #print("Ctrl7: ",bin(0x0B))
        
        
        

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
            #print("Ctrl2: ", bin(range_setting | odr | 0x80))
            self.write_reg(QMI8658Register_Ctrl2, range_setting | odr | 0x80)
        else:
            #print("Ctrl2: ", bin(range_setting | odr))
            self.write_reg(QMI8658Register_Ctrl2, range_setting | odr)
                
        # set LPF & HPF
        ctl_data = self.read_reg(QMI8658Register_Ctrl5, 1)[0]
        #ctl_data = self.read_reg(QMI8658Register_Ctrl5, 1)
        ctl_data &= 0xf0
        if lpf_enable:
            ctl_data |= 0x06
            ctl_data |= 0x01  # Enable LPF
        else:
            ctl_data &= ~0x01
            
        #print("Ctrl5: ",bin(ctl_data))    
        self.write_reg(QMI8658Register_Ctrl5, ctl_data)

    def configure_gyro(self, range_setting, odr):
        global gyro_lsb_div
        
        # Ensure range_setting is valid
        if range_setting == 0x00:
            gyro_lsb_div = 2048
        elif range_setting == 0x10:
            gyro_lsb_div = 1024
        elif range_setting == 0x20:
            gyro_lsb_div = 512
        elif range_setting == 0x30:
            gyro_lsb_div = 256
        elif range_setting == 0x40:
            gyro_lsb_div = 128
        elif range_setting == 0x50:
            gyro_lsb_div = 64
        elif range_setting == 0x60:
            gyro_lsb_div = 32
        elif range_setting == 0x70:
            gyro_lsb_div = 16
        else:
            range_setting = 64  # Default to 512dps
            gyro_lsb_div = 64

        if hpf_enable:
            #print("Ctrl3: ", bin(range_setting | odr))
            self.write_reg(QMI8658Register_Ctrl3, range_setting | odr | 0x80)
        else:
            #print("Ctrl3: ", bin(range_setting | odr))
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
        #print("Ctrl5: ",bin(ctl_data))

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
    
    
    def read_acc_g(self):
        acc = self.read_acc()
        acc_g = [x / ONE_G for x in acc]
        return(acc_g)
          
        
    def read_gyro(self):
        # Read 6 bytes of gyroscope data
        raw_data = i2c.readfrom_mem(self.address, QMI8658Register_Gx_L, 6)
        
        # Combine bytes for X, Y, and Z
        raw_gyro_x = (raw_data[1] << 8) | raw_data[0]
        raw_gyro_y = (raw_data[3] << 8) | raw_data[2]
        raw_gyro_z = (raw_data[5] << 8) | raw_data[4]
        
        raw_gyro_x = twos_complement_16bit(raw_gyro_x)
        raw_gyro_y = twos_complement_16bit(raw_gyro_y)
        raw_gyro_z = twos_complement_16bit(raw_gyro_z)
        #print("Gyroscope:", raw_gyro_x, raw_gyro_y, raw_gyro_z)

        gyro_x = raw_gyro_x / gyro_lsb_div
        gyro_y = raw_gyro_y / gyro_lsb_div
        gyro_z = raw_gyro_z / gyro_lsb_div
        #print("Gyroscope:", gyro_x, gyro_y, gyro_z)
        return gyro_x, gyro_y, gyro_z
        
        
    def init_time(self):
        self.time = time.ticks_ms()
    
    
    def update_motion(self):     
        acc_g = qmi8658.read_acc_g()
        gyro = qmi8658.read_gyro()
        acc_g = qmi8658.correct_accel(acc_g)
        gyro = qmi8658.correct_gyro(gyro)
        
        # Get current time and compute time difference
        current_time = time.ticks_ms()
        dt = (time.ticks_diff(current_time, self.time)) / 1000.0  # Convert to seconds
        self.time = current_time  # Update last time
        
        ax, ay, az = (acc_g[0],acc_g[1],acc_g[2])
        gx, gy, gz = (gyro[0],gyro[1],gyro[2])
        vx,vy,vz = self.vel
        px,py,pz = self.pos
        
        # check the magnitude of accel and adjust magwitch beta as needed
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        if abs(mag - 1.0) > self.trust_gyro_threshold:  # if not close to 1 g
            ahrs.update_beta(0.01)  # trust gyro more
            print("TRUST GYRO")
        else:
            ahrs.update_beta(self.beta)   # normal correction
        
        
        # --- Update Madgwick filter ---
        ahrs.update_imu(math.radians(gx), math.radians(gy), math.radians(gz),
                        ax, ay, az)
        q = ahrs.quaternion
        # --- Get gravity vector in body frame ---
        gx_g, gy_g, gz_g = gravity_from_quaternion(q)
        
         # --- Remove gravity from measured acceleration ---
        ax_lin_b  = (ax - gx_g) * ONE_G
        ay_lin_b = (ay - gy_g) * ONE_G
        az_lin_b = (az - gz_g) * ONE_G
        
        # 3) rotate linear accel into world frame
        # a_world = R * a_body  (we use quaternion rotate)
        ax_w, ay_w, az_w = quat_rotate_vec((ax_lin_b, ay_lin_b, az_lin_b), q)

        # 4) optional: discard accel integration while spinning too fast
        spin = math.sqrt(gx*gx + gy*gy + gz*gz)
        if spin > self.spin_cuttof:  # deg/s threshold — tune for your disc
            # skip or attenuate integration for this sample
            pass
        else:
            # integrate to velocity (world frame)
            vx += ax_w * dt
            vy += ay_w * dt
            vz += az_w * dt
    
        # 5) leakage and ZUPT (stationary detection) to limit drift
        leak = self.alpha
        vx *= leak; vy *= leak; vz *= leak

        
        # --- Zero-velocity detection ---
        if (abs(ax_w) < self.mps_threshhold and abs(ay_w) < self.mps_threshhold and abs(az_w) < self.mps_threshhold):
            vx = vy = vz = 0
        


        # Integrate velocity to get position (s = s0 + vt)
        px += vx * dt
        py += vy * dt
        pz += vz * dt
        

        self.vel = [vx, vy, vz]
        self.pos = [px, py, pz]
    

    def measure_axis(self, axis_name):
        print("\nPlace the IMU so that the", axis_name, "axis points UP (+1g).")
        input("Press Enter when ready...")
        time.sleep(2)
        a_up = sum(self.read_acc_g()[{"x": 0, "y": 1, "z": 2}[axis_name]] for _ in range(100)) / 100

        print("Now flip it so that the", axis_name, "axis points DOWN (-1g).")
        input("Press Enter when ready...")
        time.sleep(2)
        a_down = sum(self.read_acc_g()[{"x": 0, "y": 1, "z": 2}[axis_name]] for _ in range(100)) / 100

        print(f"{axis_name.upper()} +1g: {a_up:.4f}, -1g: {a_down:.4f}")
        return a_up, a_down


    def calibrate_accel(self):
        results = {}
        for axis in ("x", "y", "z"):
            a_up, a_down = self.measure_axis(axis)
            scale = 2.0 / (a_up - a_down)
            bias = (a_up + a_down) / 2.0
            results[axis] = (bias, scale)
            print(f"{axis.upper()} -> bias={bias:.5f}, scale={scale:.5f}")
        self.calib = results
        self.save_cal()



    def correct_accel(self, acc):
        ax_corr = (acc[0] - self.calib["x"][0]) * self.calib["x"][1]
        ay_corr = (acc[1] - self.calib["y"][0]) * self.calib["y"][1]
        az_corr = (acc[2] - self.calib["z"][0]) * self.calib["z"][1]
        return ax_corr, ay_corr, az_corr


    def calibrate_gyro(self, samples=500, delay=0.002):
        print("\n🧭 Gyro calibration starting...")
        print("Keep the IMU completely still until done!")
        gx_off = gy_off = gz_off = 0.0
        
        for i in range(samples):
            gx, gy, gz = self.read_gyro()
            gx_off += gx
            gy_off += gy
            gz_off += gz
            time.sleep(delay)
            if i % 100 == 0:
                print(f"Progress: {i}/{samples}")
        gx_off /= samples
        gy_off /= samples
        gz_off /= samples
        
        print("\n✅ Gyro calibration complete.")
        print(f"Offsets (deg/s): X={gx_off:.3f}, Y={gy_off:.3f}, Z={gz_off:.3f}")
        self.gyro_cal = (gx_off, gy_off, gz_off)


    def correct_gyro(self, gyro):
        gx, gy, gz = gyro
        
        gx -= self.gyro_cal[0]
        gy -= self.gyro_cal[1]
        gz -= self.gyro_cal[2]
        return(gx,gy,gz)


    def calibrate(self):
        self.calibrate_gyro()
        if not self.cal_exists():
            self.calibrate_accel()
            print("\nCalibration complete!")
        else:
            self.load_cal()
        print("\nCalibration:!")
        print(self.calib)


    def save_cal(self):
        filename=self.calfile
        try:
            with open(filename, "w") as f:
                ujson.dump(self.calib, f)
            print("✅ Cal saved to", filename)
        except Exception as e:
            print("❌ Error saving Cal:", e)
            

    def load_cal(self):
        filename=self.calfile
        try:
            with open(filename, "r") as f:
                data = ujson.load(f)
            print("✅ Cal loaded from", filename)
            self.calib = data
        except Exception as e:
            print("❌ Error loading Cal:", e)
            self.calib = None


    def cal_exists(self):
        filename=self.calfile
        try:
            os.stat(filename)
            return True
        except OSError:
            return False
        

def gravity_from_quaternion(q):
    q0, q1, q2, q3 = q
    gx = 2 * (q1*q3 - q0*q2)
    gy = 2 * (q0*q1 + q2*q3)
    gz = q0*q0 - q1*q1 - q2*q2 + q3*q3
    return gx, gy, gz
    
    
def quat_rotate_vec(v, q):
    # Using vector math: v' = v + 2*cross(q_vec, cross(q_vec, v) + q0*v)
    q0, q1, q2, q3 = q
    qv = (q1, q2, q3)
    # cross(qv, v)
    cx = qv[1]*v[2] - qv[2]*v[1]
    cy = qv[2]*v[0] - qv[0]*v[2]
    cz = qv[0]*v[1] - qv[1]*v[0]
    # cross(qv, v) + q0 * v
    tx = cx + q0 * v[0]
    ty = cy + q0 * v[1]
    tz = cz + q0 * v[2]
    # cross(qv, tx)
    rx = qv[1]*tz - qv[2]*ty
    ry = qv[2]*tx - qv[0]*tz
    rz = qv[0]*ty - qv[1]*tx
    # v' = v + 2 * (rx, ry, rz)
    return (v[0] + 2*rx, v[1] + 2*ry, v[2] + 2*rz)
if __name__ == "__main__":
    
    #init the display
    tft = gc9a01.GC9A01(
        SPI(2, baudrate=80000000,polarity=0, sck=Pin(10), mosi=Pin(11)),
        240,
        240,
        reset=Pin(14, Pin.OUT),
        cs=Pin(9, Pin.OUT),
        dc=Pin(8, Pin.OUT),
        backlight=Pin(2, Pin.OUT),
        rotation=0,
        buffer_size=16*32*2)

    tft.init()
    tft.fill(gc9a01.BLACK)
    time.sleep(1)
    tft.rotation(0)
    col_max = tft.width() - font.WIDTH*6
    row_max = tft.height() - font.HEIGHT
    tft.fill(0)
    tft.text(font,"Keep Stationary!",int(col_max/3),int(row_max/3))
    tft.text(font,"Cal in progress",int(col_max/3),int(row_max/3)*2)
    time.sleep(1)
    
        
    # Example usage
    qmi8658 = QMI8658()

    # Configure accelerometer and gyroscope
    qmi8658.configure_acc(0x20, 0x03)  # 4g, 1000Hz
    qmi8658.configure_gyro(0x50, 0x03) # 32 dps, 7174.4Hz
    qmi8658.enable_sensors()
    qmi8658.calibrate()
    tft.fill(0)
    tft.text(font,"Calibration Complete",int(col_max/4),int(row_max/2))
    #print(bin(qmi8658.read_reg(0x0,1)[0]))
    #print(bin(qmi8658.read_reg(0x1,1)[0]))
    #print("1: ",bin(qmi8658.read_reg(QMI8658Register_Ctrl1,1)[0]))
    #print("2: ",bin(qmi8658.read_reg(QMI8658Register_Ctrl2,1)[0]))
    #print("3: ",bin(qmi8658.read_reg(QMI8658Register_Ctrl3,1)[0]))
    #print("5: ",bin(qmi8658.read_reg(QMI8658Register_Ctrl5,1)[0]))
    #print("6: ",bin(qmi8658.read_reg(QMI8658Register_Ctrl6,1)[0]))
    #print("7: ",bin(qmi8658.read_reg(QMI8658Register_Ctrl7,1)[0]))
    time.sleep(1)

    sample_period = qmi8658.sample_rate  # 50 Hz loop rate
    beta_val= qmi8658.beta #Tune beta: smaller (e.g. 0.05) = smoother but slower, larger = faster but noisier.
    ahrs = MadgwickAHRS(sampleperiod=sample_period, beta=beta_val)

    counter = 0
    qmi8658.init_time()

    while True:
        '''
        acc_g = qmi8658.read_acc_g()
        acc_g = qmi8658.correct_accel(acc_g)
        gyro = qmi8658.read_gyro()
        gyro = qmi8658.correct_gyro(gyro)
        acc = [x * ONE_G for x in acc_g]
        print(acc)
        time.sleep(.25)
        '''
        qmi8658.update_motion()
        vel = qmi8658.vel
        pos = qmi8658.pos
        if (counter % 10) == 0:
            #print("Velocity:",round(vel[0],1),round(vel[1],1),round(vel[2],1))
            print("Possition:",round(pos[0],1),round(pos[1],1),round(pos[2],1))
            tft.fill(0)
            tft.text(font,f'POS X:{round(pos[0],3)}',int(col_max/3),int(row_max/5)*2)
            tft.text(font,f'POS Y:{round(pos[1],3)}',int(col_max/3),int(row_max/5)*3)
            tft.text(font,f'POS Z:{round(pos[1],3)}',int(col_max/3),int(row_max/5)*4)
        counter=counter+1
        time.sleep(sample_period)
        




# madgwickahrs.py
import math

class MadgwickAHRS:
    def __init__(self, sampleperiod=1/256, beta=0.1):
        self.sampleperiod = sampleperiod
        self.beta = beta
        self.quaternion = [1, 0, 0, 0]
    
    def update_beta(self,beta):
        self.beta = beta

    def update_imu(self, gx, gy, gz, ax, ay, az):
        q1, q2, q3, q4 = self.quaternion

        # Normalize accelerometer
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        ax, ay, az = ax / norm, ay / norm, az / norm

        # Gradient descent algorithm corrective step
        f1 = 2*(q2*q4 - q1*q3) - ax
        f2 = 2*(q1*q2 + q3*q4) - ay
        f3 = 2*(0.5 - q2*q2 - q3*q3) - az
        J_11or24 = 2 * q3
        J_12or23 = 2 * q4
        J_13or22 = 2 * q1
        J_14or21 = 2 * q2
        J_32 = 2 * J_14or21
        J_33 = 2 * J_11or24
        grad = [J_14or21*f2 - J_11or24*f1,
                J_12or23*f1 + J_13or22*f2 - J_32*f3,
                J_12or23*f2 - J_33*f3 - J_13or22*f1,
                J_14or21*f1 + J_11or24*f2]
        norm = math.sqrt(sum(g*g for g in grad))
        grad = [g / norm for g in grad]

        # Integrate rate of change of quaternion
        qDot1 = 0.5 * (-q2*gx - q3*gy - q4*gz) - self.beta * grad[0]
        qDot2 = 0.5 * (q1*gx + q3*gz - q4*gy) - self.beta * grad[1]
        qDot3 = 0.5 * (q1*gy - q2*gz + q4*gx) - self.beta * grad[2]
        qDot4 = 0.5 * (q1*gz + q2*gy - q3*gx) - self.beta * grad[3]

        q1 += qDot1 * self.sampleperiod
        q2 += qDot2 * self.sampleperiod
        q3 += qDot3 * self.sampleperiod
        q4 += qDot4 * self.sampleperiod
        norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        self.quaternion = [q1/norm, q2/norm, q3/norm, q4/norm]

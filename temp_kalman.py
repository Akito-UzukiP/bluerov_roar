import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, dt):
        # 状态向量: [位置x, 位置y, 位置z, 速度x, 速度y, 速度z, 角度yaw, 角速度]
        self.x = np.zeros((8, 1))  # 初始状态
        self.P = np.eye(8) * 0.1  # 初始协方差矩阵
        self.dt = dt  # 时间步长
        
        # 状态转移矩阵
        self.F = np.eye(8)
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt
        
        # 过程噪声协方差矩阵
        self.Q = np.eye(8) * 0.01
        
        # 观测矩阵（线性化）
        self.H = np.zeros((6, 8))
        self.H[0, 3] = 1  # x加速度观测与速度的关系
        self.H[1, 4] = 1  # y加速度观测与速度的关系
        self.H[2, 5] = 1  # z加速度观测与速度的关系
        self.H[3, 6] = 1  # 磁力计和角度yaw的关系
        self.H[4, 7] = 1  # 陀螺仪和角速度的关系
        self.H[5, 6] = 1  # 陀螺仪和角速度的关系（假设简单情况）

        # 观测噪声协方差矩阵
        self.R = np.eye(6) * 0.05

    def predict(self):
        # 预测步骤
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        # 计算卡尔曼增益
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # 更新状态向量和协方差矩阵
        y = z - np.dot(self.H, self.x)  # 残差
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))

def main():
    dt = 0.01  # 假设IMU数据每10ms更新一次
    ekf = ExtendedKalmanFilter(dt)

    # 假设输入是IMU传感器的数据
    imu_data = {
        'xacc': -2,
        'yacc': -3,
        'zacc': -999,
        'xgyro': -4,
        'ygyro': 0,
        'zgyro': -4,
        'xmag': 95,
        'ymag': 0,
        'zmag': 517,
        'temperature': 6051
    }

    # 将IMU数据转换为测量向量z
    z = np.array([
        imu_data['xacc'],
        imu_data['yacc'],
        imu_data['zacc'],
        imu_data['xmag'],
        imu_data['zgyro'],
        imu_data['ygyro']
    ]).reshape(-1, 1)

    # 运行EKF
    ekf.predict()
    ekf.update(z)

    # 输出更新后的状态
    print("Updated State:")
    print(ekf.x)

if __name__ == "__main__":
    main()

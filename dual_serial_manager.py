# dual_serial_manager.py
from imu9_serial_manager import SerialManager

class DualSerialManager:
    def __init__(self, port_a, setup_a, port_b, setup_b):
        """
        :param port_a: MCU-A 的 COM Port (如 'COM3')
        :param setup_a: MCU-A 的 IMU 配置 (如 [6, 6, 6, 6, 6, 6, 6, 6])
        :param port_b: MCU-B 的 COM Port (如 'COM4')
        :param setup_b: MCU-B 的 IMU 配置 (如 [6, 6, 6, 9, 6, 6, 6])
        """
        self.manager_a = SerialManager(port=port_a, imu_setup=setup_a)
        self.manager_b = SerialManager(port=port_b, imu_setup=setup_b)
        
        # 讓視覺化層知道總共有幾顆，維持原本的邏輯
        self.total_imus = len(setup_a) + len(setup_b)
        # 用於主程式判斷 packet size (這裡取兩者之和或主要的一個即可)
        self.total_bytes = self.manager_a.total_bytes + self.manager_b.total_bytes
        self.ser = None # 為了相容原本 visualizer 的 check

    def connect(self):
        success_a = self.manager_a.connect()
        success_b = self.manager_b.connect()
        # 為了讓 visualizer 的 if manager.ser 判斷通過
        self.ser = self.manager_a.ser 
        return success_a and success_b

    def read_data(self):
        data_a = self.manager_a.read_data()
        data_b = self.manager_b.read_data()

        # 必須兩邊都有數據才回傳，確保同步
        if data_a and data_b:
            return data_a + data_b
        return None

    def close(self):
        self.manager_a.close()
        self.manager_b.close()
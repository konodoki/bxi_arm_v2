import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from scipy import signal as sig
from collections import deque

class MultiChannelLowPassFilter:
    """多通道低通滤波器 - 同时处理多个传感器信号"""
    def __init__(self, num_channels=4, alpha=0.3):
        """
        初始化多通道滤波器
        Args:
            num_channels: 通道数量（传感器数量）
            alpha: 滤波系数 (0 < alpha < 1)
        """
        self.num_channels = num_channels
        self.alpha = alpha
        self.last_values = np.zeros(num_channels)  # 每个通道的上一个值
        self.is_initialized = False
        
    def filter(self, sensor_readings):
        """
        同时处理多个传感器的读数
        Args:
            sensor_readings: 包含多个传感器读数的列表或数组，如 [x1, x2, x3, x4]
        Returns:
            滤波后的传感器读数数组
        """
        # 转换为numpy数组
        readings = np.array(sensor_readings)
        
        if not self.is_initialized:
            # 第一次调用，用当前值初始化
            self.last_values = readings.copy()
            self.is_initialized = True
            return readings
        
        # 对每个通道独立进行一阶低通滤波
        filtered_readings = self.alpha * readings + (1 - self.alpha) * self.last_values
        
        # 更新状态
        self.last_values = filtered_readings.copy()
        
        return filtered_readings
    
    def reset(self):
        """重置所有通道的滤波器状态"""
        self.last_values = np.zeros(self.num_channels)
        self.is_initialized = False


class MultiChannelMovingAverage:
    """多通道移动平均滤波器"""
    def __init__(self, num_channels=4, window_size=5):
        """
        初始化多通道移动平均滤波器
        Args:
            num_channels: 通道数量
            window_size: 滑动窗口大小
        """
        self.num_channels = num_channels
        self.window_size = window_size
        self.buffers = []
        
        # 为每个通道创建独立的缓冲区
        for _ in range(num_channels):
            self.buffers.append([])
    
    def filter(self, sensor_readings):
        """
        同时处理多个传感器的读数
        Args:
            sensor_readings: 包含多个传感器读数的列表或数组
        Returns:
            滤波后的传感器读数数组
        """
        filtered = []
        
        for i, reading in enumerate(sensor_readings):
            # 将新读数添加到对应通道的缓冲区
            self.buffers[i].append(reading)
            
            # 保持缓冲区大小
            if len(self.buffers[i]) > self.window_size:
                self.buffers[i].pop(0)
            
            # 计算该通道的平均值
            channel_avg = np.mean(self.buffers[i])
            filtered.append(channel_avg)
        
        return np.array(filtered)
    
    def reset(self):
        """重置所有通道的缓冲区"""
        self.buffers = []
        for _ in range(self.num_channels):
            self.buffers.append([])
class MultiChannelFIRFilter:
    """多通道FIR滤波器 - 同时处理多个传感器信号"""
    def __init__(self, num_channels=4, cutoff_freq=10, sampling_freq=100, 
                filter_order=20, window='hamming'):
        """
        初始化多通道FIR滤波器
        
        Args:
            num_channels: 通道数量（传感器数量）
            cutoff_freq: 截止频率（Hz）
            sampling_freq: 采样频率（Hz）
            filter_order: 滤波器阶数
            window: 窗函数类型 ['hamming', 'hanning', 'blackman', 'kaiser', 'bartlett']
        """
        self.num_channels = num_channels
        self.cutoff_freq = cutoff_freq
        self.sampling_freq = sampling_freq
        self.filter_order = filter_order
        self.window = window
        
        # 设计FIR滤波器系数
        self.taps = self._design_fir_filter()
        
        # 为每个通道创建缓冲区 - 长度改为与抽头数相同
        self.buffers = []
        for _ in range(num_channels):
            self.buffers.append(np.zeros(len(self.taps)))  # 修改这里
        
        # 缓冲区索引
        self.buffer_index = 0

    def _design_fir_filter(self):
        """设计FIR滤波器系数"""
        nyquist = self.sampling_freq / 2
        normalized_cutoff = self.cutoff_freq / nyquist
        
        # 使用firwin设计线性相位FIR滤波器
        taps = sig.firwin(
            numtaps=self.filter_order,  # 修改这里：去掉+1，让numtaps=filter_order
            cutoff=normalized_cutoff,
            window=self.window,
            pass_zero=True  # 低通滤波器
        )
        
        return taps
    def filter(self, sensor_readings):
        """
        同时处理多个传感器的读数
        Args:
            sensor_readings: 包含多个传感器读数的列表或数组，如 [x1, x2, x3, x4]
        Returns:
            滤波后的传感器读数数组
        """
        filtered = []
        
        for i in range(self.num_channels):
            # 获取当前通道的读数
            current_value = sensor_readings[i]
            
            # 更新缓冲区：将新值添加到缓冲区的当前位置
            self.buffers[i][self.buffer_index] = current_value
            
            # 计算卷积（FIR滤波）
            # 由于缓冲区是循环的，我们需要旋转缓冲区使得最新值在正确位置
            rotated_buffer = np.roll(self.buffers[i], -self.buffer_index)
            
            # 确保缓冲区长度与抽头数匹配（修复的关键行）
            # 取旋转后缓冲区的前len(self.taps)个元素
            buffer_for_conv = rotated_buffer[:len(self.taps)]
            
            filtered_value = np.dot(buffer_for_conv, self.taps)
            
            filtered.append(filtered_value)
        
        # 更新缓冲区索引（循环缓冲区）
        self.buffer_index = (self.buffer_index + 1) % self.filter_order
        
        return np.array(filtered)
    
    def reset(self):
        """重置所有通道的滤波器状态"""
        self.buffers = []
        for _ in range(self.num_channels):
            self.buffers.append(np.zeros(self.filter_order))
        self.buffer_index = 0
    
    def get_filter_coefficients(self):
        """获取滤波器系数"""
        return self.taps.copy()
    
    def get_frequency_response(self, n_points=512):
        """获取频率响应"""
        w, h = sig.freqz(self.taps, worN=n_points)
        frequencies = w * self.sampling_freq / (2 * np.pi)  # 转换为Hz
        magnitude = 20 * np.log10(np.abs(h))  # dB
        phase = np.angle(h)  # 弧度
        
        return frequencies, magnitude, phase
class RealTimeFIRFilter:
    """实时FIR滤波器 - 优化计算效率"""
    def __init__(self, num_channels=4, coefficients=None):
        """
        初始化实时FIR滤波器
        Args:
            num_channels: 通道数量
            coefficients: FIR滤波器系数，如果不提供则使用默认系数
        """
        self.num_channels = num_channels
        
        if coefficients is None:
            # 默认使用5点移动平均
            self.taps = np.ones(5) / 5
        else:
            self.taps = np.array(coefficients)
        
        self.filter_order = len(self.taps)
        
        # 使用环形缓冲区提高效率
        self.buffers = np.zeros((num_channels, self.filter_order))
        self.write_index = 0
        
        # 预计算卷积的索引
        self.conv_indices = np.arange(self.filter_order)
    
    def filter(self, sensor_readings):
        """
        高效处理传感器读数
        Args:
            sensor_readings: 传感器读数数组
        Returns:
            滤波后的数组
        """
        filtered = np.zeros(self.num_channels)
        
        for i in range(self.num_channels):
            # 写入新数据到环形缓冲区
            self.buffers[i, self.write_index] = sensor_readings[i]
            
            # 计算卷积
            # 构建当前卷积窗口（从最新数据开始向后取）
            read_indices = (self.write_index - self.conv_indices) % self.filter_order
            conv_window = self.buffers[i, read_indices]
            
            # 计算点积（FIR滤波）
            filtered[i] = np.dot(conv_window, self.taps)
        
        # 更新写入索引
        self.write_index = (self.write_index + 1) % self.filter_order
        
        return filtered
    
    def reset(self):
        """重置滤波器"""
        self.buffers.fill(0)
        self.write_index = 0
class SimpleFIRFilter:
    """简化的FIR滤波器 - 使用移动窗口实现"""
    def __init__(self, num_channels=4, window_size=5):
        """
        初始化简化FIR滤波器
        Args:
            num_channels: 通道数量
            window_size: 窗口大小（等于滤波器阶数）
        """
        self.num_channels = num_channels
        self.window_size = window_size
        
        # 简单的平均滤波器（矩形窗）
        self.coefficients = np.ones(window_size) / window_size
        
        # 为每个通道创建缓冲区
        self.buffers = []
        for _ in range(num_channels):
            self.buffers.append([])
    
    def filter(self, sensor_readings):
        """
        处理传感器读数
        Args:
            sensor_readings: 传感器读数数组
        Returns:
            滤波后的数组
        """
        filtered = []
        
        for i in range(self.num_channels):
            # 添加新数据
            self.buffers[i].append(sensor_readings[i])
            
            # 保持窗口大小
            if len(self.buffers[i]) > self.window_size:
                self.buffers[i].pop(0)
            
            # 如果数据不足，使用部分数据
            if len(self.buffers[i]) < self.window_size:
                # 使用可用数据的平均值
                current_coeffs = np.ones(len(self.buffers[i])) / len(self.buffers[i])
                filtered_value = np.dot(self.buffers[i], current_coeffs)
            else:
                # 使用完整的FIR滤波
                filtered_value = np.dot(self.buffers[i], self.coefficients)
            
            filtered.append(filtered_value)
        
        return np.array(filtered)
    
    def set_coefficients(self, coefficients):
        """设置自定义滤波器系数"""
        if len(coefficients) != self.window_size:
            raise ValueError(f"系数长度必须为 {self.window_size}")
        self.coefficients = np.array(coefficients)
    
    def reset(self):
        """重置所有通道"""
        self.buffers = []
        for _ in range(self.num_channels):
            self.buffers.append([])
if __name__ == "__main__":
    plt.rcParams['font.family'] = 'SimHei'
    plt.rcParams['axes.unicode_minus'] = False
    # # 初始化4通道滤波器
    # lp_filter = MultiChannelLowPassFilter(num_channels=4, alpha=0.2)
    # ma_filter = MultiChannelMovingAverage(num_channels=4, window_size=5)

    # # 模拟获取4个传感器数据
    # sensor_readings = [1.2, 3.4, 0.8, 2.1]  # [sensor1, sensor2, sensor3, sensor4]

    # # 直接滤波 - 这就是您想要的方式！
    # filtered_lp = lp_filter.filter(sensor_readings)
    # filtered_ma = ma_filter.filter(sensor_readings)

    # print(f"原始传感器读数: {sensor_readings}")
    # print(f"低通滤波后: {filtered_lp}")
    # print(f"移动平均后: {filtered_ma}")

    # # 连续处理示例
    # for i in range(10):
    #     # 模拟获取传感器数据（带一些噪声）
    #     sensor1 = np.sin(i * 0.1) + np.random.normal(0, 0.1)
    #     sensor2 = np.cos(i * 0.1) + np.random.normal(0, 0.1)
    #     sensor3 = 0.5 * np.sin(i * 0.2) + np.random.normal(0, 0.1)
    #     sensor4 = 0.3 * np.cos(i * 0.3) + np.random.normal(0, 0.1)
        
    #     signals = [sensor1, sensor2, sensor3, sensor4]
        
    #     # 一次调用处理所有通道
    #     filtered_signals = lp_filter.filter(signals)
        
    #     print(f"Step {i}:")
    #     print(f"  原始: {[f'{x:.3f}' for x in signals]}")
    #     print(f"  滤波: {[f'{x:.3f}' for x in filtered_signals]}")
    #     print()
    # 创建FIR滤波器
    fir_filter = MultiChannelFIRFilter(
        num_channels=4,
        cutoff_freq=10,      # 截止频率 10Hz
        sampling_freq=100,   # 采样频率 100Hz
        filter_order=20,     # 滤波器阶数
        window='hamming'     # 汉明窗
    )

    # 简化版本
    simple_fir = SimpleFIRFilter(num_channels=4, window_size=5)

    # 实时版本（自定义系数）
    custom_coeffs = [0.1, 0.15, 0.5, 0.15, 0.1]  # 高斯型系数
    realtime_fir = RealTimeFIRFilter(num_channels=4, coefficients=custom_coeffs)

    # 模拟4个传感器的数据
    np.random.seed(42)
    num_samples = 100
    sensor_data = []

    # 生成4个不同频率的信号
    frequencies = [2, 5, 10, 20]  # Hz
    for i in range(4):
        t = np.arange(num_samples) / 100  # 100Hz采样
        signal = np.sin(2 * np.pi * frequencies[i] * t)
        noise = 0.3 * np.random.randn(num_samples)
        sensor_data.append(signal + noise)

    # 转置：每行是一个时间点，每列是一个传感器
    sensor_data = np.array(sensor_data).T

    # 滤波处理
    filtered_data_fir = []
    filtered_data_simple = []
    filtered_data_realtime = []

    for sample in sensor_data:
        # 每次处理一个时间点的所有传感器数据
        filtered_fir = fir_filter.filter(sample)
        filtered_simple = simple_fir.filter(sample)
        filtered_realtime = realtime_fir.filter(sample)
        
        filtered_data_fir.append(filtered_fir)
        filtered_data_simple.append(filtered_simple)
        filtered_data_realtime.append(filtered_realtime)

    filtered_data_fir = np.array(filtered_data_fir)
    filtered_data_simple = np.array(filtered_data_simple)
    filtered_data_realtime = np.array(filtered_data_realtime)

    # 可视化结果
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    t = np.arange(num_samples) / 100

    for i in range(4):
        # 原始信号
        axes[i, 0].plot(t, sensor_data[:, i], 'b-', alpha=0.5, label=f'原始信号 {i+1}')
        axes[i, 0].plot(t, filtered_data_fir[:, i], 'r-', label=f'FIR滤波后')
        axes[i, 0].set_title(f'传感器 {i+1} (频率={frequencies[i]}Hz)')
        axes[i, 0].set_xlabel('时间 (s)')
        axes[i, 0].set_ylabel('幅度')
        axes[i, 0].legend()
        axes[i, 0].grid(True)
        
        # 对比不同滤波器
        axes[i, 1].plot(t, sensor_data[:, i], 'b-', alpha=0.3, label='原始')
        axes[i, 1].plot(t, filtered_data_fir[:, i], 'r-', alpha=0.7, label='汉明窗FIR')
        axes[i, 1].plot(t, filtered_data_simple[:, i], 'g--', alpha=0.7, label='移动平均')
        axes[i, 1].plot(t, filtered_data_realtime[:, i], 'm:', alpha=0.7, label='实时FIR')
        axes[i, 1].set_title(f'传感器 {i+1} - 滤波器对比')
        axes[i, 1].set_xlabel('时间 (s)')
        axes[i, 1].set_ylabel('幅度')
        axes[i, 1].legend()
        axes[i, 1].grid(True)

    plt.tight_layout()
    plt.show()

    # 获取滤波器系数和频率响应
    coefficients = fir_filter.get_filter_coefficients()
    print(f"FIR滤波器系数 (长度={len(coefficients)}):")
    print(coefficients)

    # 频率响应
    freq, mag, phase = fir_filter.get_frequency_response()

    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.plot(freq, mag, 'b-')
    plt.axvline(x=10, color='r', linestyle='--', alpha=0.5, label='截止频率=10Hz')
    plt.axhline(y=-3, color='g', linestyle=':', alpha=0.5, label='-3dB')
    plt.title('FIR滤波器频率响应')
    plt.xlabel('频率 (Hz)')
    plt.ylabel('幅度 (dB)')
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.stem(np.arange(len(coefficients)), coefficients, basefmt=" ")
    plt.title('FIR滤波器系数（脉冲响应）')
    plt.xlabel('抽头索引')
    plt.ylabel('系数值')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # 在您的主循环中使用
    print("\n使用示例:")
    filter_obj = MultiChannelFIRFilter(num_channels=4, cutoff_freq=10, sampling_freq=100)

    # 模拟主循环
    for step in range(5):
        # 获取传感器数据（在真实系统中替换为实际传感器读取）
        signals = [
            1.0 + 0.1 * np.random.randn(),  # sensor1
            2.0 + 0.1 * np.random.randn(),  # sensor2
            3.0 + 0.1 * np.random.randn(),  # sensor3
            4.0 + 0.1 * np.random.randn()   # sensor4
        ]
        
        # 一次滤波所有通道
        filtered = filter_obj.filter(signals)
        
        print(f"Step {step}:")
        print(f"  原始: {[f'{x:.3f}' for x in signals]}")
        print(f"  滤波: {[f'{x:.3f}' for x in filtered]}")
        print()
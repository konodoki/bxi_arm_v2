import os
import time
import numpy as np
from scipy import linalg
from scipy.optimize import leastsq
def fit_sphere(points):
    """
    使用最小二乘法拟合球
    :param points: numpy 数组，形状 (N, 3)
    :return: (x0, y0, z0, R) 球心和半径
    """
    if not isinstance(points, np.ndarray):
        raise TypeError("points 必须是 numpy.ndarray 类型")
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points 必须是形状 (N, 3) 的二维数组")
    if points.shape[0] < 4:
        raise ValueError("至少需要 4 个非共面的点才能拟合球")

    # 初始猜测：球心为点的平均值，半径为平均距离
    center_init = points.mean(axis=0)
    radius_init = np.mean(np.linalg.norm(points - center_init, axis=1))
    params_init = np.hstack((center_init, radius_init))

    # 误差函数：点到球面的距离差
    def residuals(params, pts):
        x0, y0, z0, r = params
        distances = np.linalg.norm(pts - np.array([x0, y0, z0]), axis=1)
        return distances - r

    # 最小二乘拟合
    result, cov_x, infodict, mesg, ier = leastsq(
        residuals, params_init, args=(points,), full_output=True
    )

    if ier not in [1, 2, 3, 4]:
        raise RuntimeError(f"拟合失败: {mesg}")

    return tuple(result)  # (x0, y0, z0, R)

def fit_pico_method2(left_pico,right_pico,head_pico,CONTROLOR_ARM_LENGTH,pico_thread_event):
    sample_frq = 100 #pico更新频率假设能达到100hz
    left_sample_point = []
    right_sample_point = []
    print("将双手举过头顶挥舞")
    #等待双手举过头顶
    while left_pico.get_pos()[2]-0.4 < head_pico.get_pos()[2] or  right_pico.get_pos()[2]-0.4 < head_pico.get_pos()[2]:
        # print(left_pico.get_pos())
        if(pico_thread_event.is_set()):
            exit()
        pass
    start_time = time.time()
    print("开始读取")
    while time.time() - start_time < 10:
        if(pico_thread_event.is_set()):
            exit()
        left_sample_point.append([left_pico.get_pos()[0] - head_pico.get_pos()[0],left_pico.get_pos()[1] - head_pico.get_pos()[1],left_pico.get_pos()[2] - head_pico.get_pos()[2]])
        right_sample_point.append([right_pico.get_pos()[0] - head_pico.get_pos()[0],right_pico.get_pos()[1] - head_pico.get_pos()[1],right_pico.get_pos()[2] - head_pico.get_pos()[2]])
    left_sample_point = np.array(left_sample_point)
    right_sample_point = np.array(right_sample_point)
    print(f'共采集{left_sample_point.shape}个点')
    print('开始拟合')
    x0, y0, z0, R0 = fit_sphere(left_sample_point)
    x1, y1, z1, R1 = fit_sphere(right_sample_point)
    l_k = R0 / CONTROLOR_ARM_LENGTH
    r_k = R1 / CONTROLOR_ARM_LENGTH
    print(f"左手拟合球心: ({x0:.4f}, {y0:.4f}, {z0:.4f}, 拟合半径: {R0:.4f})")
    print(f"右手拟合球心: ({x1:.4f}, {y1:.4f}, {z1:.4f}, 拟合半径: {R1:.4f})")
    #这些球心的坐标是以头盔为坐标原点的
    return [x0, y0, z0],[x1, y1, z1],l_k,r_k

def main():
    # 生成测试数据：球心 (1, 2, 3)，半径 5，加少量噪声
    np.random.seed(0)
    true_center = np.array([1.0, 2.0, 3.0])
    true_radius = 5.0
    num_points = 100

    # 随机生成球面点
    phi = np.random.uniform(0, np.pi, num_points)
    theta = np.random.uniform(0, 2 * np.pi, num_points)
    x = true_center[0] + true_radius * np.sin(phi) * np.cos(theta)
    y = true_center[1] + true_radius * np.sin(phi) * np.sin(theta)
    z = true_center[2] + true_radius * np.cos(phi)

    # 加噪声
    noise = np.random.normal(0, 0.05, size=(num_points, 3))
    points = np.column_stack((x, y, z)) + noise
    print(points)

    # 拟合
    x0, y0, z0, R = fit_sphere(points)
    print(f"拟合球心: ({x0:.4f}, {y0:.4f}, {z0:.4f})")
    print(f"拟合半径: {R:.4f}")
if __name__ == "__main__":
    main()
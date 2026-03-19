
import onnx
import ast
import numpy as np
import onnxruntime as ort

dof_num = 29

class RunMotionPolicy:
    def __init__(self, model_onnx_path: str):
        """
        Args:
            model_onnx_path: ONNX模型文件路径
            
        Usage:
            ##1.初始化模型
            self.dance_policy = RunMotionPolicy("path/to/model.onnx")
                
            ##2.推理动作
            if self.dance_policy.timestep < self.dance_policy.motionpos.shape[0]:
                self.target_dof_pos = self.dance_policy.inference_step(q, dq, quat, omega)
        """
        self.num_obs = 96
        self.num_action = dof_num
        self.model_onnx_path = model_onnx_path

        self.target_q = np.zeros(self.num_action, dtype=np.double)
        self.action = np.zeros(self.num_action, dtype=np.double)

        policy_input = np.zeros([1, self.num_obs], dtype=np.float32)
        print("policy test")
        
        self.initialize_model(model_onnx_path)
        self.action[:] = self.inference_step(policy_input)
        
        self.timeinit = 0.0     #计算初始转换矩阵计数器
        self.timestep = 0
        
        self.obs = np.zeros(self.num_obs, dtype=np.float32)

    def initialize_model(self, model_path):
        # 配置执行提供者（根据硬件选择最优后端）
        providers = [
            'CUDAExecutionProvider',  # 优先使用GPU
            'CPUExecutionProvider'    # 回退到CPU
        ] if ort.get_device() == 'GPU' else ['CPUExecutionProvider']
        
        # 启用线程优化配置
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4  # 设置计算线程数
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

        # 加载模型
        model = onnx.load(model_path)
        metadata = {}
        for prop in model.metadata_props:
            metadata[prop.key] = prop.value

        # print("metadata", metadata)
        self.joint_names = metadata["joint_names"]
        self.joint_stiffness = np.array(ast.literal_eval(metadata["joint_stiffness"]), dtype=np.float32)
        self.joint_damping = np.array(ast.literal_eval(metadata["joint_damping"]), dtype=np.float32)
        self.action_scale = np.array(ast.literal_eval(metadata["action_scale"]), dtype=np.float32)
        self.default_joint_pos = np.array(ast.literal_eval(metadata["default_joint_pos"]), dtype=np.float32)
        # exit()
        
        # 创建推理会话
        self.session = ort.InferenceSession(
            model_path,
            providers=providers,
            sess_options=options
        )
        
        # 预存输入输出信息
        self.input_info = self.session.get_inputs()[0]
        self.output_info = self.session.get_outputs()[0]
        
        # 预分配输入内存（可选，适合固定输入尺寸）
        self.input_buffer = np.zeros(
            self.input_info.shape,
            dtype=np.float32
        )

    # 循环推理部分（极速版）
    def inference_step(self, obs_data):
        # 使用预分配内存（如果适用）
        np.copyto(self.input_buffer, obs_data)  # 比直接赋值更安全
        
        # 极简推理（比原版快5-15%）
        return self.session.run(
            [self.output_info.name], 
            {self.input_info.name: self.input_buffer}
        )[0][0]  # 直接获取第一个输出的第一个样本

    
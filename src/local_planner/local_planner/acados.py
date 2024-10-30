import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, cos, sin
import time

def create_robot_model():
    model_name = "robot_mpc_model"

    # 定义状态变量
    x = SX.sym('x')
    y = SX.sym('y')
    theta = SX.sym('theta')
    states = vertcat(x, y, theta)
    
    # 控制变量
    v = SX.sym('v')
    omega = SX.sym('omega')
    controls = vertcat(v, omega)

    # 状态转移方程
    rhs = vertcat(v * cos(theta), v * sin(theta), omega)

    # 创建模型
    model = AcadosModel()
    model.f_expl_expr = rhs
    model.x = states
    model.u = controls
    model.name = model_name

    return model

def setup_mpc_problem(N, T, model, v_max, v_min, omega_max, safe_dist):
    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = N
    ocp.solver_options.tf = N * T  # 总的预测时长

    # 设置代价函数权重
    Q = np.diag([1.0, 1.0, 0.1])
    R = np.diag([0.1, 0.02])
    ocp.cost.W_e = Q * 5
    ocp.cost.W = np.block([[Q, np.zeros((3, 2))], [np.zeros((2, 3)), R]])

    # 确保选择矩阵Vx_0和Vu_0与W_0、W匹配
    ocp.cost.Vx = np.eye(3)  # 选择矩阵，3个状态变量
    ocp.cost.Vu = np.eye(2)  # 选择矩阵，2个控制输入

    # 设置初始状态
    ocp.constraints.x0 = np.array([0, 0, 0])  # 初始状态可以根据需要进行设置

    # 设置状态和控制输入的边界约束
    ocp.constraints.lbu = np.array([v_min, -omega_max])
    ocp.constraints.ubu = np.array([v_max, omega_max])
    ocp.constraints.idxbu = np.array([0, 1])

    return ocp

def add_obstacle_constraints(ocp, obs, safe_dist):
    for ob in obs:
        cx, cy, l_long_axis, l_short_axis, orientation = ob
        c = np.cos(orientation)
        s = np.sin(orientation)
        
        a = l_long_axis
        b = l_short_axis
        obs_vec = np.array([cx, cy])

        def h(x):
            center_vec = x[:2] - obs_vec
            return b * (np.sqrt((c**2 / a**2 + s**2 / b**2) * center_vec[0]**2 + (s**2 / a**2 + c**2 / b**2) *
                                center_vec[1]**2 + 2 * c * s * (1 / a**2 - 1 / b**2) * center_vec[0] * center_vec[1]) - 1) - safe_dist

        ocp.constraints.con_h_expr = h(ocp.model.x)
        ocp.constraints.lh = np.array([0.0])  # 距离大于0
        ocp.constraints.uh = np.array([np.inf])

def solve_mpc(ocp, curr_state, goal_state, init_control, N):
    # 初始化求解器
    ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
    ocp_solver.set(0, "lbx", curr_state)
    ocp_solver.set(0, "ubx", curr_state)

    # 初始化输入和状态
    optimized_obj = float("inf")
    u_res = None
    state_res = None
    for k in range(N):
        ocp_solver.set(k, "x", goal_state[k, :3])
        ocp_solver.set(k, "u", init_control[k, :])

    # 尝试求解
    start_time = time.perf_counter()
    try:
        ocp_solver.solve()
        u_res = np.array([ocp_solver.get(i, "u") for i in range(N)])
        state_res = np.array([ocp_solver.get(i, "x") for i in range(N + 1)])
        optimized_obj = ocp_solver.get_cost()
    except Exception as e:
        print("Failed to find feasible solution:", e)

    end_time = time.perf_counter()
    print("Solving time:", end_time - start_time)

    return state_res, u_res, optimized_obj

def run_mpc():
    T = 0.1  # 单步时长
    N = 20  # 预测步长
    v_max, v_min, omega_max = 1.0, -1.0, 0.5
    safe_dist = 1.5
    curr_state = np.array([0, 0, 0])
    goal_state = np.random.rand(N, 3)  # 随机生成目标轨迹
    init_control = np.zeros((N, 2))

    model = create_robot_model()
    ocp = setup_mpc_problem(N, T, model, v_max, v_min, omega_max, safe_dist)
    add_obstacle_constraints(ocp, obs=[], safe_dist=safe_dist)

    state_res, u_res, optimized_obj = solve_mpc(ocp, curr_state, goal_state, init_control, N)
    print("Optimized objective:", optimized_obj)

run_mpc()

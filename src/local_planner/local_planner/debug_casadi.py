import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

def quadratic(x, A): return ca.mtimes([x, A, x.T])

def o(curpos_, obs_pos):
    center_vec = curpos_[:2] - ca.MX([obs_pos[0], obs_pos[1]]).T
    dist = ca.sqrt(center_vec[0] ** 2 + center_vec[1] ** 2) - 1
    return dist

def cube(curpos_):
    center_vec = curpos_[:2] - ca.MX([-0.5, 0]).T
    return ca.fabs(center_vec[0]) + ca.fabs(center_vec[1])

if __name__ == "__main__":
    N = 50
    T = 0.1
    v_min = 0
    v_max = 1
    omega_max = 1
    goal_state = np.zeros([N, 3])

    for i in range(N):
        goal_state[i][0] = i/10

    init_state = np.array([0.0, 0.0, 0])
    init_control = np.array([0.1, 0.01])
    init_control_state = np.zeros([N + 1,3])
    init_control_state[0,:] = init_state
    for i in range(N):
        state_changed = np.array(T * f(init_control_state[i,:], init_control).T)
        init_control_state[i+1,:] = init_control_state[i,:] + state_changed

    time_step = np.zeros([N,1])
    for i in range(N):
        time_step[i] = i

    obs_pos = np.array([2.5, 0])

    opti = ca.Opti()
    opt_x0 = opti.parameter(3)

    opt_states = opti.variable(N + 1, 3)
    opt_controls = opti.variable(N, 2)

    v = opt_controls[:, 0]
    omega = opt_controls[:, 1]


    opti.subject_to(opt_states[0,:] == opt_x0.T)
    opti.subject_to(opti.bounded(v_min, v, v_max))
    opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

    for i in range(N):
        x_next = opt_states[i,:] + T * f(opt_states[i,:], opt_controls[i,:]).T
        opti.subject_to(opt_states[i+1,:] == x_next)

    for i in range(N):
        # opti.subject_to(cube(opt_states[i,:]) >= 0.2)
        opti.subject_to(o(opt_states[i+1,:], obs_pos) >= (0.5*o(opt_states[i,:], obs_pos)))
        # opti.subject_to(((opt_states[i,0] + 0)**2 + (opt_states[i,1] + 0)**2) >= 0.1)

    opti.set_value(opt_x0, init_state)

    obj = 0
    R = np.diag([0.1, 0.02])
    A = np.diag([0.1, 0.02])
    for i in range(1,N):
        Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
        if i < N-1:
            obj += 0.1 * quadratic(opt_states[i, :] - goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
        else:
            obj += 0.1 * quadratic(opt_states[i, :] - goal_state[[i]], Q)
    Q = np.diag([1.0,1.0, 0.02])*5
    obj += quadratic(opt_states[N-1, :] - goal_state[[N-1]], Q)

    # I = np.diag([100,100,100])
    # obj += quadratic(opt_states[0,:] - opt_x0.T, I)

    opti.minimize(obj)
    opts_setting = {'ipopt.max_iter': 10000, 'ipopt.print_level': 3, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                    'ipopt.acceptable_obj_change_tol': 1e-3, 'ipopt.acceptable_constr_viol_tol': 1e-3}
    opti.solver('ipopt', opts_setting)

    for i in range(N):
        opti.set_initial(opt_controls[i, :], init_control)

    for i in range(N+1):
        opti.set_initial(opt_states[i,:], init_control_state[i,:])

    try:
        sol = opti.solve()
        u_res = sol.value(opt_controls)
        state_predict = sol.value(opt_states)
        optimized_obj_value = opti.value(obj)
        print(optimized_obj_value)

    except Exception as e:
        print("error")
        u_res = opti.debug.value(opt_controls)
        state_predict = opti.debug.value(opt_states)


    fig, axs = plt.subplots(2, 1)

    axs[0].plot(init_control_state[:,0], init_control_state[:,1])
    axs[0].plot(state_predict[:,0], state_predict[:,1])

    # axs[0].set_xlim(-1.5, 0.5)
    axs[0].set_aspect('equal')

    axs[1].plot(time_step, u_res[:,0])
    axs[1].plot(time_step, u_res[:,1])
    plt.show()

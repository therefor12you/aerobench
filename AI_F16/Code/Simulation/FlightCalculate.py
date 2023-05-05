'''
lzp
run_f16_sim python version for simulation
'''

import time

import numpy as np
from scipy.integrate import RK45

from highlevel.controlled_f16 import controlled_f16
from util import get_state_names, Euler

class simulation_class:
    def __init__(self):
        self.times = []
        self.states = []
        self.modes = []
        self.der_func = []
        self.integrator = []

        self.xd_list = []
        self.u_list = []
        self.throttle_list = []
        self.ele_list = []
        self.ali_list = []
        self.rud_list = []
        self.Nz_list = []
        self.ps_list = []
        self.Ny_r_list = []

        

def run_f16_sim(aircrafts, tmax, step=1/30, extended_states=False, model_str='morelli',
                integrator_str='euler', v2_integrators=False):
    '''Simulates and analyzes autonomous F-16 maneuvers

    if multiple aircraft are to be simulated at the same time,
    initial_state should be the concatenated full (including integrators) initial state.

    returns a dict with the following keys:

    'status': integration status, should be 'finished' if no errors, or 'autopilot finished'
    'times': time history
    'states': state history at each time step
    'modes': mode history at each time step

    if extended_states was True, result also includes:
    'xd_list' - derivative at each time step
    'ps_list' - ps at each time step
    'Nz_list' - Nz at each time step
    'Ny_r_list' - Ny_r at each time step
    'u_list' - input at each time step, input is 7-tuple: throt, ele, ail, rud, Nz_ref, ps_ref, Ny_r_ref
    These are tuples if multiple aircraft are used
    '''

    # 仿真计时开始
    start = time.perf_counter()

    simulation_classes = [simulation_class() for _ in aircrafts]


    # 数值分析方法的选用
    if integrator_str == 'rk45':
        integrator_class = RK45
        kwargs = {}
    else:
        assert integrator_str == 'euler'
        integrator_class = Euler
        kwargs = {'step': step}
    
    # 遍历每一架飞机，确定初始仿真条件
    for aircraft, sim_class in enumerate(aircrafts, simulation_classes):

        state = np.array(aircraft.state, dtype=float)
        llc = aircraft.ap.llc

        num_vars = len(get_state_names()) + llc.get_num_integrators()

        if state.size < num_vars:
            # append integral error states to state vector
            x0 = np.zeros(num_vars)
            x0[:state.shape[0]] = state
        else:
            x0 = state

        assert x0.size % num_vars == 0, f"expected initial state ({x0.size} vars) to be multiple of {num_vars} vars"

        # run the numerical simulation
        times = [0]
        states = [x0]

        # mode can change at time 0
        aircraft.ap.advance_discrete_mode(times[-1], states[-1])

        modes = [aircraft.ap.mode]

        if extended_states:
            xd, u, Nz, ps, Ny_r = get_extended_states(aircraft.ap, times[-1], states[-1], model_str, v2_integrators)

            xd_list = [xd]

            u_list = [u]
            throttle_list = [u[0]]
            ele_list = [u[1]]
            ali_list = [u[2]]
            rud_list = [u[3]]

            Nz_list = [Nz]
            ps_list = [ps]
            Ny_r_list = [Ny_r]


        # 仿真计算设置
        der_func = make_der_func(aircraft.ap, model_str, v2_integrators)

        # note: fixed_step argument is unused by rk45, used with euler
        integrator = integrator_class(der_func, times[-1], states[-1], tmax, **kwargs)

        # 仿真状态的存储
        sim_class.time = times
        sim_class.states = states
        sim_class.modes = modes
        sim_class.der_func = der_func
        sim_class.integrator = integrator

        # 控制变量的存储
        sim_class.xd_list.append(xd_list)
        sim_class.u_list.append(u_list)
        sim_class.throttle_list.append(throttle_list)
        sim_class.ele_list.append(ele_list)
        sim_class.ali_list.append(ali_list)
        sim_class.rud_list.append(rud_list)
        sim_class.Nz_list.append(Nz_list)
        sim_class.ps_list.append(ps_list)
        sim_class.Ny_r_list.append(Ny_r_list)
        

    # 开始仿真
    while status(simulation_classes):
        trim_step(simulation_classes)

        for aircraft, sim_class in enumerate(aircrafts, simulation_classes):

            # 传感器控制--雷达扫描、目标分配、跟踪锁定
            target = nearest_enemy()    # 离本机最近的敌机作为target
            aircraft.radar.mode(aircraft, target)
            aircraft.radar.SightAngle(aircraft, target)
            aircraft.radar.scan(t)

            # 火控决策--攻击区计算、发射判定、杀伤判定
            if aircraft.radar.TarInfo is not None:
                # 计算导弹攻击区
                aircraft.weapon.missile_attack_zone(aircraft)
                # 计算激光武器攻击区
                aircraft.weapon.laser_attack_zone(aircraft)

                # 是否发射导弹
                if aircraft.weapon.missile_lauch(aircraft):
                    # 发射导弹
                    aircraft.missile.png(aircraft)
                    
                    # 杀伤判定
                    aircraft.weapon.boom(aircraft)


            if sim_class.integrator.t >= sim_class.times[-1] + step:
                dense_output = sim_class.integrator.dense_output()

                while sim_class.integrator.t >= sim_class.times[-1] + step:
                    t = sim_class.times[-1] + step
                    #print(f"{round(t, 2)} / {tmax}")

                    sim_class.times.append(t)
                    aircraft.state = dense_output(t)
                    sim_class.states.append(aircraft.state)


                    # 机动决策--路径规划
                    aircraft.path_plan()

                    updated = aircraft.ap.advance_discrete_mode(sim_class.times[-1], sim_class.states[-1])
                    sim_class.modes.append(aircraft.ap.mode)

                    # re-run dynamics function at current state to get non-state variables
                    if extended_states:
                        xd, u, Nz, ps, Ny_r = get_extended_states(aircraft.ap, sim_class.times[-1], sim_class.states[-1], model_str, v2_integrators)

                        sim_class.xd_list.append(xd)
                        sim_class.u_list.append(u)
                        sim_class.throttle_list.append(u[0])
                        sim_class.ele_list.append(u[1])
                        sim_class.ali_list.append(u[2])
                        sim_class.rud_list.append(u[3])

                        sim_class.Nz_list.append(Nz)
                        sim_class.ps_list.append(ps)
                        sim_class.Ny_r_list.append(Ny_r)

                    if aircraft.ap.is_finished(times[-1], states[-1]):
                        # this both causes the outer loop to exit and sets res['status'] appropriately
                        sim_class.integrator.status = 'autopilot finished'
                        break

                    if updated:
                        # re-initialize the integration class on discrete mode switches
                        sim_class.integrator = integrator_class(der_func, sim_class.times[-1], sim_class.states[-1], tmax, **kwargs)
                        break    
            
            # 退出战区判定--是否结束空战仿真
            
    # assert 'finished' in integrator.status

    # 仿真结束, 存储计算结果
    for aircraft, sim_class in enumerate(aircrafts, simulation_classes):

        aircraft.res['status'] = sim_class.integrator.status
        aircraft.res['times'] = sim_class.times
        aircraft.res['states'] = np.array(sim_class.states, dtype=float)
        aircraft.res['modes'] = sim_class.modes

        if extended_states:
            aircraft.res['xd_list'] = sim_class.xd_list
            aircraft.res['ps_list'] = sim_class.ps_list
            aircraft.res['Nz_list'] = sim_class.Nz_list
            aircraft.res['Ny_r_list'] = sim_class.Ny_r_list
            
            aircraft.res['u_list'] = sim_class.u_list
            aircraft.res['throttle_list'] = sim_class.throttle_list
            aircraft.res['ele_list'] = sim_class.ele_list
            aircraft.res['ali_list'] = sim_class.ali_list
            aircraft.res['rud_list'] = sim_class.rud_list

        aircraft.res['runtime'] = time.perf_counter() - start


def status(integrate_classes):
    for c in integrate_classes:
        if c.integrator.status != "running":
            return False
    return True

def trim_step(integrate_classes):
    for c in integrate_classes:
        c.integrator.step()

def nearest_enemy():

    return 
    
def make_der_func(ap, model_str, v2_integrators):
    'make the combined derivative function for integration'

    def der_func(t, full_state):
        'derivative function, generalized for multiple aircraft'

        u_refs = ap.get_checked_u_ref(t, full_state)

        num_aircraft = u_refs.size // 4
        num_vars = len(get_state_names()) + ap.llc.get_num_integrators()
        assert full_state.size // num_vars == num_aircraft

        xds = []

        for i in range(num_aircraft):
            state = full_state[num_vars*i:num_vars*(i+1)]
            u_ref = u_refs[4*i:4*(i+1)]

            xd = controlled_f16(t, state, u_ref, ap.llc, model_str, v2_integrators)[0]
            xds.append(xd)

        rv = np.hstack(xds)

        return rv
    
    return der_func

def get_extended_states(ap, t, full_state, model_str, v2_integrators):
    '''get xd, u, Nz, ps, Ny_r at the current time / state

    returns tuples if more than one aircraft
    '''

    llc = ap.llc
    num_vars = len(get_state_names()) + llc.get_num_integrators()
    num_aircraft = full_state.size // num_vars

    xd_tup = []
    u_tup = []
    Nz_tup = []
    ps_tup = []
    Ny_r_tup = []

    u_refs = ap.get_checked_u_ref(t, full_state)

    for i in range(num_aircraft):
        state = full_state[num_vars*i:num_vars*(i+1)]
        u_ref = u_refs[4*i:4*(i+1)]

        xd, u, Nz, ps, Ny_r = controlled_f16(t, state, u_ref, llc, model_str, v2_integrators)

        xd_tup.append(xd)
        u_tup.append(u)
        Nz_tup.append(Nz)
        ps_tup.append(ps)
        Ny_r_tup.append(Ny_r)

    if num_aircraft == 1:
        rv_xd = xd_tup[0]
        rv_u = u_tup[0]
        rv_Nz = Nz_tup[0]
        rv_ps = ps_tup[0]
        rv_Ny_r = Ny_r_tup[0]
    else:
        rv_xd = tuple(xd_tup)
        rv_u = tuple(u_tup)
        rv_Nz = tuple(Nz_tup)
        rv_ps = tuple(ps_tup)
        rv_Ny_r = tuple(Ny_r_tup)

    return rv_xd, rv_u, rv_Nz, rv_ps, rv_Ny_r

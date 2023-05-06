'''
Stanley Bak
run_f16_sim python version
'''

import time
import numpy as np
import copy
# from scipy.integrate import RK45

from highlevel.controlled_f16 import controlled_f16
from lowlevel.low_level_controller import LowLevelController
from util import get_state_names, Euler
import rticonnextdds_connector as rti
from os import path 

def run_f16_sim(initial_state, tmax, ap, missile, laser, battery, radar, step, extended_states=False, model_str='morelli',
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

    start = time.perf_counter()

    initial_state = np.array(initial_state, dtype=float)
    llc = ap.llc
    # print(type(llc))
    # assert isinstance(llc, LowLevelController)
    

    num_vars = len(get_state_names()) + llc.get_num_integrators()

    if initial_state.size < num_vars:
        # append integral error states to state vector
        x0 = np.zeros(num_vars)
        x0[:initial_state.shape[0]] = initial_state
    else:
        x0 = initial_state

    assert x0.size % num_vars == 0, f"expected initial state ({x0.size} vars) to be multiple of {num_vars} vars"

    # run the numerical simulation
    times = [0]
    states = [x0]

    # mode can change at time 0
    ap.advance_discrete_mode(times[-1], states[-1])

    modes = [ap.mode]
    
    if extended_states:
        xd, u, Nz, ps, Ny_r = get_extended_states(ap, times[-1], states[-1], model_str, v2_integrators)

        xd_list = [xd]

        u_list = [u]
        throttle_list = [u[0]]
        ele_list = [u[1]]
        ali_list = [u[2]]
        rud_list = [u[3]]

        Nz_list = [Nz]
        ps_list = [ps]
        Ny_r_list = [Ny_r]


    der_func = make_der_func(ap, model_str, v2_integrators)

    if integrator_str == 'rk45':
        # integrator_class = RK45
        kwargs = {}
    else:
        assert integrator_str == 'euler'
        integrator_class = Euler
        kwargs = {'step': step}

    # note: fixed_step argument is unused by rk45, used with euler
    integrator = integrator_class(der_func, times[-1], states[-1], tmax, **kwargs)

    # DDS
    file_path = path.dirname(path.realpath(__file__))
    pub_connector = rti.Connector("MyParticipantLibrary::MyPubParticipant", file_path+"\ShapeExample.xml")
    sub_connector = rti.Connector("MyParticipantLibrary::MySubParticipant", file_path+"\ShapeExample.xml")

    # list
    missile_pos_list = [copy.deepcopy(missile.position)]
    missile_speed_list = [copy.deepcopy(missile.speed)]
    battery_soc_list = [copy.deepcopy(battery.soc)]
    laser_thermal_list = [copy.deepcopy(laser.Q)]
    radar_radius_list = [copy.deepcopy(radar.radius)]
    radar_azi_list = [copy.deepcopy([copy.deepcopy(radar.azi_1), copy.deepcopy(radar.azi_2)])]

    while integrator.status == 'running':
        integrator.step()

        if integrator.t >= times[-1] + step:
            dense_output = integrator.dense_output()

            while integrator.t >= times[-1] + step:
                t = times[-1] + step
                print(f"{round(t, 2)} / {tmax}")
                times.append(t)
                states.append(dense_output(t))
                updated = ap.advance_discrete_mode(times[-1], states[-1])
                modes.append(ap.mode)

                # 发布本机信息
                output = pub_connector.get_output("MyPublisher::MyAircraftWriter")
                output.instance.set_dictionary({"state":states[-1].tolist()})
                output.write()
                print('飞机信息已发布{}'.format(states[-1].tolist()))
                # output.wait() # Wait for all subscriptions to receive the data before exiting

                # 导弹
                missile.png(step, t)
                missile_pos_list.append(copy.deepcopy(missile.position))
                missile_speed_list.append(copy.deepcopy(missile.speed))
                # print(missile_pos_list) 

                # 雷达
                radar.scan(t)
                radar_radius_list.append(copy.deepcopy(radar.radius))
                radar_azi_list.append([copy.deepcopy(radar.azi_1), copy.deepcopy(radar.azi_2)])

                # 高能装备电模型
                battery.dis_and_charge(t)
                battery_soc_list.append(copy.deepcopy(battery.soc))
                
                # 高能装备热模型
                laser.thermal(step)
                laser_thermal_list.append(copy.deepcopy(laser.Q))

                # 若被激光武器击中，将丢失跟踪能力
                if laser.launch_flag:
                    missile.be_hitted()
                

                # re-run dynamics function at current state to get non-state variables
                if extended_states:
                    xd, u, Nz, ps, Ny_r = get_extended_states(ap, times[-1], states[-1], model_str, v2_integrators)

                    xd_list.append(xd)
                    u_list.append(u)
                    throttle_list.append(u[0])
                    ele_list.append(u[1])
                    ali_list.append(u[2])
                    rud_list.append(u[3])

                    Nz_list.append(Nz)
                    ps_list.append(ps)
                    Ny_r_list.append(Ny_r)

                   
                if ap.is_finished(times[-1], states[-1]):
                    # this both causes the outer loop to exit and sets res['status'] appropriately
                    integrator.status = 'autopilot finished'
                    break

                if updated:
                    # re-initialize the integration class on discrete mode switches
                    integrator = integrator_class(der_func, times[-1], states[-1], tmax, **kwargs)
                    break

    assert 'finished' in integrator.status

    res = {}
    res['status'] = integrator.status
    res['times'] = times
    res['aircraft_states'] = np.array(states, dtype=float)
    res['modes'] = modes
    res['laser_thermal'] = laser_thermal_list
    res['missile_pos'] = missile_pos_list
    res['missile_vel'] = missile_speed_list
    res['battery_soc'] = battery_soc_list
    res['radar_radius'] = radar_radius_list
    res['radar_azi'] = radar_azi_list

    if extended_states:
        res['xd_list'] = xd_list
        res['ps_list'] = ps_list
        res['Nz_list'] = Nz_list
        res['Ny_r_list'] = Ny_r_list
        
        res['u_list'] = u_list
        res['throttle_list'] = throttle_list
        res['ele_list'] = ele_list
        res['ali_list'] = ali_list
        res['rud_list'] = rud_list


    res['runtime'] = time.perf_counter() - start

    return res

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

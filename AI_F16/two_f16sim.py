'''
Stanley Bak
run_f16_sim python version
'''

import time

import numpy as np
from scipy.integrate import RK45
import matplotlib.pyplot as plt
from aerobench.highlevel.controlled_f16 import controlled_f16
from aerobench.util import get_state_names, Euler


def Two_f16_sim(initial_state_blue, initial_state_red, tmax, ap_blue, ap_red, step=1/30, extended_states=False, model_str='morelli',
                integrator_str='rk45', v2_integrators=False):
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

    # 红蓝双方初始状态
    initial_state_blue = np.array(initial_state_blue, dtype=float)
    initial_state_red = np.array(initial_state_red, dtype=float)

    llc_blue = ap_blue.llc
    llc_red = ap_red.llc

    num_vars_blue = len(get_state_names()) + llc_blue.get_num_integrators()
    num_vars_red = len(get_state_names()) + llc_red.get_num_integrators()

    if initial_state_blue.size < num_vars_blue or initial_state_red.size < num_vars_red:
        # append integral error states to state vector
        x0_blue = np.zeros(num_vars_blue)
        x0_blue[:initial_state_blue.shape[0]] = initial_state_blue
        x0_red = np.zeros(num_vars_red)
        x0_red[:initial_state_red.shape[0]] = initial_state_red
    else:
        x0_blue = initial_state_blue
        x0_red = initial_state_red

    assert x0_blue.size % num_vars_blue == 0, f"expected initial state ({x0_blue.size} vars) to be multiple of {num_vars} vars"

    # run the numerical simulation
    times_blue = [0]
    times_red = [0]
    states_blue = [x0_blue]
    states_red = [x0_red]

    # mode can change at time 0
    updated_blue = ap_blue.advance_discrete_mode(times_blue[-1], states_blue[-1], states_red[-1])
    updated_red = ap_red.advance_discrete_mode(times_red[-1], states_red[-1], states_blue[-1])

    modes_blue = [ap_blue.mode]
    modes_red = [ap_red.mode]

    if extended_states:
        xd_blue, u_blue, Nz_blue, ps_blue, Ny_r_blue = get_extended_states(ap_blue, times_blue[-1], states_blue[-1], model_str, v2_integrators)
        xd_red, u_red, Nz_red, ps_red, Ny_r_red = get_extended_states(ap_red, times_red[-1], states_red[-1], model_str, v2_integrators)
        
        xd_list_blue = [xd_blue]
        u_list_blue = [u_blue]
        Nz_list_blue = [Nz_blue]
        ps_list_blue = [ps_blue]
        Ny_r_list_blue = [Ny_r_blue]

        xd_list_red = [xd_red]
        u_list_red = [u_red]
        Nz_list_red = [Nz_red]
        ps_list_red = [ps_red]
        Ny_r_list_red = [Ny_r_red]


    der_func_blue = make_der_func(ap_blue, model_str, v2_integrators)
    der_func_red = make_der_func(ap_red, model_str, v2_integrators)

    if integrator_str == 'rk45':
        integrator_class = RK45
        kwargs = {}
    else:
        assert integrator_str == 'euler'
        integrator_class = Euler
        kwargs = {'step': step}

    # note: fixed_step argument is unused by rk45, used with euler
    integrator_blue = integrator_class(der_func_blue, times_blue[-1], states_blue[-1], tmax, **kwargs)
    integrator_red = integrator_class(der_func_red, times_blue[-1], states_red[-1], tmax, **kwargs)

 
    while integrator_red.status == 'running' and integrator_blue.status == 'running':
        integrator_blue.step()
        integrator_red.step()

        if integrator_blue.t >= times_blue[-1] + step:
            dense_output_blue = integrator_blue.dense_output()
            
            while integrator_blue.t >= times_blue[-1] + step:
                t_blue = times_blue[-1] + step
                print(f"蓝方{t_blue}/ {tmax}, 蓝方时间：{integrator_blue.t}")

                times_blue.append(t_blue)
                states_blue.append(dense_output_blue(t_blue))

                if len(times_blue)%10 == 0:
                    updated_blue = ap_blue.advance_discrete_mode(times_blue[-1], states_blue[-1], states_red[-1])
                    updated_red = ap_red.advance_discrete_mode(times_red[-1], states_red[-1], states_blue[-1])
                modes_blue.append(ap_blue.mode)
                modes_red.append(ap_red.mode)
                # re-run dynamics function at current state to get non-state variables
                if extended_states:
                    xd_blue, u_blue, Nz_blue, ps_blue, Ny_r_blue = get_extended_states(ap_blue, times_blue[-1], states_blue[-1], model_str, v2_integrators)
                    
                    xd_list_blue.append(xd_blue)
                    u_list_blue.append(u_blue)
                    Nz_list_blue.append(Nz_blue)
                    ps_list_blue.append(ps_blue)
                    Ny_r_list_blue.append(Ny_r_blue)


                if ap_blue.is_finished(times_blue[-1], states_blue[-1]):
                    # this both causes the outer loop to exit and sets res['status'] appropriately
                    # integrator_blue.status = 'autopilot_blue finished'
                    break

                if updated_blue and updated_red:
                    # re-initialize the integration class on discrete mode switches
                    integrator_blue = integrator_class(der_func_blue, times_blue[-1], states_blue[-1], tmax, **kwargs)
                    integrator_red= integrator_class(der_func_red, times_red[-1], states_red[-1], tmax, **kwargs)
                    break

        if integrator_red.t >= times_red[-1] + step:
            dense_output_red = integrator_red.dense_output()

            while integrator_red.t >= times_red[-1] + step:
                t = times_red[-1] + step
                print(f"红方{t} / {tmax}, 红方时间：{integrator_red.t}")

                times_red.append(t)
                states_red.append(dense_output_red(t))
                
                if len(times_red)%10 == 0:
                    updated_blue = ap_blue.advance_discrete_mode(times_blue[-1], states_blue[-1], states_red[-1])
                    updated_red = ap_red.advance_discrete_mode(times_red[-1], states_red[-1], states_blue[-1])
                modes_red.append(ap_red.mode)
                modes_blue.append(ap_blue.mode)
                # re-run dynamics function at current state to get non-state variables
                if extended_states:
                    xd_red, u_red, Nz_red, ps_red, Ny_r_red= get_extended_states(ap_red, times_red[-1], states_red[-1], model_str, v2_integrators)
                    
                    xd_list_red.append(xd_red)
                    u_list_red.append(u_red)
                    Nz_list_red.append(Nz_red)
                    ps_list_red.append(ps_red)
                    Ny_r_list_red.append(Ny_r_red)

                if ap_red.is_finished(times_red[-1], states_red[-1]):
                    # this both causes the outer loop to exit and sets res['status'] appropriately
                    # integrator_red.status = 'autopilot_red finished'
                    break

                if updated_red:
                    # re-initialize the integration class on discrete mode switches
                    integrator_red= integrator_class(der_func_red, times_red[-1], states_red[-1], tmax, **kwargs)
                    break

    # assert 'finished' in integrator_blue.status
    # assert 'finished' in integrator_red.status

    res_blue = {}
    res_blue['status'] = integrator_blue.status
    res_blue['times'] = times_blue
    res_blue['states'] = np.array(states_blue, dtype=float)
    res_blue['modes'] = modes_blue
    res_red = {}
    res_red['status'] = integrator_red.status
    res_red['times'] = times_red
    res_red['states'] = np.array(states_red, dtype=float)
    res_red['modes'] = modes_red

    if extended_states:
        res_blue['xd_list'] = xd_list_blue
        res_blue['ps_list'] = ps_list_blue
        res_blue['Nz_list'] = Nz_list_blue
        res_blue['Ny_r_list'] = Ny_r_list_blue
        res_blue['u_list'] = u_list_blue
        
        res_red['xd_list'] = xd_list_red
        res_red['ps_list'] = ps_list_red
        res_red['Nz_list'] = Nz_list_red
        res_red['Ny_r_list'] = Ny_r_list_red
        res_red['u_list'] = u_list_red

    res_blue['runtime'] = time.perf_counter() - start
    res_red['runtime'] = time.perf_counter() - start

    return res_blue,res_red

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

import sys
sys.path.append('d:\\LZP_HP\\yanjiusheng\\TX\\DoD\\aerobench')

from AI_F16.Code.Test.run_f16_sim_test import run_f16_sim
from examples.waypoint.waypoint_autopilot import WaypointAutopilot
from AI_F16.Code.Simulation.Conponents.Missile import Missile
from AI_F16.Code.Simulation.Conponents.Battery import Battery
from AI_F16.Code.Simulation.Conponents.Laser import Laser
from AI_F16.Code.Simulation.Conponents.Radar import Radar
from AI_F16.Code.Test.viewer_test import ScenarioViewer

from numpy import deg2rad

def main():
    'main function'

    ### Aircraft Initial Conditions ###
    power = 9 # engine power level (0-10)
    # Default alpha & beta
    alpha = deg2rad(2.1215) # Trim Angle of Attack (rad)
    beta = 0                # Side slip angle (rad)
    # Initial Attitude
    alt = 1500        # altitude (ft)
    vt = 540          # initial velocity (ft/sec)
    phi = 0           # Roll angle from wings level (rad)
    theta = 0         # Pitch angle from nose level (rad)
    psi = 0           # Yaw angle from North (rad)
    # Build Initial Condition Vectors
    # state = [vt, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow]
    init = [vt, alpha, beta, phi, theta, psi, 0, 0, 0, 0, 0, alt, power]
    
    # make waypoint list
    waypoints = [[-5000, -7500, alt],
                 [-15000, -7500, alt],
                 [-20000, 0, alt+500]]
    ap = WaypointAutopilot(waypoints, stdout=True)


    ### Laser Initial Conditions ###
    laser = Laser()

    ### Battery Initial Conditions ###
    battery = Battery()

    ### Missile Initial Conditions ###
    missile = Missile()

    ### Radar Initial Conditions ###
    radar = Radar()

    ### Run Simulation ###
    step = 0.05
    tmax = 5
    extended_states = True
    res = run_f16_sim(init, tmax, ap, missile, laser, battery, radar, step=step, extended_states=extended_states, integrator_str='euler')
    

    ### Visualize ###
    file_names = ['D:\\LZP_HP\\yanjiusheng\\TX\\DoD\\aerobench\\AI_F16\\Code\\Test\\view_results\\flight.gif', 'D:\\LZP_HP\\yanjiusheng\\TX\\DoD\\aerobench\\AI_F16\\Code\\Test\\view_results\\key_variable.jpg']
    my_viewer = ScenarioViewer(res, file_names)
    my_viewer.summary_video()
    my_viewer.key_variable_plot()
    print(f"Simulation Completed in {round(res['runtime'], 2)} seconds (extended_states={extended_states})")

if __name__ == '__main__':
    main()
    


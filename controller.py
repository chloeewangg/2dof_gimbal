'''goals7controller.py

   This encapsulates the motor control code into a single function.
'''



#
#   Define your entire code as a function.  The argument will be the
#   shared data that is accessed by both pieces.
#
def controller(shared):
    import hebi
    from math import inf, sin, cos
    import numpy as np              
    import matplotlib.pyplot as plt

    from math import pi, sin, cos, asin, acos, atan2, sqrt
    from time import sleep, time
    from keycheck import kbhit, getch

    from enum import Enum

    names = ['6.7', '9.7']
    group = hebi.Lookup().get_group_from_names(['robotlab'], names)
    if group is None:
        print("Unable to find both motors " + str(names))
        raise Exception("Unable to connect to motors")

    command  = hebi.GroupCommand(group.size)
    feedback = hebi.GroupFeedback(group.size)

    class Traj(Enum):
        HOLD = 0 # Keeps a constant pos, zero velocity forever
        SPLINE = 1 # Computes a cubic spline, ends at tf
        SCAN = 2 # Computes sinusoidal pos/vel, never ends

    class Mode(Enum):
        GOHOME = 0 # Go to the home position (0,0)
        TRACKING = 1 # Track the primary object of interest
        SCANNING = 2 # Scan the entire field of view (w/o tracking)

    def get_time(p0, pf, v0, vf, vmax):
        calc_t = (1.5 * (abs(pf - p0)))/vmax + abs(v0)/(vmax/0.6) + abs(vf)/(vmax/0.6)

        return calc_t

    def calc_params(t0, tf, p0, pf, v0, vf, vmax):
        tmove = tf - t0
        if tmove == 0:
            c = 0
            d = 0
        else:
            c = 3*(pf - p0)*(1/tmove**2)-vf*(1/tmove)-2*v0*(1/tmove)
            d = -2*(pf - p0)*(1/tmove**3)+vf*(1/(tmove**2))+v0*(1/(tmove**2))

        return (p0, v0, c, d)

    def splinecmds(t0, t, a, b, c, d):
        pcmd = a + b * (t - t0) + c * (t - t0) ** 2 + d * (t - t0) ** 3
        vcmd = b + 2 * c * (t - t0) + 3 * d * (t - t0) ** 2

        return (pcmd, vcmd)

    dt = 0.01                       # HEBI feedback comes in at 100Hz!

    #
    #  Define the parameters
    #

    T = 20

    N = int(T / dt)                 # 100 samples/second.

    Time = [0.0] * N # creates a list of N elements where each element is 0.0
    PAct_pan = [0.0] * N
    PCmd_pan = [0.0] * N
    VAct_pan = [0.0] * N
    VCmd_pan = [0.0] * N

    PAct_tilt = [0.0] * N
    PCmd_tilt = [0.0] * N
    VAct_tilt = [0.0] * N
    VCmd_tilt = [0.0] * N

    Object_pan = [0.0] * N
    Object_tilt = [0.0] * N

    historyofobjects = []
    knownobjects = []

    #
    #  Execute the movement.
    #
    # Initialize the index and time.
    index = 0
    t     = 0.0

    # Scan parameters

    T_scan = 17.6243347866

    A_pan = 1.309
    p0_pan_scan = 0
    v0_pan_scan = (A_pan * 6 * pi)/T_scan

    A_tilt = pi/6
    p0_tilt_scan = A_tilt
    v0_tilt_scan = 0

    t0 = 0
    tf = inf

    # Starting parameters
    traj = Traj.HOLD
    mode = Mode.GOHOME
    r_match = 0.3
    obj_interest = 0

    objectpan = 0
    objecttilt = 0

    feedback = group.get_next_feedback(reuse_fbk=feedback)
    phold_pan = feedback.position[0]
    phold_tilt = feedback.position[1]

    while True: 
        if traj is Traj.SPLINE:
            # Compute the commands for this time step.
            params_pan = calc_params(t0, tf, p0_pan, pf_pan, v0_pan, vf_pan, vmax_pan)
            a_pan, b_pan, c_pan, d_pan = params_pan

            params_tilt = calc_params(t0, tf, p0_tilt, pf_tilt, v0_tilt, vf_tilt, vmax_tilt)
            a_tilt, b_tilt, c_tilt, d_tilt = params_tilt

            pcmd_pan = splinecmds(t0, t, a_pan, b_pan, c_pan, d_pan)[0]
            vcmd_pan = splinecmds(t0, t, a_pan, b_pan, c_pan, d_pan)[1]

            pcmd_tilt = splinecmds(t0, t, a_tilt, b_tilt, c_tilt, d_tilt)[0]
            vcmd_tilt = splinecmds(t0, t, a_tilt, b_tilt, c_tilt, d_tilt)[1]    

        elif traj is Traj.SCAN:
            pcmd_pan = A_pan * sin(2 * pi * 3 * (t - t0)/T_scan)
            vcmd_pan = A_pan * (6 * pi / T_scan) * cos(6 * pi * (t - t0)/T_scan)
            
            pcmd_tilt = A_tilt * cos(2 * pi * (t - t0)/T_scan)
            vcmd_tilt = -A_tilt * (2 * pi / T_scan) * sin(2 * pi * (t - t0)/T_scan)

        elif traj is Traj.HOLD:
            pcmd_pan = phold_pan
            vcmd_pan = 0

            pcmd_tilt = phold_tilt
            vcmd_tilt = 0
        else:
            raise ValueError(f'Bad trajectory type {traj}')


    # Send the commands.  This returns immediately.
        command.position = [pcmd_pan, pcmd_tilt]
        command.velocity = [vcmd_pan, vcmd_tilt]
        group.send_command(command)


    # Read the actual data. This blocks (internally waits) 10ms for
    # the data and therefor replaces the "sleep(0.01)".
        feedback = group.get_next_feedback(reuse_fbk=feedback)
        pact_pan = feedback.position[0]
        vact_pan = feedback.velocity[0]

        pact_tilt = feedback.position[1]
        vact_tilt = feedback.velocity[1]

        phold_pan = pact_pan
        phold_tilt = pact_tilt

        if shared.lock.acquire():
            shared.motorpan = pact_pan
            shared.motortilt = pact_tilt
            shared.lock.release() 

        if kbhit():
        # Grab and report the key
            c = getch()
            print("Saw key ’%c’" % c)

            if c == "s":
                #Scanning
                p0_pan = pcmd_pan
                pf_pan = p0_pan_scan

                p0_tilt = pcmd_tilt
                pf_tilt = p0_tilt_scan

                v0_pan = vcmd_pan
                vf_pan = v0_pan_scan
                vmax_pan = 1.4

                v0_tilt = vcmd_tilt
                vf_tilt = v0_tilt_scan
                vmax_tilt = 1.2

                t0 = t
                tf = max(get_time(p0_pan, pf_pan, v0_pan, vf_pan, vmax_pan), get_time(p0_tilt, pf_tilt, v0_tilt, vf_tilt, vmax_tilt), 0.1) + t0

                traj = Traj.SPLINE
                mode = Mode.SCANNING

                historyofobjects = []

            if c == "z":
                # To (0, 0)
                p0_pan = pcmd_pan
                pf_pan = 0

                p0_tilt = pcmd_tilt
                pf_tilt = 0

                v0_pan = vcmd_pan
                vf_pan = 0
                vmax_pan = 1.4

                v0_tilt = vcmd_tilt
                vf_tilt = 0
                vmax_tilt = 1.2

                t0 = t
                tf = max(get_time(p0_pan, pf_pan, v0_pan, vf_pan, vmax_pan), get_time(p0_tilt, pf_tilt, v0_tilt, vf_tilt, vmax_tilt), 0.1) + t0

                traj = Traj.SPLINE
                mode = Mode.GOHOME
            if c == "t":
                # Tracking
                p0_pan = pcmd_pan
                pf_pan = pcmd_pan

                p0_tilt = pcmd_tilt
                pf_tilt = pcmd_tilt

                v0_pan = vcmd_pan
                vf_pan = 0
                vmax_pan = 1.4

                v0_tilt = vcmd_tilt
                vf_tilt = 0
                vmax_tilt = 1.2

                t0 = t
                tf = inf

                traj = Traj.SPLINE
                mode = Mode.TRACKING
            if c == "q":
                break
        
        if shared.lock.acquire():
            if shared.newdata == True:
                historyofobjects = historyofobjects + shared.detectedobjects
                if len(knownobjects) == 0 and len(historyofobjects) != 0:
                    knownobjects.append(historyofobjects[0])
                elif len(knownobjects) != 0 and len(historyofobjects) != 0:
                    for detectedobject in shared.detectedobjects:
                        newobject = True
                        for j, knownobject in enumerate(knownobjects):
                            if (abs(detectedobject[0] - knownobject[0]) < r_match and abs(detectedobject[1] - knownobject[1]) < r_match):
                                knownobjects[j] = detectedobject
                                newobject = False
                        if newobject:
                            knownobjects.append(detectedobject)

                shared.newdata = False

            shared.lock.release()

        if mode is Mode.TRACKING:
            objectpan = knownobjects[obj_interest % len(knownobjects)][0]
            objecttilt = knownobjects[obj_interest % len(knownobjects)][1]
                                    
            p0_tilt = pcmd_tilt
            pf_tilt = objecttilt
            v0_tilt = vcmd_tilt
            vf_tilt = 0

            p0_pan = pcmd_pan
            pf_pan = objectpan
            v0_pan = vcmd_pan
            vf_pan = 0 

            t0 = t
            tf = t + max(get_time(p0_pan, pf_pan, v0_pan, vf_pan, vmax_pan), get_time(p0_tilt, pf_tilt, v0_tilt, vf_tilt, vmax_tilt), .2)

            traj = Traj.SPLINE

        
        if traj is Traj.SPLINE and t+dt > tf:
            if mode is Mode.SCANNING:
                # In SCANNING mode: Transition to the SCAN trajectory.
                traj = Traj.SCAN
                t0 = t
            elif mode is Mode.GOHOME:
                # IN GOHOME mode: Transition to the HOLD trajectory.
                traj = Traj.HOLD
                t0 = t
            elif mode is Mode.TRACKING:
                traj = Traj.HOLD
                t0 = t
            else:
                # This shouldn’t happen.
                raise ValueError('Unexpected end of motion')

    # Store the data for this time step (at the current index).
        if index < N:
            Time[index] = t
            PAct_pan[index] = pact_pan
            PCmd_pan[index] = pcmd_pan
            VAct_pan[index] = vact_pan
            VCmd_pan[index] = vcmd_pan

            PAct_tilt[index] = pact_tilt
            PCmd_tilt[index] = pcmd_tilt
            VAct_tilt[index] = vact_tilt
            VCmd_tilt[index] = vcmd_tilt

            if shared.lock.acquire():
                objectpan = shared.objectpan
                objecttilt = shared.objecttilt
                shared.lock.release() 

            Object_pan[index] = objectpan
            Object_tilt[index] = objecttilt

    # Advance the index/time.
        index += 1
        t     += dt
        if index % 200 == 0:
            obj_interest += 1


    fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

    ax1.plot(Time[0:index], PAct_pan[0:index], color='blue', linestyle='-',  label='Act')
    ax1.plot(Time[0:index], PCmd_pan[0:index], color='blue', linestyle='--', label='Cmd')
    ax1.plot(Time[0:index], PAct_tilt[0:index], color='green', linestyle='-',  label='Act')
    ax1.plot(Time[0:index], PCmd_tilt[0:index], color='green', linestyle='--', label='Cmd')

    ax2.plot(Time[0:index], VAct_pan[0:index], color='blue', linestyle='-',  label='Act')
    ax2.plot(Time[0:index], VCmd_pan[0:index], color='blue', linestyle='--', label='Cmd')
    ax2.plot(Time[0:index], VAct_tilt[0:index], color='green', linestyle='-',  label='Act')
    ax2.plot(Time[0:index], VCmd_tilt[0:index], color='green', linestyle='--', label='Cmd')

    ax3.plot(Time[0:index], Object_tilt[0:index], color='green', linestyle='-', label='Tilt')
    ax3.plot(Time[0:index], Object_pan[0:index], color='red', linestyle='-', label='Pan') 

    ax1.set_title('Step 8')
    ax1.set_ylabel('Rads')
    ax2.set_ylabel('Angular Velocity')
    ax2.set_xlabel('Time (s)')
    ax3.set_ylabel('Pan & Tilt')

    ax1.grid()
    ax2.grid()
    ax3.grid()

    ax1.legend()
    ax2.legend()
    ax3.legend() 

    ax1.set_ylabel('Tilt')
    ax1.set_xlabel('Pan')   

    x = []
    y = []

    for i in range(len(historyofobjects)):
        x.append(historyofobjects[i][0])
        y.append(historyofobjects[i][1])

    plt.show()

    
    plt.figure()
    plt.scatter(x, y, marker='o')
    plt.ylim(-1, 1)
    plt.xlim(-1.7, 1.7)

    plt.show()
    print('done')


if __name__ == "__main__":
    controller(None)

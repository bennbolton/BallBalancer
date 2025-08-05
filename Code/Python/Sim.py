import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import solve_ivp
import sympy as sym
import control as ctrl

class Plant():
    Ms = 5
    Mw = 1
    Mb = 7.135
    Mtot = Ms+Mw+Mb

    Rs = 0.115
    Rw = 0.05
    Rtot = Rs + Rw

    grav = 9.81
    L = Rs + 0.14

    Is = Ms*(Rs**2)*2/5
    Iw = 1.9e-3
    Idb = 2.4

    Lam = Mw*Rtot+Mb*L
    def __init__(self, controller=None, inital_input=np.array([0.0, 0.0]), friction=True, show_scat=False, control='Torque'):
        self.controllerFunc = controller
        self.control_input = inital_input
        self.control_torques = []
        self.friction = friction
        self.show_scat = show_scat
        self.control = control
    
    @property
    def controller(self, controllerFunc):
        self.controllerFunc = controllerFunc
    
    def M(s, q, lib=np):
        theta, psi = q
        if lib == np:
            m = np.array([
                [s.Is+(s.Rs**2)*s.Mtot+((s.Rs**2)/s.Rw**2)*s.Iw, s.Rs*s.Lam*np.cos(psi)- ((s.Rs**2)/s.Rw**2)*s.Iw],
                [s.Rs*s.Lam*np.cos(psi) - ((s.Rs**2)/s.Rw**2)*s.Iw, (s.Rtot**2)*s.Mw + ((s.Rs**2)/s.Rw**2)*s.Iw + s.Idb]
            ])
        elif lib == sym:
            m = sym.Matrix([
                [s.Is+(s.Rs**2)*s.Mtot+((s.Rs**2)/s.Rw**2)*s.Iw, s.Rs*s.Lam*sym.cos(psi)- ((s.Rs**2)/s.Rw**2)*s.Iw],
                [s.Rs*s.Lam*sym.cos(psi) - ((s.Rs**2)/s.Rw**2)*s.Iw, (s.Rtot**2)*s.Mw + ((s.Rs**2)/s.Rw**2)*s.Iw + s.Idb]
            ])
        return m
    
    def C(s, q, q_dot, lib=np):
        ''' Returns the Coriolis matrix C(q, q_dot)'''
        theta, psi = q
        d_theta, d_psi = q_dot
        if lib is np:
            c = np.array([
                [0, -1*s.Rs*s.Lam*d_psi*np.sin(psi)],
                [0, 0]
            ])
        elif lib is sym:
            c = sym.Matrix([
                [0, -1*s.Rs*s.Lam*d_psi*sym.sin(psi)],
                [0, 0]
            ])
        return c
    
    def G(s, q, lib=np):
        ''' Returns the Gravity vector G(q)'''
        theta, psi = q
        if lib is np:
            g = np.array([0, -s.Lam*s.grav*np.sin(psi)])
        elif lib is sym:
            g = sym.Matrix([0, -s.Lam*s.grav*sym.sin(psi)])
        return g
    
    def D(s, q_dot):
        u_theta = 1
        u_psi = 1
        theta_dot, psi_dot = q_dot
        d = np.array([u_theta*theta_dot, u_psi*psi_dot])
        return d
        
    def state2cartesian(s, state_coords):
        '''
        Converts state variables (q) into Cartesian coordinates for animation.

        Parameters:
            state_coords (array): [theta, psi] (generalized coordinates)

        Returns:
            tuple: (robot_COM_xy, virtual_wheel_xy, ball_xy)
        '''
        theta, psi = state_coords

        ball_xy = np.array([s.Rs*theta, 0.0])
        robot_COM_xy = np.array([s.Rs*theta + s.L*np.sin(psi), s.L*np.cos(psi)])
        virtual_wheel_xy = np.array([s.Rs*theta + s.Rtot*np.sin(psi), s.Rtot*np.cos(psi)])
                        
        return robot_COM_xy, virtual_wheel_xy, ball_xy
    
    def dynamics(s, t, state):
        '''
        Computes q_dot and q_ddot given current state and control input.

        Parameters:
            t (float): Current time
            state (array): [q, q_dot] where q = generalized coordinates, q_dot = velocities
            control_input (array): Torques/forces (tau) (size: nx1)

        Returns:
            dstate_dt (array): Time derivatives [q_dot, q_ddot]
        '''
        q = state[:2]
        q_dot = state[2:]
        
        # stationary ball
        # q[0] = 0
        # q_dot[0] = 0

        torque_in = s.controllerFunc(s, state, t=t)
        if s.control == "Accel":
            torque_in *= s.Iw
            
        translated_torques = np.array([torque_in*s.Rs/s.Rw, torque_in*-s.Rs/s.Rw])
        s.control_torques.append((t, torque_in))
        if s.friction:
            q_ddot = np.linalg.inv(s.M(q)) @ (translated_torques - s.C(q, q_dot) @ q_dot - s.G(q) - s.D(q_dot))
        else:
            q_ddot = np.linalg.inv(s.M(q)) @ (translated_torques - s.C(q, q_dot) @ q_dot - s.G(q))
        return np.concatenate((q_dot, q_ddot))



    def run(self, dur=10, dt=0.01, inital_state=np.array([0.0, 0.0, 0.0, 0.0])):
        self.dt = dt
        self.t_span = (0, dur)
        print("pre-solve")
        solution = solve_ivp(self.dynamics, self.t_span, inital_state, t_eval=np.arange(*self.t_span, dt), method='RK45')
        print("post-solve")
        theta_vals, psi_vals = solution.y[0], solution.y[1]  # Generalized coordinates
        self.time_vals = solution.t
        self.theta_vals, self.psi_vals = solution.y[0], solution.y[1]
        self.theta_dot_vals, self.psi_dot_vals = solution.y[2], solution.y[3]
        self.phi_dot_vals = [(self.Rs/self.Rw)*(self.theta_dot_vals[i]-self.psi_dot_vals[i]) for i in range(len(self.theta_dot_vals))]
        self.omega_vals = [self.L*self.theta_dot_vals[i]/self.Rw for i in range(len(self.theta_dot_vals))]
        
        robot_COM_xy_vals = []
        virtual_wheel_xy_vals = []
        ball_xy_vals = []

        for i in range(len(self.time_vals)):
            # if self.psi_vals[i] > np.pi / 2:
                
            robot_COM_xy, virtual_wheel_xy, ball_xy = self.state2cartesian([theta_vals[i], psi_vals[i]])
            robot_COM_xy_vals.append(robot_COM_xy)
            virtual_wheel_xy_vals.append(virtual_wheel_xy)
            ball_xy_vals.append(ball_xy)

        self.robot_COM_xy_vals = np.array(robot_COM_xy_vals)
        self.virtual_wheel_xy_vals = np.array(virtual_wheel_xy_vals)
        self.ball_xy_vals = np.array(ball_xy_vals)
        
        # print(self.control_torques)
        
    def initSim(s):
        s.fig, s.ax = plt.subplots(1, 2, figsize=(12, 5))
        max_x = max(s.theta_vals)*s.Rs
        min_x = min(s.theta_vals)*s.Rs
        pad = 0.1
        s.ax[0].set_xlim(min_x-s.Rs-pad, max_x+s.Rs + pad)
        s.ax[0].set_ylim(-s.Rs - pad, s.Rs+s.L+0.1)
        s.ax[0].set_aspect('equal')
        
        # Create circles for the bodies
        s.robot_COM_circle = plt.Circle((s.robot_COM_xy_vals[0]), 0.02, color='blue', label="Robot COM")
        s.virtual_wheel_circle = plt.Circle(s.virtual_wheel_xy_vals[0], s.Rw, color='green', label="Virtual Wheel")
        s.ball_circle = plt.Circle(s.ball_xy_vals[0], s.Rs, color='red', label="Ball")
        
        s.ax[0].add_patch(s.robot_COM_circle)
        s.ax[0].add_patch(s.virtual_wheel_circle)
        s.ax[0].add_patch(s.ball_circle)
        s.ax[0].legend()
        s.ball_line, = s.ax[0].plot([], [], '-')
        s.wheel_line, = s.ax[0].plot([], [], '-')
        
        max_y = max(np.concatenate((
            # s.theta_vals,
            s.theta_dot_vals,
            s.psi_vals,
            s.psi_dot_vals,
            s.phi_dot_vals,
            # s.omega_vals
            )))
        min_y = min(np.concatenate((
            # s.theta_vals,
            s.theta_dot_vals,
            s.psi_vals,
            s.psi_dot_vals,
            s.phi_dot_vals,
            # s.omega_vals
            )))
        s.ax[1].set_xlim(s.t_span)
        s.ax[1].set_ylim(min_y-pad*10, max_y+pad*10)
        s.ax[1].set_title("State Variables Over Time")
        s.ax[1].set_xlabel("Time (s)")
        s.ax[1].set_ylabel("State Values")
        # s.theta_line, = s.ax[1].plot([], [], 'b-', label=r'$\theta$')
        s.psi_line, = s.ax[1].plot([], [], 'g-', label=r'$\psi$')
        s.theta_dot_line, = s.ax[1].plot([], [], 'b--', label=r'$\dot{\theta}$')
        s.psi_dot_line, = s.ax[1].plot([], [], 'g--', label=r'$\dot{\psi}$')
        s.control_torque_line, = s.ax[1].plot([], [], 'r-', label=r'$\tau$', zorder=1)
        s.phi_dot_line, = s.ax[1].plot([], [], '-', label=r'$\dot{\phi}$')
        # s.omega_line, = s.ax[1].plot([], [], '-', label=r'$\omega$')
        s.ax[1].legend()
        
        if s.show_scat: s.torque_scatter = s.ax[1].scatter([], [], color='black', marker='o', label='Torque Points', s=5, zorder=2)
        
        s.paused = False
        s.fig.canvas.mpl_connect('button_press_event', s.toggle_pause)
        
    def toggle_pause(self, *args, **kwargs):
        self.paused = not self.paused
        if self.paused:
            self.ani.pause()
        else:
            self.ani.resume()
        
        
    def updateFrame(s, frame):
        
        s.robot_COM_circle.set_center(s.robot_COM_xy_vals[frame])
        s.virtual_wheel_circle.set_center(s.virtual_wheel_xy_vals[frame])
        s.ball_circle.set_center(s.ball_xy_vals[frame])
        
        s.ball_line.set_data([s.ball_xy_vals[frame][0], s.Rs*np.cos(-s.theta_vals[frame]) + s.ball_xy_vals[frame][0]], [0, s.Rs*np.sin(-s.theta_vals[frame])])
        
        phi = (s.Rs/s.Rw)*(s.theta_vals[frame]-s.psi_vals[frame])
        s.wheel_line.set_data([s.virtual_wheel_xy_vals[frame][0], s.Rw*np.cos(phi) + s.virtual_wheel_xy_vals[frame][0]], [s.virtual_wheel_xy_vals[frame][1], s.Rw*np.sin(phi)+s.virtual_wheel_xy_vals[frame][1]])
        # s.theta_line.set_data(s.time_vals[:frame], s.theta_vals[:frame])
        s.psi_line.set_data(s.time_vals[:frame], s.psi_vals[:frame])
        s.theta_dot_line.set_data(s.time_vals[:frame], s.theta_dot_vals[:frame])
        s.psi_dot_line.set_data(s.time_vals[:frame], s.psi_dot_vals[:frame])
        s.phi_dot_line.set_data(s.time_vals[:frame], s.phi_dot_vals[:frame])
        # s.omega_line.set_data(s.time_vals[:frame], s.omega_vals[:frame])
        
        time = s.time_vals[frame]
        
        s.control_torques.sort(key=lambda x: x[0])  
        times, torques = zip(*dict(s.control_torques).items())
        
        mask = times <= time
        times = np.array(times)[mask]
        torques = np.array(torques)[mask]
        
        # torque line:
        s.control_torque_line.set_data(times, torques)
        
        # UnComment to show scatter of torque calculated points
        if s.show_scat: s.torque_scatter.set_offsets(np.column_stack((times, torques)))
        items = [s.robot_COM_circle, 
                 s.virtual_wheel_circle, 
                 s.ball_circle, 
                #  s.theta_line, 
                 s.psi_line, 
                #  s.theta_dot_line, 
                 s.psi_dot_line, 
                 s.control_torque_line, 
                 s.ball_line, 
                 s.wheel_line, 
                 s.phi_dot_line, 
                #  s.omega_line
                 ]
        if s.show_scat: items.append(s.torque_scatter)
        return items
    
    def show(self):
        self.initSim()
        self.ani = animation.FuncAnimation(self.fig, self.updateFrame, frames=len(self.time_vals), interval=self.dt*1000, blit=True)
        plt.show()
        

class PDController():
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.count = 0
        self.saved = []
        self.integ_add = 0
        
    def torque_control(self, plant, state, desired_state=[0,0,0,0], t=None):
        q = state[:2]
        q_dot = state[2:]
        
        q_desired = desired_state[:2]
        q_dot_desired = desired_state[2:]
        
        psi_desired = q_desired[1]
        psi = q[1]
        error = psi-psi_desired

        psi_dot_desired = q_dot_desired[1]
        psi_dot = q_dot[1]

        self.integ_add += error
        return self.Kp*error + self.Kd*(psi_dot-psi_dot_desired) + self.Ki*self.integ_add
    
def noControl(plant, state, t=None):
    return 0

class LQRControl():
    def __init__(self, plant):
        self.calcSS(plant)
    
    def control(s, plant, state, desired_state=[0,0,0,0], t=None):
        u = -s.K @ state
        return u[0]
    
    def calcSS(self, plant: Plant):
        theta, psi, theta_dot, psi_dot, ddphi = sym.symbols('theta psi theta_dot psi_dot ddphi')
        M = plant.M([theta, psi], lib=sym)
        C = plant.C([theta, psi], [theta_dot, psi_dot], lib=sym)
        G = plant.G([theta, psi], lib=sym)
        q_ddot = M.inv() * (sym.Matrix([[ddphi*plant.Iw*plant.Rs/plant.Rw], [ddphi*plant.Iw*-plant.Rs/plant.Rw]]) - C * sym.Matrix([[theta_dot], [psi_dot]]) - G)

        x = sym.Matrix([theta, psi, theta_dot, psi_dot])
        f = sym.Matrix([theta_dot, psi_dot, q_ddot[0], q_ddot[1]])

        A = f.jacobian(x)
        B = f.jacobian(sym.Matrix([ddphi]))

        # Evaluate at equilibrium (theta=0, psi=0, theta_dot=0, psi_dot=0, tau=0)
        A_lin = A.subs({theta: 0, psi: 0, theta_dot: 0, psi_dot: 0, ddphi: 0})
        B_lin = B.subs({theta: 0, psi: 0, theta_dot: 0, psi_dot: 0, ddphi: 0})

        A = sym.matrix2numpy(A_lin)
        B = sym.matrix2numpy(B_lin)
        
        Q = np.diag([5, 500, 500, 5])
        R = np.array([[2]])  # Penalise control effort

        K, _, _ = ctrl.lqr(A, B, Q, R)
        # print("LQR Gain Matrix K:", K)
        self.K = K
    
class TorqueTrackControl():
    def __init__(self, Kp, Ki):
        self.I = 0
        self.Kp = Kp
        self.Ki = Ki
        self.saved = []
    def torqueTrackControl(self, plant, state, desired_phi_dot=1, t=None):
        phi_dot = (state[2]-state[3])*(plant.Rs/plant.Rw)
        error = desired_phi_dot-phi_dot
        self.I += error * plant.dt
        tau = self.Kp * error + self.Ki * self.I
        if t is not None: self.saved.append((t, tau))
        return tau
    
    def PDControl(self, plant, state):
        Kp = 1
        error = -state[1]
        velo_command = error*Kp
        return self.torqueTrackControl(plant, state, 1)
        
Kp = 10
Kd = 5
Ki = 0

if __name__ == "__main__":
    # velocityTorque = TorqueTrackControl(1, 0)

    # controller = PDController(Kp, Kd, Ki)
    # system = Plant(controller.torque_control, friction=True)


    # # # system = Plant(velocityTorque.torqueTrackControl, show_scat=True)
    # system = Plant(control='Accel')


    system = Plant(control='Accel')
    LQR = LQRControl(system)
    system.controllerFunc = LQR.control


    # system.controllerFunc = controller.torque_control
    # system = Plant(noControl)
    system.run(10, inital_state=np.array([0,0.03,0,0]))
    system.show()
import numpy as np

def test_koop():
    print('koopman')
    
class Koop_LQR():
    def __init__(self,dt=0.0,x_state=4,u_state=1, x_curr = 0.0, theta_curr = 0.0, u_curr = 0.0, xdot_curr = 0.0, thetadot_curr = 0.0):
        """_summary_

        Args:
            dt (float): sampling time Defaults to 0.
            x_state (float): number of state observations
            u_state: number of input observtations
            x_curr (float): current x position
            theta_curr (float): current x
            u_curr (float): current cmd_vel x input
            xdot_curr (float): current linear velocity
            thetadot_curr (float): current rod velocity
            
        """
        
        self.dt = dt
        self.state_obs = x_state+3
        self.u_obs = u_state+1
        
        self.x_curr = x_curr
        self.theta_curr = theta_curr
        self.thetadot_curr = thetadot_curr
        self.u_curr = u_curr
        self.xdot_curr = xdot_curr
        self.udot_curr = udot_curr
        
        self.xdot_old = xdot_curr
        self.thetadot_old = thetadot_curr
        self.u_old = u_curr
        
        
    
        def update_vel_states(xdot,thetadot,u):
            """_summary_

            Args:
                xdot (float): cart velocity
                thetadot (_type_): rod angular velocity
                u (_type_): cmd_vel command
            """
            
            self.xdot_old = self.xdot_curr
            self.thetadot_old = self.thetadot_curr
            self.u_old = self.u_curr
            
            self.xdot_curr = xdot
            self.thetadot_curr = thetadot
            self.u_curr = u
            
        def compute_accel_states():
            """_summary_

            Returns:
                xddot (float): linear accel
                thetaddot (float): ang_accel
            """
            
            xddot = (self.xdot_curr - self.xdot_old)/self.dt
            thetaddot = (self.thetadot_curr - self.thetadot_old)/self.dt
            
            return xddot, thetaddot
    
        def compute_uddot():
            """_summary_

            Returns:
                udot: cmd_accel
            """
            
            return ((self.u_curr - self.u_old)/self.dt)
        
        
            
            
      
        
                
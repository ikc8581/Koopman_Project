import numpy as np
import scipy as sp

from control import lqr

"""
Koopman based LQR control package
"""


def test_koop():
    print('koopman')
    

    
class Koop_LQR():
    
    def __init__(self,dt=0.0,x_state=4,u_state=1, x_curr = 0.0, theta_curr = 0.0, u_curr = 0.0, xdot_curr = 0.0, thetadot_curr = 0.0, iterations = 1000):
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
            udot_curr (float): cmd_accel
            
        """
        print('STARTING KOOPMAN!!!!!!!!!!!!!')
        self.dt = dt
        
        self.iterations = iterations
        self.iteration_curr = 0
        
        self.state_obs = x_state+3
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        
        self.Q = np.eye(self.state_obs)
        self.Q[4,4] = 0
        self.Q[5,5] = 0
        self.Q[6,6] = 0
        
        # self.R = np.eye(self.u_obs)
        # self.R[1,1] = 0
        
        self.R = np.eye(u_state)
        
        self.K_control = np.zeros( [self.u_obs,self.state_obs] )
        
        self.state_matrix = np.zeros([self.state_obs,self.state_obs])
  
        self.x_curr = x_curr
        self.xdot_curr = xdot_curr
        self.theta_curr = theta_curr
        self.thetadot_curr = thetadot_curr
        self.u_curr = u_curr
        # self.udot_curr = udot_curr
        
        self.x_old = x_curr
        self.xdot_old = xdot_curr
        self.theta_old = theta_curr
        self.thetadot_old = thetadot_curr
        self.u_old = u_curr
        # self.udot_old = udot_curr
        
    def dummy_method(self):
        print('dummy method')
            
    def train_model(self,x,xdot,theta,thetadot, u):
        
        """train model parameters using Koopman

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular acceleration
        
        Returns:
            self.iteration_curr (int): current training iteration - used to determine when to finalize model
        """
    
        self.iteration_curr += 1
        
        if self.iteration_curr<self.iterations:
            
            self.x_old = self.x_curr
            self.xdot_old = self.xdot_curr
            self.theta_old = self.theta_old 
            self.thetadot_old = self.thetadot_curr
            self.u_old = self.u_curr
            
            self.x_curr = x
            self.xdot_curr = xdot
            self.theta_curr = theta
            self.thetadot_curr = thetadot
            self.u_curr = u
            # self.udot_curr = udot
            
            psix_old = self.zee_x(self.x_old, self.xdot_old, self.theta_old, self.thetadot_old) #This is an array
            psix_curr = self.zee_x(self.x_curr, self.xdot_curr, self.theta_curr, self.thetadot_curr)
            
            
            psiu_old = self.zee_u(self.theta_old, self.u_old)
            psiu_curr = self.zee_u(self.theta_curr, self.u_curr)
            
            print('psixold type: ', type(psix_old))
            print('psiuold:', type(psiu_old))
            
            koop_old = np.concatenate((psix_old, psiu_old))
            koop_curr = np.concatenate((psix_curr, psiu_curr))
            
            self.A += np.outer(koop_old,koop_old) / self.iteration_curr
            self.G += np.outer(koop_old,koop_curr) / self.iteration_curr
            
        if self.iteration_curr == self.iterations:
            self.calculateK()                        
        
        return self.iteration_curr
    

    
    def zee_x(self,x,xdot,theta,thetadot):
        """Compute state observation parameters

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular acceleration
        """
        
        return np.array([x,xdot,theta,thetadot,1,np.sin(theta),np.cos(theta)])
    
    def zee_u(self,theta,u):
        """compute control input observation parameters

        Args:
            theta (float): rod angle
            u (float): cmd_vel command , np.cos(theta)
        """
        
        return np.array([u])
        

    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
        print('A: ', self.A)
        print('G: ', self.G)
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))
        print('K: ', self.K)
        self.K = np.real(sp.linalg.logm(self.K))/self.dt
        print('K continuous: ', self.K)
        
    
    
    def returnKoopState(self):
        """
        Incorporates basis functions along with base states
        Returns:
            the state A matrix
            the control B matrix
        """
        
        return self.K[:self.state_obs,:self.state_obs], self.K[:self.state_obs,(self.state_obs):]
        
    
    def computeLQR(self):
        """Compute LQR controller for data-based Koopman model

        Returns:
            Gain matrix (7x2 for cart pole system)
        """
        A_state, B_state = self.returnKoopState()
        print('A_state: ', A_state)
        print('A state shape ', np.shape(A_state))
        print('B_state: ', B_state)
        print('B state shape ', np.shape(B_state))
        gains = lqr(A_state,B_state,self.Q,self.R)
        print('LQR matrix: ', gains)
        print('gain matrix dimensions: ', np.shape(gains))
        return gains
    
    def compute_cmd(self,x,xdot,theta,thetadot, u, K):
        """Compute the control effort to exert based on the current LQR state model and current states
        
        Args:
            x (float): cart x position
            xdot (float): cart x velocity
            theta (float): rod angle
            thetadot (float): rod angular velocity
            K: control gain matrix
        Returns:
            v (float): linear cmd magnitude to exert
        """
        
        state = np.concatenate( (self.zee_x(x,xdot,theta,thetadot), self.zee_u( u ) )
        desired = np.concatenate( (self.zee_x(0,0,0,0), self.zee_u(0, 0))  )
        
        control_state = state-desired
        
        cmd = np.dot(K, control_state.T)
        
        
        return cmd
        
        
        
        #one of the Koopman u components is not a physical component, but a basis function. How do I resolve that?
        #the lqr algorihm should be solved for the for u, not koopman space v , and for z we should exclude all non observables see paper page 3
        
        
    def update_vel_states(self,xdot,thetadot,u):
        """update velocity states

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
        """compute linear and angular accel

        Returns:
            xddot (float): linear accel
            thetaddot (float): ang_accel
        """
        
        xddot = (self.xdot_curr - self.xdot_old)/self.dt
        thetaddot = (self.thetadot_curr - self.thetadot_old)/self.dt
        
        return xddot, thetaddot

    def compute_uddot(self):
        """comput cmd_accel

        Returns:
            udot: cmd_accel
        """
        
        return ((self.u_curr - self.u_old)/self.dt)
    
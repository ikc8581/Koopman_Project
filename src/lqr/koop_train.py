import numpy as np
import scipy.linalg as spicy_linalg

"""
Non control koopman training
"""
    
class Koop():
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
            udot_curr (float): cmd_accel
            
        """
        
        self.dt = dt
        
        self.state_obs = x_state+3
        self.u_obs = u_state
        self.total_obs = self.u_obs + self.state_obs
        
        self.A = np.zeros([self.total_obs,self.total_obs])
        self.G = np.zeros([self.total_obs,self.total_obs])
        self.K = np.zeros([self.total_obs,self.total_obs])
        self.Kcont = np.zeros([self.total_obs,self.total_obs])
        
        
        self.x_curr = x_curr
        self.xdot_curr = xdot_curr
        self.theta_curr = theta_curr
        self.thetadot_curr = thetadot_curr
        self.u_curr = u_curr
        
        self.x_old = x_curr
        self.xdot_old = xdot_curr
        self.theta_old = theta_curr
        self.thetadot_old = thetadot_curr
        self.u_old = u_curr
        
        self.divider = 1


    def train_model(self,x,xdot,theta,thetadot, u):
        
        """train model parameters using Koopman

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular acceleration
        
        Returns:
            None
        """
        
        self.x_old = self.x_curr
        self.xdot_old = self.xdot_curr
        self.theta_old = self.theta_curr
        self.thetadot_old = self.thetadot_curr
        self.u_old = self.u_curr
        
        self.x_curr = x
        self.xdot_curr = xdot
        self.theta_curr = theta
        self.thetadot_curr = thetadot
        self.u_curr = u
        
        psix_old = self.zee_x(self.x_old, self.xdot_old, self.theta_old, self.thetadot_old) #This is an array
        psix_curr = self.zee_x(self.x_curr, self.xdot_curr, self.theta_curr, self.thetadot_curr)
        
        psiu_old = self.zee_u(self.u_old)
        psiu_curr = self.zee_u(self.u_curr)

        koop_old = np.concatenate((psix_old, psiu_old))
        koop_curr = np.concatenate((psix_curr, psiu_curr))
        
        self.A += np.outer(koop_curr,koop_old) / self.divider 
        self.G += np.outer(koop_old,koop_old) / self.divider
        self.divider+=1
        
    def zee_x(self,x,xdot,theta,thetadot):
        """Compute state observation parameters

        Args:
            x (float): cart position
            xdot (float): cart velocity
            theta (float): rod angle
            thetadot (float): rod angular acceleration
        """
        
        return np.array([x,xdot,theta,thetadot,1,np.sin(theta),np.cos(theta)])
    
    def zee_u(self,u):
        """compute control input observation parameters

        Args:
            u (float): cmd_vel command , np.cos(theta)
        """
        
        return np.array([u])
    
    
    def calculateK(self):
        """
        K matrix calculation (continuous)
        
        """
        # print('A: ', self.A)
        # print('G: ', self.G)
        # print('current divider: ', (self.cycle_count*(self.iteration_curr+1)))
        
        self.K =  np.dot(self.A,np.linalg.pinv(self.G))
        # self.K =  np.dot(self.A/(self.cycle_count*(self.iteration_curr+1)),np.linalg.pinv(self.G/(self.cycle_count*(self.iteration_curr+1))))
        
        
        # print('K: ', self.K)
        # self.Kcont = np.real(spicy_linalg.logm(self.K))/self.dt
        K_reduced = self.K[:4,:]
        print('K reduced : ',  K_reduced)
        # print('K continuous: ', self.Kcont)

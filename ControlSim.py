#!/usr/bin/python
"""
2D Unicycle model-based robot simulator with measurement and process noise
for CIS 390 Fall 2015 at the University of Pennsylvania
"""

from matplotlib import pyplot as plt
import numpy as np
import time

class CreateSim(object):
    def __init__(self):
        """
        Initialize starting pose and uncertaqinty parameters here. Goal is always the origin.
        """
        self.done=False
        self.x_gt = np.array([[1,1,-3*np.pi/4-0.2]]).T
        self.v = 0.05
        self.omega = 0
        self.Q_t = np.eye(3)
        self.R_t = np.eye(3)
        self.x_t = np.array([[0,0,0]]).T
        self.P_t = np.array([[10.**6,0,0],[0,10.**6,0],[0,0,10.**6]])
        self.dt=1.0/60
        self.noise = 0.01

    def get_marker_pose(self):
        """
        Functions are the same as on the Creates, although there is no
        simulation for fresh detections or field of view constraints
        """
        x_noisy = self.x_gt + np.array([np.random.normal(0, self.noise, 3)]).T
        return x_noisy,True

    def F_matrix(self,dt,v,theta_t):
        """
        Same as on the Creates.
        """ 
        return np.array([[1, 0, -dt*v*np.sin(theta_t)],
                         [0, 1,  dt*v*np.cos(theta_t)],
                         [0, 0,                     1]])

    def command_velocity(self,vx,wz):
        """
        Simulate the robot's motion using Euler integration
        """
        vx_noisy = vx + np.random.normal(0, self.noise)
        wz_noisy = wz + np.random.normal(0, self.noise)
        self.x_t[0,0]+=self.dt*vx_noisy*np.cos(self.x_t[2,0])
        self.x_t[1,0]+=self.dt*vx_noisy*np.sin(self.x_t[2,0])
        self.x_t[2,0]+=self.dt*wz_noisy
        if self.x_t[2,0]>np.pi:
            self.x_t[2,0]-=2*np.pi
        if self.x_t[2,0]<-np.pi:
            self.x_t[2,0]+=2*np.pi
        # This part computes robot's motion based on the groundtruth
        self.x_gt[0,0]+=self.dt*vx*np.cos(self.x_gt[2,0])
        self.x_gt[1,0]+=self.dt*vx*np.sin(self.x_gt[2,0])
        self.x_gt[2,0]+=self.dt*wz
        if self.x_gt[2,0]>np.pi:
            self.x_gt[2,0]-=2*np.pi
        if self.x_gt[2,0]<-np.pi:
            self.x_gt[2,0]+=2*np.pi
        if np.sqrt(self.x_t[0,0]**2+self.x_t[1,0]**2)<0.01:
            self.done=True
        return
        
    def command_create(self):
        """ 
        YOUR CODE HERE
        """
        MAX_SPEED=0.1
        """
        dt = None
        if self._last_time is not None:
            dt = (rospy.Time.now() - self._last_time).to_sec()
        self._last_time = rospy.Time.now()
        """

        z_t, fresh = self.get_marker_pose()
        print("z: " + str(z_t))
        if fresh:
            # update step
            K = np.dot(self.P_t, np.linalg.inv(self.P_t + self.R_t))
            self.x_t = self.x_t + np.dot(K, z_t - self.x_t)
            self.P_t = np.dot(np.eye(3) - K, self.P_t)
        elif self.dt:
            # prediction step
            F = self.F_matrix(self.dt, self.v, self.x_t[2][0])
            self.x_t[0][0] = self.x_t[0][0] + self.v*self.dt*np.cos(self.x_t[2][0])
            self.x_t[1][0] = self.x_t[1][0] + self.v*self.dt*np.sin(self.x_t[2][0])
            self.x_t[2][0] = self.x_t[2][0] + self.omega*self.dt 
            self.P_t = np.dot(np.dot(F, self.P_t), F.T) + self.Q_t
        print("x: " + str(self.x_t))


        return

def main():
    sim = CreateSim()
    fig = plt.figure(1,figsize=(5,5),dpi=90)
    ax=fig.add_subplot(111)
    plt.ylim((-2,2))
    plt.xlim((-2,2))
    ax.plot(sim.x_t[0,0],sim.x_t[1,0],'rx')
    plt.hold(True)
    iter = 0
    while not sim.done and iter<1000:
        sim.command_create()
        if (iter%10)==0:
            ax.plot(sim.x_t[0,0],sim.x_t[1,0],'rx')
            ax.plot(sim.x_gt[0,0],sim.x_gt[1,0],'gx')
            ax.arrow(sim.x_t[0,0],sim.x_t[1,0],0.05*np.cos(sim.x_t[2,0]),0.05*np.sin(sim.x_t[2,0]),head_width=0.005,head_length=0.01)
            plt.draw()
            plt.show(block=False)
        iter+=1
    if sim.done:
        print "Goal reached!"
    else:
        print "Max iters reached"
    plt.show()

if __name__ == "__main__":
    main()

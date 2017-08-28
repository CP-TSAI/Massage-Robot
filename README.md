# Massage-Robot
#### The result is accepted by IEEE International Conference on Robotics and Automation (ICRA) 2017.
- The objective of this research is to present a percussive massage based on robotic tapping motion.
- It is the first research to discuss the massage technique for this kind. 

# Difficulties:
- The tapping motion is different from the common contact motion.
- It satisfies with the short-time contact on human muscles, and the robot end-effector itself has an initial velocity and acceleration to make an impulse contact. 

# Solutions:
- To tackle with the tapping motion problems, we utilize the online trajectory generator (OTG) based on the impedance control, which ensures the safety of human without any possible accident which can hurt human. 
- We implement the tapping motion with a dual arm robot developed at the iCeiRA lab in NTU. 
- By the Cartesian space teach function and the concept of virtual point, the tapping force and contact position can be changed adaptively to meet different needs.

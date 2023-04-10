# Autonomous_driving

1 Task 1 – Simultaneous Localisation and Mapping (SLAM)
In Task_1_SLAM.zip, the particle filter SLAM algorithm has been designed to navigate two UGVs around the landmarks. In the current settings, relative position of the landmarks is measured from the UGVs in x-y axis. The details of the code are given in the Lab Sessions on SLAM.
1.1 Particle filter SLAM with range and bearing measurements
Let us assume that the vehicle is equipped with a LIDAR sensor, which returns the range and bearing measurements of the landmarks. Hence, the measurement equation is changed as follows:

􏰀􏰂(xL(k)−x(k))2+(yL(k)−y(k))2 􏰁
z(k) ≜ h(xL(k), x(k)) + v = tan−1 ((yL(k) − y(k))/(xL(k) − x(k))) + v, (1)

where the measurement noise is changed to v1 ∼N(0,12)mforrangemeasurements,andv2 ∼N(0,0.12) rad for bearing measurements.

Derive the measurement Jacobian matrix and write the equation in the report [2.5 marks]. Modify the MATLAB codes accordingly to implement the SLAM algorithm with range and bearing measurements, and write the modified parts of the code in the report [5 marks]. Compare the estimation error with the x-y relative position measurements, discuss the reason why the differences occur, and propose alternative methods to improve the estimation accuracy [5 marks].

Hint: Difference in angular measurements should be wrapped to [−π, π], and plot the animations with new types of measurements.

# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
・filter:
	When a new measurement value is received from the sensor, the prediction and the measurement value are updated.
    It also calculates the new state transition matrix and process noise covariance matrix based on the elapsed time.
・track management
	Decrease the track score for unassigned tracks and delete tracks if the score is below the threshold.
・association
	The measured values are associated using the nearest neighbor association and gating based on Mahalanobis distance.
・camera fusion
	The score is calculated using an appropriate sensor according to the position of the vehicle.
    
Implementing an association was the most difficult.
The code was so long that I had to write a complicated function and spent a lot of time trying to get rid of the "Stop Iteration" error when I ran it.


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
Theoretically:
As the number of sensors increases, so does the accuracy.
My conclete result:
It was found that the RMSE value decreased and the accuracy increased when there were two sensors than when there was only one sensor.


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
There is a problem that a vehicle that is not a vehicle but has a shape similar to a vehicle is tracked and falsely detected.
There was a time when the plant was detected as a vehicle even during the project.


### 4. Can you think of ways to improve your tracking results in the future?
I think there are ways to improve each of the hardware and software.
·hardware
The development of hardware that can digitize the surrounding environment more precisely may help improve accuracy.
For example, in Lidar, by firing a laser at a higher density, it becomes possible to grasp the shape of a detected object more accurately, and it becomes possible to distinguish between a vehicle and something other than a vehicle.
·software
Accuracy will be improved by understanding the characteristics of vehicles and non-vehicles and creating filters to distinguish them.
For example, white lines are often written on roads. By tracking the location of the white line and the object and calculating the location of the object with respect to the white line, it may be possible to recognize that the detected object is a vehicle.



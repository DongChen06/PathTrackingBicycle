# PathTrackingBicycle
Implementation of path tracking with a linear/non-linear bicycle model. We use the PID and standley controllers to control the longitudinal and lateral movements, respectively. We use the key idea of ref.[1], while replacing the vehicle dynamics in Carla simulator with linear/non-linear bicycle models. 

## Bicycle Models
-------

- Linear bicycle model. 

- Non-linear bicycle model.


## experiments
-------

We test vehicle models with PID and standley controllers. 

<p align="center">
     <img src="results/speed_linear.png" alt="output_example" width="60%" height="60%">
     <br>Fig.1 Speed tracking of linear vehicle model
</p>

<p align="center">
     <img src="results/trajectory_linear.png" alt="output_example" width="60%" height="60%">
     <br>Fig.2 Path tracking of linear vehicle model
</p>

The testing results on non-linear bicycle models.
<p align="center">
     <img src="results/speed_non_linear.png" alt="output_example" width="60%" height="60%">
     <br>Fig.3 Speed tracking of non-linear vehicle model
</p>

<p align="center">
     <img src="results/trajectory_non_linear.png" alt="output_example" width="60%" height="60%">
     <br>Fig.4 Path tracking of non-linear vehicle model
</p>


*Here we enlarge the throttle by 5 times for better visualization.*


## Reference:
-------

1. [Self Driving Cars Longitudinal and Lateral Control Design](https://github.com/enginBozkurt/SelfDrivingCarsControlDesign)

2. []()

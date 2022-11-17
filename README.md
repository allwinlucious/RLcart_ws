# ROS_RL_race
## Reinforcement Based Controller for a Race car simulation

Simulation setup provided as part of Automated and Connected Driving Challenges [course](https://www.ika.rwth-aachen.de/en/education/students/lectures/3769-acdc.html)

### Result:


https://user-images.githubusercontent.com/15308488/202494783-b1c17a73-7e5b-48b1-920b-fd41b717b815.mp4


### To train the controller :

```
roslaunch racing train_controller.launch
```


https://user-images.githubusercontent.com/15308488/202228639-98917b96-a021-4382-8478-2d69d033c135.mp4



### To run the trained controller :

edit the code to include the proper location of trained model. (./model/PPO_racing_cart3)

```
roslaunch racing RaceCar.launch
```

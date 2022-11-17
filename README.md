# ROS_RL_race
## Reinforcement Based Controller for a Race car simulation

Simulation setup provided as part of Automated and Connected Driving Challenges [course](https://www.ika.rwth-aachen.de/en/education/students/lectures/3769-acdc.html)
Proximal Policy Optmization was used.

Reward function:

lap complete : +1000
crash        : -100
everytimestep: linear velocity/100


### Result:
after ~1M iterations, the model is able to navigate the race track without major crashes for a lap. lap time was recorded to be 11-16 seconds. This can be improved  with further training.



https://user-images.githubusercontent.com/15308488/202496267-83247441-2916-47ac-bf98-174021926e8b.mp4



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

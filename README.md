# Robot Behaviors

## Drive in a Square

I decided to take the `odom` approach for this behavior. Since subscribing to this topic gives frequent updates, I used that subscription handler to update the linear and angular velocities of the robot. I simply have the robot drive forward until it reaches a certain x/y, and have it rotate 90 degrees between these forward drives, forming a square.

I broke down the robot behavior into a list of task functions and arguments to call these functions with. These task functions (`set_angle`, `drive_until_x`, and `drive_until_y`) must return `True` when the task completes, otherwise returning `False`. Thus, each time an `odom` update comes in, I call the current task, and if the task is complete I increment the task counter.

I had a number of helper functions dedicated to working with the start angle for the robot's rotation, the current angle, and the target angle. These helper functions determined which direction was most efficient for the robot to turn towards a certain angle, as well as to determine when the desired angle has been reached.

The driving tasks set the linear velocity, and the angle task sets the angular velocity. I also added a very slow angular velocity during the linear travel tasks, in order to correct the angle that the robot is driving at. This is because I discovered that turning extremely slow will result in a more accurate resulting angle, but also takes too long, so I chose to have a faster turn with some slight correction during the linear drive.

![Robot](https://github.com/Loonride/warmup_project/raw/main/gifs/drive_square.gif)

## Person Follower

![Robot](https://github.com/Loonride/warmup_project/raw/main/gifs/person_follower.gif)

## Wall Follower

TODO

# Challenges

A couple challenges were working with current robot angles, as well as handling the latency of commands so that I can point the robot in the desired direction.

# Future Work

TODO

# Takeaways

TODO

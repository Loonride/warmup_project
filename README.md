# Robot Behaviors

## Drive in a Square

I decided to take the `odom` approach for this behavior. Since subscribing to this topic gives frequent updates, I used that subscription handler to update the linear and angular velocities of the robot. I simply have the robot drive forward until it reaches a certain x/y, and have it rotate 90 degrees between these forward drives, forming a square.

I broke down the robot behavior into a list of task functions and arguments to call these functions with. These task functions (`set_angle`, `drive_until_x`, and `drive_until_y`) must return `True` when the task completes, otherwise returning `False`. Thus, each time an `odom` update comes in, I call the current task, and if the task is complete I increment the task counter.

I had a number of helper functions dedicated to working with the start angle for the robot's rotation, the current angle, and the target angle. These helper functions determined which direction was most efficient for the robot to turn towards a certain angle, as well as to determine when the desired angle has been reached.

The driving tasks set the linear velocity, and the angle task sets the angular velocity. I also added a very slow angular velocity during the linear travel tasks, in order to correct the angle that the robot is driving at. This is because I discovered that turning extremely slow will result in a more accurate resulting angle, but also takes too long, so I chose to have a faster turn with some slight correction during the linear drive.

![Robot](https://github.com/Loonride/warmup_project/raw/main/gifs/drive_square.gif)

## Person Follower

I approached this problem by using proportional control of speed and angular speed to quickly move towards the target when far away, then slow down when getting closer. If the robot is far way from the target and at a large angle from the target, the robot only turns until the target is in front it before driving forward, in order to ensure that the robot does not mistakenly lose the target from within its LiDAR range by driving away. The robot also does not drive at all within a close range of the target, as it is supposed to maintain a safe distance.

I handled all speed and angular speed setting in the callback of the `/scan` subscription. In there, I use the `ranges` array of the `LaserScan` to determine how far the nearest object is, and what direction it is in. The array has 360 elements which contain the scanned distance of each angle. This allowed me to determine the angle from the center of the robot of the target, which I used for proportional control of the angular speed.

At a distance of > 2m and an angle of > 60 degrees away from directly facing the target, the robot is only allowed to turn until it is at an angle < 60 degrees. This ensures that the robot doesn't run off and lose the target.

![Robot](https://github.com/Loonride/warmup_project/raw/main/gifs/person_follower.gif)

## Wall Follower

I approached this problem by first making proportional control of angle that tries to maintain a 90 degree angle between the robots direction and the nearest wall. I added a preferred direction for the robot to turn when it hits a corner, ensuring that the robot keeps going around the square room in the same direction. In addition to keeping track of angle with the nearby wall, I also make the robot turn if it is approaching something from the front. This allows the robot to start turning before it crashes into the oncoming wall.

Like my person follower, my wall follower sets linear and angular speeds in the callback of the `/scan` topic. If there is no wall nearby, the robot simply drives forward until it finds one. It then selects a turning direction that it will maintain, based on which direction will be easier to make the initial turn for.

Proportional control is done with the angle that the robot is away from travelling parallel with the nearby wall. This nearby wall is determined by finding the closest object on the LiDAR. The robot has a bias towards turning away from the wall, as this is preferred to potentially running into the wall.

I also check for the distance of objects in front of the robot scan, even if they are not the nearest objects. This is because I can then start turning the robot before it runs into the oncoming wall. This turn overrides other proportional control, so if there is an object too close in front of the robot it will stop going forward and start turning regardless of what is happening to the wall it was previously following (This allows it to then transition to following the new wall after it makes the turn).

![Robot](https://github.com/Loonride/warmup_project/raw/main/gifs/wall_follower.gif)

# Challenges

The challenges I ran into while making the Square Drive behavior were working with current robot angles, as well as handling the latency of commands so that I can point the robot in the desired direction. I used the `odom` approach for that problem, and I overcame these challenges by making slight angle adjustments as the robot drove forwards to make the edges of the square, as well as making many angle helper functions to determine the best way to turn. Making the Person Follower was quite straightforward, and worked well after iterating and finding good proportional control parameters. The Wall Follower was the most challenging, as it required driving parallel to an object, as well as handling a corner that transitions to a new wall. I overcame this challenge by breaking the problem down and getting parallel wall driving working first, before then adding on wall turn handling.

# Future Work

For my Square Drive behavior, I realized later that a proportional control strategy would combine very well with setting angular speeds based on the `odom` readings. I would implement this to allow quick, 90 degree turns that hit the target angle well, as my constant angular speed approach was not ideal. For the Person Follower, I might add improvements to ensure that if the person gets too close, the robot would back away from the person. With the Wall Follower, I would want to add improvements to make the robot work in rooms that are not square. Currently, my robot would likely lose the wall if it was handling an outer corner rather than an inner corner.

# Takeaways

- Proportional control. Whenever possible, proportional control should be used to ensure the robot does not overshoot targets. This is also useful for accomplishing tasks faster with good accuracy.
- Focus on angles. The first thing to figure out is always getting a sense of direction with the angles your robot should form in the world or with other objects. This is important because driving forward is useless unless you first orient the robot in the direction you desire, or alternatively figure out how to orient the robot as it is driving forward. Included in this is also figuring out how to handle when the robot overshoots an angle, which comes back to doing proportional control. 

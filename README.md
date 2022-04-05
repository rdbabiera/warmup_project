# warmup_project
## drive_square
### Description:
Though self-explanatory, the goal of the drive_square script is to cause 
TurtleBot to drive in a square. The approach used to solve this problem 
consisted of publishing commands for TurtleBot with specific timings, such that 
it would always move forward for the same amount of time for each side of a 
square, and that it would always turn 90 degrees each time.

### Code Overview:
```python
class DriveSquare(object):
    def __init__(self, duration, linear)

    def stop(self)

    def forward(self)

    def turn(self)

    def run(self):

```
The drive_square.py script is divided into the modules listed above. Init takes 
a duration (time to turn 90 degrees, in seconds) and a linear speed (m/s) as 
inputs. The init function initalizes the rospy node 'drive_square', creates a 
publisher for /cmd_vel, and then sleeps for a second in order to warm up the 
turtlebot and prevent packet dropping.

stop(), forward(), and turn() are helper functions that move the robot. Stop 
initializes a blank Twist object and publishes it to the robot, forward sends 
a Twist object with no angular velocity and with the linear speed in init, and 
turn publishes a Twist() object with no linear velocity but an angular-z velocity 
of $\frac{\pi}{2}$ / duration.

Lastly, run combines these helper functions to drive turtlebot in a square. 
Messages are sent at a rate of 10Hz using rospy.Rate(10). In a for-loop that 
runs four times, TurtleBot moves forward for 6 seconds, and then turns for the 
turn duration. These timings are done using inner for-loops that send commands 
over some x amount of times, such that the code runs for $\frac{x}{10}$ seconds. 
An bare-bones example of moving fowards is given below:

```python
def run(self):
    r = rospy.Rate(10)
    for i in range(0, 4):
        for j in range(0, x):
            self.forward()
            r.sleep()
        ...
```
At the end of running the outermost for loop, a stop() command is issued and the 
function exits. The script makes a call to initializing the object and calling 
run() once.

### Demonstration:


## person_follower
### Description:

### Code Overview:

### Demonstration:

## wall_follower
### Description:

### Code Overview:

### Demonstration:

## Challenges

## Future Work

## Takeaways
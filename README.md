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
![](https://github.com/rdbabiera/warmup_project/blob/main/gifs/drive-square.gif)

## person_follower
### Description:
The purpose of person_follower is to cause TurtleBot to follow a person, and then 
stop when at a set distance from the person. Within the set distance, TurtleBot 
turns at a set angular velocity towards the direction the closest object is facing 
until it is within a 90 degree cone facing TurtleBot.

In order to approach this problem, two scenarios were considered - the one in 
which the robot is within the stopping distance, and the generalized case where 
it is not. For this second case, it made sense that the robot would have some 
proportional control based off of difference between the closest angle and 
moving straight. This can be generalized as:

angular velocity = (state - error) * c = (angle - straight) * constant.

In my code, I normalized the error on a scale from one to zero before multiplying 
by a constant.Proportional control is also used on the linear 
velocity depending on the angle error to zero, as a high angular and linear 
velocity causes TurtleBot to move backwards when right behind the person. Thus, 
linear velocity is minimized when TurtleBot needs to make sharper turns, and this 
is generalized below:

linear velocity = (state - error) * c = (-angle + backwards) * constant

### Code Overview:
```python
class PersonFollower(object):
    def __init__(self):
    
    def process_scan(self, data):

    def get_directions(self):

    def run(self):
```

The person follower implementation is divided into the subroutines listed above. 
The init function intializes the rospy node 'person_follower', creates a subscriber 
to /scan and a publisher to /cmd_vel, intializes robot state variables 
'self.closest_index' and 'self.closest_range', and sets up static variables for max 
range, stopping distance, and linear velocity; these values are 3.5m, 0.4m, and 0.15m/s 
respectively. Lastly, it sets rospy to sleep for 2 seconds in order to allow 
spinup time.

process_scan() is passed to the subscriber, and runs every time data is retrieved. 
A local range and index are defaulted to 4 and 0 so that if nothing is scanned by 
the robot, then default behavior will be performed. A for loop over all ranges is 
then performed. If the current range is the smallest seen so far and is not 
zero (no scan data), then the local range and index variables update. Lastly, 
the state variables self.closest_index and self.closest_range are updated using 
the local values.

get_directions() first intializes a Twist() command for the robot. If the current 
closest range given by the state variable is 4 (default/nothing in range), the 
robot will do nothing. Otherwise, there are two separate conditionals to be 
taken care of. In the first, if the robot is within the stopping distance, it 
spins clockwise or counterclockwise with angular magnitude 0.785 rad/s towards 
the closest object. If the closest object is further than the stopping distance, 
and TurtleBot sees the closest object within a 30 degree cone, it will move 
straight with velocity self.linear_vel. Otherwise, on the left side of the person 
[14, 180), TurtleBot has angular velocity (closest index / 180) * 1.5 and linear velocity 
((-self.closest_index + 179) / 180) * self.linear_vel. This makes TurtleBot turn 
faster towards the target when directly behind, and the linear velocity is set 
such that this turn is done properly. When facing the target, linear velocity 
is higher. On the right side [180, 345), angular velocity is (self.closest_index - 360) / 180 
and linear velocity is ((self.closest_index - 180) / 180) * self.linear_vel. This 
performs the same functionality, as all angles within the range will become negative, 
and having greatest magnitude at 180.

Lastly, run() sets a rate of 10Hz, and constantly spins to grab directions using 
get_directions(). Each command is then run twice at the set rate.

### Demonstration:
![](https://github.com/rdbabiera/warmup_project/blob/main/gifs/person_follower.gif)

## wall_follower
### Description:
wall_follower is a script that causes TurtleBot to follow a wall parallel to it 
and move forward if a wall cannot be found. This script maintains a safe distance 
to the wall at all times, with the only drawback being that Turtlebot can only 
follow the wall in one direction - TurtleBot keeps the wall on its left side 
at all times.

The goal to solve this problem is to establish proportional control for angular 
velocity. If parallel to the wall, TurtleBot should have the closest angle be at 
90 degrees at all time (left side). If the closest angle is less than that, then 
TurtleBot may be approaching an inside corner and needs to turn away. However, when 
approaching an outside corner, the closest angle slowly shifts towards greater 
than 90 degrees and TurtleBot needs to turn inward, 'hooking' the corner until 
it perfectly rounds it. This error can then be established as such:

angular velocity = (angle - 89) * c

The one drawback to this is that turtlebot will generally only turn in one direction, 
which can result in 270 degree spins when facing a wall instead of 90 degree turns.

### Code Overview:
```python
class WallFollower(object):
    def __init__(self):

    def update_directions(self, angles):

    def update_control(self):

    def process_scan(self, data):

    def run(self):
```

The wall following routine is divided into the subroutines above. init() creates 
a /scan subscriber and a /cmd_vel publisher, sets the state variables 
'self.closest_index' and 'self.closest_range' as well as static variables for 
max range, safe distance, safe distance buffer, and constant linear velocity, 
and lastly sleeps for 2 seconds for warmup time.

process_scan() is the function passed to the /scan subscriber. Upon receiving the 
LiDAR scan data, it passes the angles to update_directions() and then upon return 
makes a call to update_control(). The run() function causes TurtleBot to spin, 
as 

update_directions() sets the local closest range and index at 4m and 0 respectively, 
as well as a variable, increment, that controls how wide of a spread to be considered 
when getting the range for each angle; here it is set to 2, corresponding to a 
5-degree spread. Similar to person_follower, the smallest index and the corresponding 
range get saved to the robot states in this function, and nothing is returned.

update_control() creates a Twist() object. If nothing is within range of the robot, 
the robot will move forward unconditionally. The error vector of the closest angle 
is calculated as (angle - 89), corresponding to the difference between the 
closest object and a vector pointing left out of the LiDAR scanner. Two cases are 
considered for setting the angular velocity of the robot. The first is when the robot 
is outside of the safe distance's buffer region. Here, angular velocity is set to 
the range of the closest object to the robot minus the safe distance, meaning the 
robot turns left inward when too far and right outward when too close. This error 
is multiplied by the constant 0.03. This only runs when the robot is within a 7 
degree spread of parallel to the wall. The second case captures all other scenarios, 
in which the angular error stated above is multiplied by 0.03. When the error is 
negative, TurtleBot turns away from the corner (clockwise) assuming the corner is 
inside, however when positive TurtleBot turns counterclockwise towards bends. 
This command is then published once.

### Demonstration:
![](https://github.com/rdbabiera/warmup_project/blob/main/gifs/wall_follower_1.gif)
![](https://github.com/rdbabiera/warmup_project/blob/main/gifs/wall_follower_2.gif)

## Challenges
Some of the challenges faced in the project can be divided into timing and proportional control. 
For drive_square, the most important aspect of the task was to have constant 
timing on all turns and movements. However, my first attempts consisted of sending 
commands once and sleeping for a set amount of time, which caused many inconsistencies 
in when commands would run and for how long. I then learned that commands can be 
sent at a rate a set amount of times in order to emulate running that command for 
a predetermined time, and this fixed all problems. 

For person_follower and wall_follower, the main problem related to proportional 
control as it pertained to turning in different directions. For person follower, 
this meant dividing the robot into two halves, and chosing the correct error 
function for each side. Until the last day there were problems for the range 
[180, 360) however realizing that subtracting by 360 would guarantee that 
a negative angular velocity would be chosen and that it would be fastest 
behind the robot. For wall follower, generalizing behavior to follow the wall 
in both directions presented a harder challenge, however realizing that TurtleBot 
could follow in one direction simplified the task greatly.

## Future Work
Given more time, there are a few changes and possible alternatives I would like to 
try. For drive_square, it may be interesting to have conditional turns based off 
of displacement rather than timing in order to implement a more sensory motor 
approach for the task. For person follower, one optimization I would like to 
implement is one in which the robot keeps track of where the person was last sighted, 
so it continues to follow the same object if a wall or another person gets closer 
than the person. For wall_follow, I would like to add the ability to drive in both 
directions rather than in only one, similar to how person follower can turn in 
both directions.

## Takeaways
1. Many state to behavior problems can be broken down into simple proportional 
control equations. The main challenge in person follower was getting right side 
of the robot to interpret how fast to turn/evaluating what the error function would 
be. After diagraming the range circle, this slowly became more clear. Diagramming 
vectors was also beneficial in diagramming angular velocity relative to robot 
position in wall follower.
2. Don't be scared of the ROS learning curve! Example code can give lots of 
insight to best practice, as repeating messages is more reliable than sending 
a message once. Abstracting publishers and subscribers becomes much clearer 
after high level introduction, and organizing states becomes much easier.
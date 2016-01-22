# grasp
# GMAR grasping exercise
download this folder in your src folder in workspace which is containing baxter and movIt files
# starting Baxter in Simulation

Copy this folder to your workspace which is containing moveIt and baxter files

Open terminal
Terminal commands to start Baxter in Simulator
Exectute one by one:


```bash 
$ cd gmar_ws/
$ source devel/setup.bash
$ ./baxter.sh sim
$ roslaunch baxter_gazebo baxter_world.launch
```

--------------------------------------------------
# starting MoveIt
Open new terminal
Exectute one by one:


```bash 
$ cd gmar_ws/
$ ./baxter.sh sim
$ source devel/setup.bash
$ rosrun baxter_tools enable_robot.py -e
$ roslaunch baxter_moveit_config baxter_grippers.launch
```
------------------------------------------------------

# Baxter Collision Scene
 
IMPORTANT:
Never execute a plan without the appropriate collision domain!!

Copy and Paste the file baxter_table.scene into your workspace

In the Motion Planning window select the "Scene Objects" tab.
Press "Import From Text" and select the baxter_table.scene you copied before.

Go to the "Context" tab and press "Publish Current Scene".
Now MoveIt has the table in it's collision domain.

In the window displays select "Motion Planning" - "Planning Request". In the "Planning Group"
select the arm you want to plan.

Now you are ready to plan.
Please keep in mind that before every planning request you have to set the update the start state
to the current state.

# Grocery Restocker
The task we will be asking our robot to perform is restocking grocery shelves. The robot will check each of the cells on a shelf to determine which ones require a product. Furthermore, depending on which cell is empty, the robot will be able to determine which product belongs on that shelf, retrieve it from the crate, and place it in the appropriate spot. This is demonstrated by the diagram below.

## To Run the Codes
- Clone a local repo on your VM. Follow the instructions on downloading the VM at https://uofi.app.box.com/s/3sqaarqs2zuniultzyc0ztz7yg4cn25h. There should also be an email sent out on this.
- When you clone the repo the local repo name will be "ECE470-project". Remember to change it to "project", otherwise when you ```catkin_make``` it won't be able to recognize the path.
- Instead of ```rosrun lab2pkg_py lab2_exec.py```, do ```rosrun grocery_py grocery_exec.py``` to run the program. 
- When you run ```roslaunch ur3_driver ur3_gazebo_lauch``` it now spawns the world with the shelf model.  

## TODO (as of 11/3/2022):
- Create the objects (I already created a shelf model). I can working on creating the objects (Mike). For now just use the blocks given. 
- Work on the codes so that the manipulator can perform a pick and place on an object (this part should already be done because I am using the move_block code from lab2). But test to see if works.
- For each position of the shelf, we nedd a corresponding set of joint values, similar to task 2. However, I am not super sure how this part should be done considering that we can't manually move the manipulator in simulation as we did with the physical manipulator.

#Temporary Update 11/18/2022 (Dylan):
I modified grocery_exec.py to pick up a red block, and place it on top of the green block. The code is imperfect, capable of picking up the block but not setting it down gently. 
How to run:
- Before running ```rosrun grocery_py grocery_exec.py``` you should run ```rosrun grocery_py grocery_spawn.py```
	- choose position 1 when prompted
- When running ```rosrun grocery_py grocery_exec.py``` it will prompt for a loop count. Choose any nonzero option to run the code once. (I commented out the loop example movement code)


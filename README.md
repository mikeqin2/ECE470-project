# Grocery Restocker
The task we will be asking our robot to perform is restocking grocery shelves. The robot will check each of the cells on a shelf to determine which ones require a product. Furthermore, depending on which cell is empty, the robot will be able to determine which product belongs on that shelf, retrieve it from the supply, and place it in the appropriate spot. 

## To Run the Codes
- Clone a local repo on your VM. Follow the instructions on downloading the VM at https://uofi.app.box.com/s/3sqaarqs2zuniultzyc0ztz7yg4cn25h. There should also be an email sent out on this.
- When you clone the repo the local repo name will be "ECE470-project". Remember to change it to "project", otherwise when you ```catkin_make``` it won't be able to recognize the path.
- Instead of ```rosrun lab2pkg_py lab2_exec.py```, do ```rosrun grocery_py grocery_exec.py``` to run the program. 
- Open terminal and source the ```project``` folder by entering ```source devel/setup.bash```
- Run ```roslaunch ur3_driver ur3_gazebo.launch``` to run the gazebo simulator and spawn the world with the shelf model.  
- In a separate terminal, source the ```project``` folder and enter ```rosrun grocery_py grocery_spawn1.py``` to spawn the supply products, represented by blocks. Enter ```n``` when prompted. 
- In the same terminal, enter ```rosrun grocery_py grocery_exec.py``` to run the program. 


## Update 12/06/2022: 
Because of the limitations of the physical robot and the inverse kinematics solutions, the robot will only stock a 3x3 grid of cells on the shelf. This grid is the +y+z most 3x3 grid. The robot will search each cell using the vacuum gripper input, and if it does not detect a product then it will pick up a product from its supply. The supply is represented by a similar 3x3 array of colored blocks. In practical application, the supply and the shelf can be assumed to be of a known position relative to the robot (once an additional robot-mobility program is added, which will allow the robot to roam the store). 



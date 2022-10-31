# Grocery Restocker
The task we will be asking our robot to perform is restocking grocery shelves. The robot will check each of the cells on a shelf to determine which ones require a product. Furthermore, depending on which cell is empty, the robot will be able to determine which product belongs on that shelf, retrieve it from the crate, and place it in the appropriate spot. This is demonstrated by the diagram below.
TODO:
- Create the objects (we need to create the grocery models, a grocery shelf, and perhaps a container for the grocery where you grab the grocery from). 
- Spawn objects (refer to the lab2 document sent in the email).
- Grab the objects (refer to lab2 code)

## To Run the Codes
- Clone a local repo on your VM. Follow the instructions on downloading the VM at https://uofi.app.box.com/s/3sqaarqs2zuniultzyc0ztz7yg4cn25h. There should also be an email sent out on this.
- When you clone the repo the local repo name will be "ECE470-project". Remember to change it to "project", otherwise when you ```catkin_make``` it won't be able to recognize the path.
- Instead of ```rosrun lab2pkg_py lab2_exec.py```, do ```rosrun grocery_py grocery_exec.py``` to run the program. 

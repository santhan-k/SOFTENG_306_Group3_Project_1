SOFTENG_306_Group3_Project_1
============================

Documentation - Contains project related documents(Planning etc.)

Old - Contains old packages

src - contains the package (alpha_two) which is the latest working copy.

Instructions for Setting up a workspace and running the simulation.
===================================================================

1. Navigate into src directory and run "catkin_init_workpace"
2. Navigate back to the trunk directory and run "catkin_make" to compile the workspace
3. You might need to comment out R0, R1, grass, cloud, sheepDog and farm form Cmakelist.txt. (header files for messages have not been compiled yet)
4. Uncomment the lines that you commented in step 3 and run catkin_make again
5. Source this workspace by adding it to bashrc (source [pathtoyourworkspace]/devel/setup.bash>
6. Navigate to the world directory (roscd alpha_two/world)
7. run the python generator script to start simulation (python pygen.py [num of sheep] [Field number (1-4)])
8. Run kill.py to kill the simulation.



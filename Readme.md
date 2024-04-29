# A simple implemenatation of the TagSlam based on GTSAM 
GitHub setup

1. Create the token in Github webpage, activate the repo option
2. Install Git's Credential Helper (to avoid typing password repetitively) 

		sudo apt-get install libsecret-1-0 libsecret-1-dev
		cd /usr/share/doc/git/contrib/credential/libsecret
		sudo make
		git config --global credential.helper /usr/share/doc/git/contrib/credential/libsecret/git-credential-libsecret

4. Create a repo on Github webpage
5. prepare local repo:

		
		git init
		git add .
		git commit -m "InitialCommit"
		git remote add origin http://......
		git push -u origin main (--force)
		

Here they will ask for username and password, use token as your password instead of github password
#### Other useful commands
		1. Rename branch: git branch -m master main
		2. Switch branch: git checkout main
		3. Merge changes: git merge other-branch-name
		4. Fetch changes: git fetch origin
		5. Pull changes: git pull origin main


Build the project: 
1. Create the work directory:
```
cd catkin/src
catkin_create_pkg AprilSlamCPP roscpp std_msgs tf2_ros nav_msgs
cd AprilSlamCPP/src
code ..
```

3. Modify cmake file:
    
4. Modify XML
	


Python instruction:

```	
sudo apt install python3-pip
pip install gtsam
```

make the python script executable
```chmod +x /home/shuoyuan/catkin_ws/src/april_slam/odomGTSAMTest.py```

If error, add #endif to the header when compiling gtsam (std_optional_serialization.h)




Some Remarks: 

1. error: ‘optional’ in namespace ‘std’ does not name a template type
	std::optional is c++17 only, add this line to your cmake file:

```set(CMAKE_CXX_STANDARD 17)```

2. error: static assertion failed: Error: GTSAM was built against a different version of Eigen

	need to rebuild:
```cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..```

4. error: gtsam_wrapper.mexa64 unable to find libgtsam.so.4

This is due to the fact that the default search directory of gtsam_wrapper.mexa64 is /usr/lib/ yet all related libs are installed to /usr/local/lib. All corresponding files (the ones mentioned in Matlab error message) needs to be copied to /usr/lib/

```
sudo cp /usr/local/lib/libgtsam.so.4 /usr/lib/
sudo cp /usr/local/lib/libgtsam.so.4 /usr/lib/
```
		
4. Matlab toolbox: cmake -D GTSAM_INSTALL_MATLAB_TOOLBOX=1 ..
	copy the toolbox from usr/local/ to work directory, then add the folder to path in Matlab

5. To hide "Warning: TF_REPEATED_DATA ignoring data with redundant timestamp" error in terminal
```
source devel/setup.bash
rosrun AprilSlamCPP AprilSlamCPP 2> >(grep -v TF_REPEATED_DATA buffer_core)
rosbag play --pause rerecord_3_HDL.bag
```
6. Compile:
```
catkin_make --pkg AprilSlamCPP
```















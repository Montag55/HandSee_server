# HandSee_server
Clone *opencv calib* (https://github.com/opencv/opencv_contrib), navigate to *./modules* and delete everything exept for the *ximgproc* folder.
Then build opencv again but add external module path to configuration. For this navigate to your opencv directory and run:
'''
$ cd build
$ cmake .. -DOPENCV_EXTRA_MODULES_PATH=<path to modules folder from justz cloned opencv_contrib>/modules
$ make 
$ make -j4
'''
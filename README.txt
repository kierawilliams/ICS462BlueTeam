ICS 462, AI for Games
Programming Assignment 3 Solution with modified version of Millington's AI for Games's decision tree code (see LICENSE file for the use license for that original code)

The current version of BZFlag does not allow robots to pick up/drop flags. Download the fixes in robotFlags-2.4.14-v2.zip (http://www2.hawaii.edu/~chin/462/Assignments/robotFlags-2.4.14-v2.zip), unzip it and replace the corresponding files in your src and include directory with these files. You will have to quit bzfs and rebuild it. Likewise with bzflag.

To compile, copy the files RobotPlayer.h, RobotPlayer.cxx, AStarNode.h, AStarNode.cxx, Astar.h, Astar.cxx, dectree.h, and dectree.cxx to your bzflags-2.4.14\bzflag\src\bzflag folder (overwriting the originals of RobotPlayer.h and RobotPlayer.cxx).  Then add AStarNode.h to the bzflag project by right clicking bzflag, selecting "Add > Existing Item" and then find AStarNode.h in your src\bzflag folder.  Likewise add AStarNode.cxx, Astar.h, Astar.cxx, dectree.cxx and dectree.h to the bzflag project. Then build bzflag in Microsoft Visual Studio as usual.

For UNIX-based systems copy the same files to your bzflags-2.4.14/src/bzflag directory.  You will have to edit src/bzflag/Makefile.am to add AStarNode.h, AStarNode.cxx, Astar.h, Astar.cxx, dectree.cxx and dectree.h to bzflag_SOURCES.
Depending on your OS and compiler, you may have to convert the line endings using a program like dos2unix. Next run autogen.sh and configure again.  Finally compile as usual.
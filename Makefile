##
# Robot Car
#
# @file
# @version 0.1
source = source ~/project/env/bin/activate &&

robot: robot.py
	${source} python3 robot.py

line_following: line_following.py
	${source} python3 line_following.py

shape_detection: shape_detection.py
	${source} python3 shape_detection.py
# end

 First, change the speed and the sleep time in code to make sure the car turn the same angle.
 Second, put the object that you want to go straight to in front of the car and make sure the back ground can not affect the line detection.
 Third, put April tag at the position that the camera can see when the car run around the object and turn back.
 Fourth, after the car stop in the front of tag, you can "cd /final/final_py" and "sudo python3 car_control.py /dev/ttyUSB0" to start Xbee python control.
 Fifth, input the information of region, director, and parallel and vertical distance just like HW4_Part1, the car will park to the position you desire.
 
 Note.
 Sometimes the camera can not detect the tag when the turn back.
 Line detection to object is easy affected by the background information.
 The picture of object and tag position is attached. 

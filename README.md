# Recognize OK Gesture

**OK gesture is recognized on the following conditions**
1. 3rd, 4th, 5th fingers are pointing roughly in the same direction
   - Compare finger vectors
2. 3rd, 4th, 5th fingers' are between slightly bent to straight
   - Compare Euclidian distance vs Joint distance
3. 3rd, 4th, 5th fingers are roughly inline with palm
4. 1st & 2nd finger distance is almost Zero
   - Leap Motion SDKs
5. 1st and 2nd finger joints are almost coplanar
   - Maybe difficult to do?
6. 1st and 2nd finger joints resemble a circle
   - https://github.com/AlliedToasters/circle-fit



Follow hand

Get absolute position of human hand using Leap motion and UR Robot Arm telemetry
Make Arm follow position?

Follow hand WHILE hand is making a pointing gesture
Collect time-series data of index finger position and direction it is pointing
When hand is not making a pointing gesture anymore, retrace the index finger position and pointing with the RGBD Camera
Return to home position

Recognize pointing gesture 
1. 2nd finger straight
2. All other joints are quite close to each other
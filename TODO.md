TODO LIST - CVML Project
========================


-1. Some information we observed
--------------------------------
- Maximum safe speed inside lane ~ 1400
- Maximum safe speed outside lane ~ 1700
- PieceID = ID of the track piece the vehicle is on (8 pieces)
- LocalisationID = ID of the horizontal "dash" the vehicle is on


0. I/O management
-----------------
- Wait for "connection successful" before inputing new commands [Done]
- Get the vehicle's location as a return variable instead of printed on screen []
----> using Georg's code instead


1. Basic features, using only position (Goal: 1 car, 10 laps, optimize time)
----------------------------------------------------------------------------
- Detect if vehicle is going in the wrong direction -> Uturn
- Prevent the vehicle from going outside the red borders
- Create a function that returns the lane and track piece given the localisation informatin output by the vehicle
- Detect if the vehicle is out of the road (no localisation information) and try to set random speed to get back in track
- Detect if vehicle gets disconnected and reconnect
- Create a policy function that set the vehicle's speed/lane depending on its position


2. Adding Computer Vision (CV) and Machine Learning (ML) 
--------------------------------------------------------
- (CV) Detect where the turns and straight parts of the track are
- (CV) Detect vehicles' positions (background substraction)
- (CV) Detect red borders/limits of the track
- (ML) Implement Reinforcement Learning/Q-Learning for the situation with only one car to get the best policy


3. With opponents...
---------------------
- Think about good strategies when you have opponents
- (ML) Try to adapt the learning algorithm to a situation where you have 1 or multiple opponents
- (CV) Keep track of your other opponent's position/distance from the goal

4. And a new track!
-------------------
- Hopefully hat we implemented for (CV) and (ML) should apply well to the new track
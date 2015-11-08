TODO LIST - CVML Project
========================

RED Car (orange...)

-1. Some information we observed
--------------------------------
- Maximum safe speed inside lane ~ 1400
- Maximum safe speed outside lane ~ 1700
- PieceID = ID of the track piece the vehicle is on (8 pieces)
- LocalisationID = ID of the horizontal "dash" the vehicle is on

0. Main
-------
Build script: ./build.sh
Run: ./build/dist/bin/main [name] [adapter] [verbose]
where:
 - [name] is the color (eg grey) or name of the vehicle (eg boson)
 - [adapter] is the bluetooth adapter (optionaal, defaults to hci0)
 - [verbose] increases verbosity output


1. Basic features, using only position (Goal: 1 car, 10 laps, optimize time)
----------------------------------------------------------------------------
- Detect if vehicle is going in the wrong direction -> Uturn [Done]
- Prevent the vehicle from going outside the red borders
- Create a function that returns the lane and track piece given the localisation information output by the vehicle
- Detect if the vehicle is out of the road (no localisation information) and try to set random speed to get back in track [Done]
- Detect if vehicle gets disconnected and reconnect
- Create a policy function that set the vehicle's speed/lane depending on its position


2. Adding Computer Vision (CV) and Machine Learning (ML) 
--------------------------------------------------------
- (CV) Get default background image				[Done (Multithread)]
- (CV) Detect where the turns and straight parts of the track are
- (CV) Detect vehicles' positions (background substraction)   [Done]
- (CV) Detect red borders/limits of the track
- (CV) Detect color of the car using hsv space   [Done with saturation/value thresholding]
- (ML) Implement Reinforcement Learning/Q-Learning for the situation with only one car to get the best policy


3. With opponents...
---------------------
- Think about good strategies when you have opponents
- (ML) Try to adapt the learning algorithm to a situation where you have 1 or multiple opponents
- (CV) Keep track of your other opponent's position/distance from the goal

4. And a new track!
-------------------
- Hopefully hat we implemented for (CV) and (ML) should apply well to the new track


5. Less important tasks / Finetuning
-------------------------------------
- (CV) Apply some kind of normalization in high lights zone to detect ligh cars (orange, yellow)
- (CV) Finetune parameters of blob detection for different colors (eg bad case = red and grey)
- Totest: init background as current picture (less waiting time)

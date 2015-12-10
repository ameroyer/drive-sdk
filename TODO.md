TODO LIST - CVML Project (Rho)
==============================


Next Week
---------
- (TODO) Update the Github
- Laptime function [Done (for this track)]
- (TODO) Create optimal deterministic policy for one car
- (TODO) Start the Machine Learning process
- Meet in the demo room

-1. Some information we observed
--------------------------------
- Maximum safe speed inside lane ~ 1400
- Maximum safe speed outside lane ~ 1700
- PieceID = ID of the track piece the vehicle is on (8 pieces)
- LocalisationID = ID of the horizontal "dash" the vehicle is on
- 5/10 commands a second seems to be the botlleneck for the C interface (set update period accordingly)

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
- Detect if the vehicle is out of the road (no localisation information) and try to set random speed to get back in track [Done]
- Prevent the vehicle from going outside the red borders
- Detect if vehicle gets disconnected and reconnect
- Control several cars
- Add human interface


2. Adding Computer Vision (CV) and Machine Learning (ML) 
--------------------------------------------------------
- (CV) Get default background image				[Done (Multithread)]
- (CV) Detect vehicles' positions (background substraction).   [Done]
- (CV) Detect red borders/limits of the track	  		[Done]
- (CV) Detect color of the car using hsv space   [Done with saturation/value thresholding]
- (CV) Take a picture as the start of the race for default background (better color match)
- (CV) Partition the track   [Done]
- (CV) Detect curvatures of the track

- (ML) Create a policy function that set the vehicle's speed/lane depending on its position
- (ML) Implement Reinforcement Learning/Q-Learning for the situation with only one car to get the best policy
- (ML) Create main q-learning algorithm structure

Questions:
Why is the car try to change to inside so much? random behaviour, equal for going outside?
Problem with (output of) runs? set speed 1500, then state speed still 1000...
How to define rewards?


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

# Planty-Bot-v3-ROS

Demo video: https://drive.google.com/file/d/13Zs-4b7OKjKfE2LTI1eZ02aVVR63W-fj/view?usp=sharing

DESIGN CRITERIA:
Use computer vision to detect an AR tag and identify it as the final location (the plant pot)
Use LIDAR sensor to create a shortest path to the pot while avoiding all obstacles based on the sensor inputs
Stop and turn on water pump when reach desired location

BASE ROBOT CHOICE:
We decided to use Turtlebot because it is readily available, has all sensors for collecting needed data, and can handle required movement. Our design choices were actually guided by the setup of the TurtleBot provided for us because of the way the LiDAR sensor and Raspi camera are placed. Since the TurtleBot is only able to move in a 2D plane, we had to reconstruct our product and dependencies to only adhere to the x-y plane.

![Turtlebot](https://user-images.githubusercontent.com/80079738/218007202-d6341251-2f68-47ee-8b7a-b9adc66e207b.png)

SENSOR CHOICE:
Since we want the product to be as modular as possible, we decided to use the built-in Raspi camera instead of an external camera as introduced in class. Doing so can help avoid wiring problems and the robot can be free from environmental dependencies (us holding the external camera or the robot can only move within the camera range if it is mounted up). This built-in camera, however, has very low resolution, so we have to use bigger AR tags as well as be mindful to not put the robot too far from the tags since it needs to detect the AR tags in order to start moving. This creates another problem where having the Turtlebot too close to the pot moves the AR tags out of the camera frame.

We decided to employ laser detection from the LiDAR sensor to detect objects in front of the Turtlebot instead of using computer vision because it is more simple and straightforward. Since we only need data of how far objects are from the robot, by doing so we can cut down on the bulk of unnecessary work and focus on perfecting the robot movement.

PHYSICAL DESIGN CHOICE:
Since the sensors are in fixed positions, we have to be very mindful in designing the physical outer layer that holds the water bottle and the tube so that it does not obstruct any of the sensors. We decided to make 2 water bottle holders in the back so that the water weight can be evenly distributed. The front is made solid with only an opening for the camera so that we can prevent any water splashing into the robot boards due to the open form factor. The water tube is designed to be as low as possible in order to stay clear from the LiDAR sensor, while still ensuring that the water can be dispensed at the middle of the robot since we program the robot to hit the pot (AR tag) head on. This brings the problem of the water tube being shorter than the AR tag, which hinders the watering process. This can be fixed by making new AR tags with suitable height and recalibrating the camera.

![CAD_original](https://user-images.githubusercontent.com/80079738/218007274-d7c3f793-62cd-48d0-987a-3abd4fdd4cd1.png)

SOFTWARE
The software we wrote consisted of two main components: first, we have a ROS node that takes in LIDAR sensor data and filters it so that we only retain the spatial information pertaining to the regions of interest, these being the front, left, and right sides of the Turtlebot. The second node we wrote was the main driver node, which contained all of the logic related to searching for AR tags, driving to them, and used the LIDAR information to perform the obstacle detection algorithm. It uses the ar_track_alvar package to detect AR tags. The ROS graph of our system is shown below:

![ros_graph](https://user-images.githubusercontent.com/80079738/218007826-ba95c720-6eb5-41e3-8113-d25f9450e4cd.jpeg)

And here is what our LIDAR data and AR tracking data looks like in rviz:

![LiDar sensor data](https://user-images.githubusercontent.com/80079738/218007863-7537b34c-43bc-4ee8-937f-4577385d5f0e.png)

![AR tracking data](https://user-images.githubusercontent.com/80079738/218007885-f55b2f25-7679-4385-a60c-5b123c0dbeac.png)

FULL SYSTEM OPERATION
First, we need to initialize the camera. By default, the camera outputs a compressed image. To rectify the image, we need to publish our camera’s image as a raw image, thus we feed the compressed image into the image_transport node. Then we rectify the image using the image_proc node. Once we have the camera set up, the Turtlebot will rotate in place searching for the first AR tag to drive to. When it finds the AR tag, it will turn in the direction of it, and begin driving towards it. To account for noisy measurements, imperfections in the robot’s motors, and other external factors, we implemented a feedback control system so that the Turtlebot is able to correct any deviation from its path during driving. If it detects any obstacles in its way while driving (using LIDAR data from the front of the robot only) it will enter an obstacle avoidance sequence.

In the obstacle avoidance sequence, it will use LIDAR data to determine open space around the obstacle. It will then determine whether to move to the left or right of the obstacle. If there is space on both sides, it will determine whether the left or right space is closer to the Turtlebot. Once a direction is chosen it will turn in that direction and continue moving until it can no longer detect the object in its path using LIDAR data from the left/right sides (depending on which direction it's moving). Then it will turn back in the direction of the AR tag and continue moving forward. If at any point during this sequence it detects another obstacle, it will perform the same algorithm again.

Finally, once the Turtlebot reaches the plant pot, it will stop for some period of time to water the plant (the watering was not implemented in this project), then reverse and look for the next AR tag to drive to. The process is repeated until all the specified plants have been watered.

Overall, our system is modeled and implemented as a finite state machine which is illustrated below:

![fsm1](https://user-images.githubusercontent.com/80079738/218007974-f2186afe-b397-44b9-ad81-90cdacffebef.jpeg)



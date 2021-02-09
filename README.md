# digit_sim
Pybullet simulation package for Digit robot.

This digit simulation can perform both path planning and arm motion planning using Caelan Garrett's [pybullet_planning](https://pybullet-planning.readthedocs.io/en/latest/) package. It also uses [IKFast](http://openrave.org/docs/0.8.0/openravepy/ikfast/) as the inverse kinematics solver for Digit's arms. 

In my bid to skirt Digit's tedious bipedial locomotion trouble, I made the joints in Digit's legs fixed joints and mounted it on a hoverboard for good measure.


## Installation
It is recommended that you install this package in a conda environment. Here's how to create one: [Create a conda environment](https://docs.conda.io/projects/conda/en/4.6.0/_downloads/52a95608c49671267e40c689e0bc00ca/conda-cheatsheet.pdf). Python3.6 is recommended.

To install dependencies,  
1. Clone this repo

	`$ git clone https://github.com/alphonsusadubredu/digit_sim.git`

2. Install dependencies

	`$ pip install -r requirements.txt`


## Usage
To run navigation example
 
	$ cd examples 
	$ python navigation_example.py 

Digit moving in 'circles'

![](navigation.gif)


To run manipulation example
 
	$ cd examples
	$ python manipulation_example.py 

Digit picking up a 'coke' can

![](manipulation.gif)
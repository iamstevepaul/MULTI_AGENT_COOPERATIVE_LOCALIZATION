from Environment import Environment
from Robot import Robot
import numpy as np
import random

max_rectangle = 100
min_rectangle = 0
corners = [[min_rectangle,min_rectangle], [min_rectangle,max_rectangle], [100, min_rectangle], [max_rectangle,max_rectangle]]
corners = np.array(corners)

n_landmarks = 10

landmarks = np.array([[random.random(), random.random()] for i in range(n_landmarks)])*max_rectangle











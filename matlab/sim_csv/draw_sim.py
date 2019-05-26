import cv2
import numpy as np
import math
import os
import pandas as pd

height = 400
width = 800
wheel_r = 50
pole_l = 150

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('simulation.mp4', fourcc, 1000.0, (width, height))

def draw_state(phi, theta):
    image = np.zeros((height, width, 3), np.uint8)
    image[:, :] = (255, 255, 255)
    floor = height-50
    pos = (int(theta/math.pi*wheel_r+width/2.), int(floor-wheel_r-5))
    pole_pt = (int(pos[0]+pole_l*math.sin(phi)),
               int(pos[1]-pole_l*math.cos(phi)))
    cv2.line(image, (0, floor), (width, floor), (0, 0, 0), 2)
    cv2.line(image, pos, pole_pt, (237, 157, 0), 2*wheel_r)
    cv2.circle(image, pos, wheel_r, (255, 50, 0), 7)
    return image


df = pd.read_csv(os.path.abspath("sim_phi_control.csv"))

for i, state in enumerate(df.values):
    state_img = draw_state(state[0], state[1])
    cv2.imshow('simulation', state_img)
    out.write(state_img)
    #if i%30==0 : cv2.imwrite('output%d.jpg'%i, state_img)

    k = cv2.waitKey(1)


out.release()
cv2.destroyAllWindows()

"""
4bar_analysis.py

Remi Salmon, 2018
salmon.remi@gmail.com
"""

# imports
import glob
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as so

# functions
def points_4bar(): # get linkage points coordinates from image
    points_xy = np.zeros((7, 2)) # (x,y) coordinates of the points:
    # points_xy[0, :] = main pivot
    # points_xy[1, :] = lower chainstay/seatstay pivot
    # points_xy[2, :] = high seatstay pivot
    # points_xy[3, :] = rocker pivot
    # points_xy[4, :] = rear axle
    # points_xy[5, :] = rocker shock mount
    # points_xy[6, :] = frame shock mount

    plt.title('click main pivot')
    points = plt.ginput(1)
    points_xy[0, :] = points[0]
    plt.scatter(points_xy[0, 0], points_xy[0, 1], color = 'r', marker = 'o')
    plt.draw()

    plt.title('click lower chainstay/seatstay pivot')
    points = plt.ginput(1)
    points_xy[1, :] = points[0]
    plt.scatter(points_xy[1, 0], points_xy[1, 1], color = 'r', marker = 'o')
    plt.draw()

    plt.title('click high seatstay pivot')
    points = plt.ginput(1)
    points_xy[2, :] = points[0]
    plt.scatter(points_xy[2, 0], points_xy[2, 1], color = 'r', marker = 'o')
    plt.draw()

    plt.title('click rocker pivot')
    points = plt.ginput(1)
    points_xy[3, :] = points[0]
    plt.scatter(points_xy[3, 0], points_xy[3, 1], color = 'r', marker = 'o')
    plt.draw()

    plt.title('click rear axle')
    points = plt.ginput(1)
    points_xy[4, :] = points[0]
    plt.scatter(points_xy[4, 0], points_xy[4, 1], color = 'r', marker = 'o')
    plt.draw()

#    plt.title('click rocker shock mount')
#    points = plt.ginput(1)
#    points_xy[5, :] = points[0]
#    plt.scatter(points_xy[5, 0], points_xy[5, 1], color = 'r', marker = 'o')
#    plt.draw()
#
#    plt.title('click frame shock mount')
#    points = plt.ginput(1)
#    points_xy[6, :] = points[0]
#    plt.scatter(points_xy[6, 0], points_xy[6, 1], color = 'r', marker = 'o')
#    plt.draw()

    return(points_xy)

def l_4bar(points_xy): # find linkage lenghts from points
    l1 = np.linalg.norm(points_xy[1, :]-points_xy[0, :]) # link l1 length (main pivot to lower chainstay/seatstay pivot)
    l2 = np.linalg.norm(points_xy[2, :]-points_xy[1, :]) # link l2 length (lower chainstay/seatstay pivot to high chainstay pivot)
    l3 = np.linalg.norm(points_xy[3, :]-points_xy[2, :]) # link l3 length (high chainstay to rocker pivot)
    l4 = np.linalg.norm(points_xy[0, :]-points_xy[3, :]) # link l4 length (rocker pivot to main pivot)

    return(l1, l2, l3, l4)

def config_4bar(points_xy): # find linkage configuration from points
    (l1, l2, l3, l4) = l_4bar(points_xy)

    lpra = np.linalg.norm(points_xy[1, :]-points_xy[4, :]) # distance from rear axle to lower chainstay/seatstay pivot

    lprap = int(round(100*lpra/min([l1, l2, l3, l4]))) # lpra in % of shortest link

    if  lprap < 20 : # guess...
        linkage_config = 'Split-pivot'
    elif lpra > max(l1, l3):
        if points_xy[3, 1] < points_xy[2, 1]:
            linkage_config = 'VPP'
        else:
            linkage_config = 'DW-link/Maestro'
    elif points_xy[4, 1] > points_xy[1, 1]: # y origin is on top
        linkage_config = 'Linkage-driven single-pivot'
    else:
        linkage_config = 'Horst-link'

    return(linkage_config)

def f_4bar(x, l1, l2, l3, l4, theta1, theta4): # vector loop function = [0, 0]
    theta2 = x[0]
    theta3 = x[1]

    fx = l1*np.cos(theta1)+l2*np.cos(theta2)+l3*np.cos(theta3)+l4*np.cos(theta4)
    fy = l1*np.sin(theta1)+l2*np.sin(theta2)+l3*np.sin(theta3)+l4*np.sin(theta4)

    f = np.array([fx, fy])

    return(f)

def J_4bar(x, l1, l2, l3, l4, theta1, theta4): # Jacobian of vector loop function f
    theta2 = x[0]
    theta3 = x[1]

    J = np.zeros((2, 2))

    J[0, 0] = l1*np.cos(theta1)-l2*np.sin(theta2)+l3*np.cos(theta3)+l4*np.cos(theta4)
    J[0, 1] = l1*np.cos(theta1)+l2*np.cos(theta2)-l3*np.sin(theta3)+l4*np.cos(theta4)
    J[1, 0] = l1*np.sin(theta1)+l2*np.cos(theta2)+l3*np.sin(theta3)+l4*np.sin(theta4)
    J[1, 1] = l1*np.sin(theta1)+l2*np.sin(theta2)+l3*np.cos(theta3)+l4*np.sin(theta4)

    return(J)

def angles_4bar(points_xy): # find angles between links
    (l1, l2, l3, l4) = l_4bar(points_xy)

    theta1 = np.arctan2(points_xy[1, 1]-points_xy[0, 1], points_xy[1, 0]-points_xy[0, 0]) # l1 angle
    theta2 = np.arctan2(points_xy[2, 1]-points_xy[1, 1], points_xy[2, 0]-points_xy[1, 0]) # l2 angle
    theta3 = np.arctan2(points_xy[3, 1]-points_xy[2, 1], points_xy[3, 0]-points_xy[2, 0]) # l3 angle
    theta4 = np.arctan2(points_xy[0, 1]-points_xy[3, 1], points_xy[0, 0]-points_xy[3, 0]) # l4 angle

    return(theta1, theta2, theta3, theta4)

def new_angles_4bar(points_xy, theta1_new): # find links angles with new theta1
    (l1, l2, l3, l4) = l_4bar(points_xy)

    (theta1, theta2, theta3, theta4) = angles_4bar(points_xy)

    x0 = [theta2, theta3] # initial guess

    sol = so.root(f_4bar, x0, args = (l1, l2, l3, l4, theta1_new, theta4), jac = J_4bar) # theta4 constant (l4 == frame)

    theta2_sol = sol.x[0]
    theta3_sol = sol.x[1]

    angles_new = [theta1_new, theta2_sol, theta3_sol, theta4]

    return(angles_new)

def new_points_4bar(points_xy, angles_new): # find points coordinates with new theta1
    (l1, l2, l3, l4) = l_4bar(points_xy)

    theta1_new = angles_new[0]
    theta2_new = angles_new[1]

    points_xy_new = np.zeros(points_xy.shape)

    points_xy_new[0, :] = points_xy[0, :]
    points_xy_new[1, :] = points_xy_new[0, :]+[l1*np.cos(theta1_new), l1*np.sin(theta1_new)]
    points_xy_new[2, :] = points_xy_new[1, :]+[l2*np.cos(theta2_new), l2*np.sin(theta2_new)]
    points_xy_new[3, :] = points_xy[3, :]

    return(points_xy_new)

def ra_4bar(points_xy, points_xy_new, angles_new): # find rear axle coordinates from linkage points
    (l1, l2, l3, l4) = l_4bar(points_xy)

    v2 = [points_xy[2, 0]-points_xy[1, 0], points_xy[2, 1]-points_xy[1, 1]] # l2 vector
    vra = [points_xy[4, 0]-points_xy[1, 0], points_xy[4, 1]-points_xy[1, 1]] # rear axle-lower chainstay/seatstay pivot vector

    l = np.linalg.norm(vra) # distance to rear axle from lower chainstay/seatstay pivot

    if l == 0: # avoid divide by 0 (Split-pivot configuration)
        l = 1

    cos_alpha = (v2[0]*vra[0]+v2[1]*vra[1])/(l2*l) # l2 = norm(v2)
    sin_alpha = (v2[0]*vra[1]-vra[0]*v2[1])/(l2*l) # l2 = norm(v2)

    alpha = np.arctan2(sin_alpha, cos_alpha) # angle to rear axle from l2 (constant)

    theta2_new = angles_new[1]

    x = points_xy_new[1, 0]+l*np.cos(theta2_new+alpha)
    y = points_xy_new[1, 1]+l*np.sin(theta2_new+alpha)

    return(x, y)

def ic_4bar(points_xy): # find coordinates of instant center of rotation = intersection of the 2 rotating links l1, l3
    a = (points_xy[1, 1]-points_xy[0, 1])/(points_xy[1, 0]-points_xy[0, 0]) # l1 slope
    c = points_xy[0, 1]-a*points_xy[0, 0] # l1 offset
    b = (points_xy[3, 1]-points_xy[2, 1])/(points_xy[3, 0]-points_xy[2, 0]) # l3 slope
    d = points_xy[2, 1]-b*points_xy[2, 0] # l3 offset

    if a == b:
        ic = [-1, -1] # avoid divide by 0 (should not happen...)
    else:
        ic = [(d-c)/(a-b), (a*d-b*c)/(a-b)] # (cf. https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_the_equations_of_the_lines)

    return(ic)

# main
# read and show image
image_files = glob.glob('*.jpg')+glob.glob('*.jpeg')
image_file = image_files[0] # process only 1 image

image = plt.imread(image_file)

image = 0.2126*image[:, :, 0]+0.7152*image[:, :, 1]+0.0722*image[:, :, 2] # convert rgb to grayscale

fig = plt.figure()
plt.imshow(image, cmap = 'gray')
xlim = plt.xlim()
ylim = plt.ylim()

# get points coordinates on image
points_xy = points_4bar()

# get linkage configuration
linkage_config = config_4bar(points_xy)

# plot linkages
plt.title(linkage_config+' linkage detected')
plt.plot([points_xy[0, 0], points_xy[1, 0]], [points_xy[0, 1], points_xy[1, 1]], color = 'r')
plt.plot([points_xy[1, 0], points_xy[2, 0]], [points_xy[1, 1], points_xy[2, 1]], color = 'r')
plt.plot([points_xy[2, 0], points_xy[3, 0]], [points_xy[2, 1], points_xy[3, 1]], color = 'r')
plt.plot([points_xy[3, 0], points_xy[0, 0]], [points_xy[3, 1], points_xy[0, 1]], color = 'r', linestyle = '--')

## calculate and plot instant center of rotation
ic_xy = ic_4bar(points_xy)

plt.scatter(ic_xy[0], ic_xy[1], color = 'b', marker = 'o')
plt.plot([points_xy[0, 0], ic_xy[0]], [points_xy[0, 1], ic_xy[1]], color = 'b', linestyle = '--')
plt.plot([points_xy[3, 0], ic_xy[0]], [points_xy[3, 1], ic_xy[1]], color = 'b', linestyle = '--')

# calculate and plot rear axle and instant center of rotation paths
(theta1, theta2, theta3, theta4) = angles_4bar(points_xy)

for theta1_new in np.linspace(theta1, theta1+np.radians(30), 90): # y axis inverted = theta1 increases
    angles_new = new_angles_4bar(points_xy, theta1_new)
    points_xy_new = new_points_4bar(points_xy, angles_new)

    ra_xy = ra_4bar(points_xy, points_xy_new, angles_new)
    ic_xy = ic_4bar(points_xy_new)

    plt.scatter(ra_xy[0], ra_xy[1], color = 'r', marker = '.')
    plt.scatter(ic_xy[0], ic_xy[1], color = 'b', marker = '.')

plt.xlim(xlim)
plt.ylim(ylim)

plt.savefig('4bar_analysis.png')

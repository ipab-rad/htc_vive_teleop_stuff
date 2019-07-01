import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import cv2
import glob
import sys

if len(sys.argv) != 2:
    print("Usage: python play_demo.py <file_glob_pattern>")
    sys.exit(0)

print('Args:', sys.argv)
# Specify a folder, then search for each of the images in this 
f_names = sorted(glob.glob(sys.argv[1]))
print('Files:', f_names, len(f_names))

fig = plt.figure()
ims = []


ims = (cv2.imread(f_name) for f_name in f_names)
im_arts = [[plt.imshow(im[:, :, [2, 1, 0]], animated=True)] for im in ims]

ani = animation.ArtistAnimation(fig, im_arts, interval=50, blit=True, repeat_delay=3000)

# # Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=20, metadata=dict(artist='pc'), bitrate=1800)
# ani.save('video.mp4', writer=writer)

plt.show()
import numpy as np
import matplotlib.pyplot as plt
point="C1t1"
posx_clip=np.load("./{}_x.npy".format(point))

x=np.arange(0,len(posx_clip))
y=np.array([268]*len(posx_clip))
plt.plot(x,posx_clip,'r.')
plt.plot(x,y,'b')
plt.savefig("./{}_x.png".format(point))
plt.show()





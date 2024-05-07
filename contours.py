import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter

seabed = np.random.random((50,50))

seabed = gaussian_filter(seabed, sigma=7)
seabed *= 100
mi, ma = np.floor(np.nanmin(seabed)), np.ceil(np.nanmax(seabed))
step = 2
levels = np.arange(10*(mi//10), ma+step, step)
lws = [0.5 if level % 10 else 1 for level in levels]

# Make the plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(1,1,1)
im = ax.imshow(seabed, cmap='GnBu_r', aspect=0.5, origin='lower')
cb = plt.colorbar(im, label="TWT [ms]")

params = dict(linestyles='solid', colors=['black'], alpha=0.4)
cs = ax.contour(seabed, levels=levels, linewidths=lws, **params)
ax.clabel(cs, fmt='%d')
plt.show()
import time
import plotly.graph_objects as go
import numpy as np

# initialize and display plot
fig = go.FigureWidget()
fig.add_scatter(y=np.random.randint(0,10, 5), fill='tozeroy')
display(fig)

# modify plot using new data
for i in range(10):
    time.sleep(0.5)
    fig.data[0].y = np.random.randint(0,10, 5)
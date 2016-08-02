#!/usr/bin/env python

from IPython import embed
import plotly.plotly as py
import plotly.graph_objs as go
from r4d_common.plot_tools import get_data, simple_plot, odom_pos

def solve_time():
    s = [
            ['elapsed_time', '~/fiducial_slam/out_multimarker_table.bag', '/solve_time', lambda m: m.data.to_sec()],
        ]

    df = get_data(s).iloc[:1000:50]

    embed()

    fig = {
        'data': [go.Bar(x=range(1, 1000, 50),
                        y=df.elapsed_time,
                        name='Solve time',)],
        'layout': {
            'xaxis': {'title': 'Frame number'},
            'yaxis': {'title': "Solve time [s]"}
        }
    }

    url = py.plot(fig, filename='r4d_2_multimarker_solve_time')

if __name__ == "__main__":
    solve_time()
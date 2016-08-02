#!/usr/bin/env python

from IPython import embed
import plotly.plotly as py
import plotly.graph_objs as go
from r4d_common.plot_tools import *

def position_comparison():
    s = [
            ['single', '~/fiducial_slam/out_multimarker_table.bag', '/origin_marker', odom_pos('x')],
            ['multi', '~/fiducial_slam/out_multimarker_table.bag', '/p_ic', odom_pos('x')],
            ['has_m0', '~/fiducial_slam/markers_table.bag', '/rcars/detector/tags', has_marker(0)],
            ['has_m1', '~/fiducial_slam/markers_table.bag', '/rcars/detector/tags', has_marker(1)],
        ]

    df = get_data(s)
    start_time = df.index[0]
    df = df.iloc[140:250]

    data = [go.Bar(x=df.index - start_time,
                   y=df.has_m0,
                   name='Marker 0',
                   yaxis='y2',
                   opacity=0.3)] \
           + [go.Bar(x=df.index - start_time,
                     y=df.has_m1,
                     name='Marker 1',
                     yaxis='y2',
                     opacity=0.3)] \
           + [go.Scatter(x=df.index - start_time,
                         y=df[who],
                         mode='markers',
                         name=who.title(),
                         yaxis='y1') for who in ('single', 'multi')]

    layout = go.Layout(
        barmode='stack',
        title='',
        xaxis={'title': 'Time [s]'},
        yaxis={'title': 'Camera X position [m]'},
        yaxis2={'title': 'Marker visibility', 'overlaying': 'y', 'side': 'right', 'range': (0, 12), 'showticklabels': False})


    fig = go.Figure(data=data, layout=layout)
    url = py.plot(fig, filename='r4d_2_single_vs_multi')


def map_building():
    s = [
            ['has_m0', '~/fiducial_slam/markers_table.bag', '/rcars/detector/tags', has_marker(0)],
            ['has_m1', '~/fiducial_slam/markers_table.bag', '/rcars/detector/tags', has_marker(1)],
            ['m1', '~/fiducial_slam/out_multimarker_table.bag', '/markers_map', marker_pos(1, 'z')],
    ]

    df = get_data(s)

    embed()

    start_time = df.index[0]
    df = df.iloc[140:440]

    data = [
        go.Scatter(x=df.index - start_time,
                   y=df.m1,
                   name='Position'),
         go.Bar(x=df.index - start_time,
                  y=df.has_m0,
                  name='Marker 0',
                  yaxis='y2',
                  opacity=0.3),
         go.Bar(x=df.index - start_time,
                  y=df.has_m1,
                  name='Marker 1',
                  yaxis='y2',
                  opacity=0.3)
    ]

    layout = go.Layout(
        barmode='stack',
        xaxis={'title': 'Time [s]'},
        yaxis={'title': 'Relative marker Z position [m]'},
        yaxis2={'overlaying': 'y', 'side': 'right', 'range': (0, 6), 'showticklabels': False})

    fig = go.Figure(data=data, layout=layout)
    url = py.plot(fig, filename='r4d_2_map')


if __name__ == "__main__":
    # position_comparison()
    map_building()
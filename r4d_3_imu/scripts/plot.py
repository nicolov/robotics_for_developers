#!/usr/bin/env python

from IPython import embed
import plotly.plotly as py
import plotly.graph_objs as go
from r4d_common.plot_tools import *


def pause_biases():
    bag = '~/fiducial_slam/out_imu_table_pause_200_40_skip_1.bag'
    s = [
            ['accel_bias', bag, '/imu_biases', gyro_bias('y')],
        ]

    df = get_data(s)
    start_time = df.index[0]
    df = df.iloc[:1020]

    data = [] \
           + [go.Scatter(x=df.index - start_time,
                         y=df[who],
                         mode='lines+markers',
                         name=who.title()) for who in ('accel_bias',)]

    layout = go.Layout(
      # barmode='stack',
      title='',
      xaxis={'title': 'Time [s]'},
      yaxis={'title': 'Gyro Y bias [rad/s]'},
    )


    fig = go.Figure(data=data, layout=layout)
    url = py.plot(fig, filename='r4d_3_imu_bias_pause')  


def pause_position_comparison():
    bag = '~/fiducial_slam/out_imu_table_pause_200_40_skip_1.bag'
    s = [
            ['Estimate', bag, '/p_ic', odom_pos('x')],
            ['Prediction', bag, '/p_ic_pred', odom_pos('x')],
        ]

    df = get_data(s)
    start_time = df.index[0]
    df = df.iloc[130:230]

    data = [] \
           + [go.Scatter(x=df.index - start_time,
                         y=df[who],
                         mode='lines+markers',
                         name=who.title()) for who in ('Estimate', 'Prediction')]

    layout = go.Layout(
      # barmode='stack',
      title='',
      xaxis={'title': 'Time [s]'},
      yaxis={'title': 'Camera X position [m]'},
    )


    fig = go.Figure(data=data, layout=layout)
    url = py.plot(fig, filename='r4d_3_imu_prediction_pause_pos')


def pause_orientation_comparison():
    bag = '~/fiducial_slam/out_imu_table_pause_200_40_skip_1.bag'
    s = [
            ['Estimate', bag, '/p_ic', odom_euler(1)],
            ['Prediction', bag, '/p_ic_pred', odom_euler(1)],
        ]

    df = get_data(s)
    start_time = df.index[0]
    df = df.iloc[130:230]

    data = [] \
           + [go.Scatter(x=df.index - start_time,
                         y=df[who],
                         mode='lines+markers',
                         name=who.title()) for who in ('Estimate', 'Prediction')]

    layout = go.Layout(
      # barmode='stack',
      title='',
      xaxis={'title': 'Time [s]'},
      yaxis={'title': 'Pitch angle [rad]'},
    )


    fig = go.Figure(data=data, layout=layout)
    url = py.plot(fig, filename='r4d_3_imu_prediction_pause_orient')


def skip_position_comparison():
    bag = '~/fiducial_slam/out_imu_table_pause_0_0_skip_5.bag'
    s = [
            ['Estimate', bag, '/p_ic', odom_pos('x')],
            ['Prediction', bag, '/p_ic_pred', odom_pos('x')],
        ]

    df = get_data(s)
    start_time = df.index[0]
    df = df.iloc[100:600]

    data = [] \
           + [go.Scatter(x=df.index - start_time,
                         y=df[who],
                         mode='lines+markers',
                         name=who.title()) for who in ('Estimate', 'Prediction')]

    layout = go.Layout(
      # barmode='stack',
      title='',
      xaxis={'title': 'Time [s]'},
      yaxis={'title': 'Camera X position [m]'},
    )


    fig = go.Figure(data=data, layout=layout)
    url = py.plot(fig, filename='r4d_3_imu_prediction_skip_pos')


def skip_orientation_comparison():
    bag = '~/fiducial_slam/out_imu_table_pause_0_0_skip_5.bag'
    s = [
            ['Estimate', bag, '/p_ic', odom_euler(1)],
            ['Prediction', bag, '/p_ic_pred', odom_euler(1)],
        ]

    df = get_data(s)
    start_time = df.index[0]
    df = df.iloc[100:600]

    data = [] \
           + [go.Scatter(x=df.index - start_time,
                         y=df[who],
                         mode='lines+markers',
                         name=who.title()) for who in ('Estimate', 'Prediction')]

    layout = go.Layout(
      # barmode='stack',
      title='',
      xaxis={'title': 'Time [s]'},
      yaxis={'title': 'Pitch angle [rad]'},
    )


    fig = go.Figure(data=data, layout=layout)
    url = py.plot(fig, filename='r4d_3_imu_prediction_skip_orient')


if __name__ == "__main__":
    pause_biases()
    # pause_position_comparison()
    # pause_orientation_comparison()
    # skip_position_comparison()
    # skip_orientation_comparison()
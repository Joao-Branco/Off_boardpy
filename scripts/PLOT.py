
import math
from bagpy import bagreader
import pandas as pd
import matplotlib
matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})
import matplotlib.pyplot as plt
import numpy as np
from functools import reduce
from scipy import signal
import os.path
import sklearn.metrics
from matplotlib.animation import FuncAnimation, PillowWriter




def PLOTS(bag_felix, bag_branco):

    topics_felix = ['/target_position', 
                    '/uav0/target_position_geolocation']

    topics_branco = [   '/uav0/target_position', 
                        '/uav0/target_position_geolocation',
                        '/uav1/target_position', 
                        '/uav1/target_position_geolocation',
                        '/uav2/target_position', 
                        '/uav2/target_position_geolocation'] 

    # Give filename of rosbag
    b_felix = bagreader(bag_felix)
    b_branco = bagreader(bag_branco)


    # load messages to dataframes
    dataframes = {}
    minimos_felix = []
    minimos_branco= []

    for topic in topics_felix:
        data = b_felix.message_by_topic(topic)
        if data is not None:
            df = pd.read_csv(data)
            # Add a unique key by appending '_felix' to the topic name
            dataframes[topic + '_felix'] = df
            minimos_felix.append(df.iloc[0, 0])

    for topic in topics_branco:
        data = b_branco.message_by_topic(topic)
        if data is not None:
            df = pd.read_csv(data)
            # Add a unique key by appending '_branco' to the topic name
            dataframes[topic + '_branco'] = df
            minimos_branco.append(df.iloc[0, 0])


    minimo_felix = min(minimos_felix)
    minimo_branco = min(minimos_branco)

    topics_felix = [item + '_felix' for item in topics_felix]
    topics_branco = [item + '_branco' for item in topics_branco]

    for key in dataframes:
        if key  in topics_felix:
            dataframes[key].iloc[:,0] = dataframes[key].iloc[:,0] - minimo_felix
        if key  in topics_branco:
            dataframes[key].iloc[:,0] = dataframes[key].iloc[:,0] - minimo_branco


    felix_error = np.array(np.sqrt(dataframes['/target_position_felix'].x_pos ** 2 + dataframes['/target_position_felix'].y_pos ** 2))
    felix_time = np.array(dataframes['/target_position_felix'].iloc[:,0])

    index = len(felix_error) - np.argmax(felix_error[::-1] == 0) 
    felix_error = felix_error[np.argmax(felix_error != 0):]
    felix_time = felix_time[index:]

    branco_error_0 = np.array(np.sqrt(dataframes['/uav0/target_position_branco'].x_pos ** 2 + dataframes['/uav0/target_position_branco'].y_pos ** 2))
    branco_time_0 = np.array(dataframes['/uav0/target_position_branco'].iloc[:,0])
    branco_error_1 = np.array(np.sqrt(dataframes['/uav1/target_position_branco'].x_pos ** 2 + dataframes['/uav1/target_position_branco'].y_pos ** 2))
    branco_time_1 = np.array(dataframes['/uav1/target_position_branco'].iloc[:,0])

    index = len(branco_error_0) - np.argmax(branco_error_0[::-1] == 0)
    branco_error_0 = branco_error_0[np.argmax(branco_error_0 != 0):]
    branco_time_0 = branco_time_0[index:]

    index = len(branco_error_1) - np.argmax(branco_error_1[::-1] == 0) 
    branco_error_1 = branco_error_1[np.argmax(branco_error_1 != 0):]
    branco_time_1 = branco_time_1[index:]

    min_time = min(branco_time_0[0], branco_time_1[0], felix_time[0])

    branco_time_0 = branco_time_0 - min_time
    branco_time_1 = branco_time_1 - min_time
    felix_time = felix_time - min_time


    print('Felix Error: ', np.mean(felix_error))
    print('Branco 1 Error: ', np.mean(branco_error_1))
    print('Branco 2 Error: ', np.mean(branco_error_0))


    felix_obs_time = np.array(dataframes['/uav0/target_position_geolocation_felix'].iloc[:,0])
    felix_obs_x = np.array(dataframes['/uav0/target_position_geolocation_felix'].x_pos)
    felix_obs_y = np.array(dataframes['/uav0/target_position_geolocation_felix'].y_pos)

    branco_obs_time0 = np.array(dataframes['/uav0/target_position_geolocation_branco'].iloc[:,0])
    branco_obs_x0 = np.array(dataframes['/uav0/target_position_geolocation_branco'].x_pos)
    branco_obs_y0 = np.array(dataframes['/uav0/target_position_geolocation_branco'].y_pos)
    branco_obs_time1 = np.array(dataframes['/uav1/target_position_geolocation_branco'].iloc[:,0])
    branco_obs_x1 = np.array(dataframes['/uav1/target_position_geolocation_branco'].x_pos)
    branco_obs_y1 = np.array(dataframes['/uav1/target_position_geolocation_branco'].y_pos)




    plt.figure(figsize=(13, 6))

    plt.plot(felix_time, felix_error, '+', markersize=3, label='Félix')
    plt.plot(branco_time_0, branco_error_0, '+', markersize=3, label='UAV 1')
    plt.plot(branco_time_1, branco_error_1, '+', markersize=3, label='UAV 2')

    plt.xlabel('Tempo (s)', fontsize=30, fontweight='bold')
    plt.ylabel('Erro de Posição (m)', fontsize=20, fontweight='bold')

    plt.grid()
    plt.tick_params(axis='both', labelsize=15)
    plt.legend(fontsize=20)

    plot_jpg = '/mnt/hdd_1_500gb/jbranco/sims/Error_position.png'
    plt.savefig(plot_jpg)


    plot_jpg = '/mnt/hdd_1_500gb/jbranco/sims/Error_position.pgf'
    plt.savefig(plot_jpg)

    fig, axs = plt.subplots(2, figsize=(13, 9))

    axs[1].plot(felix_obs_time, felix_obs_y, '+', markersize=3)
    axs[0].plot(felix_obs_time, felix_obs_x, '+', markersize=3)
    axs[1].set_ylim(-50, 150)
    axs[0].set_ylim(-50, 150)

    axs[1].set_xlabel('Tempo (s)', fontsize=30, fontweight='bold')
    axs[1].set_ylabel('Y (m)', fontsize=20, fontweight='bold')
    axs[0].set_ylabel('X (m)', fontsize=20, fontweight='bold')
    axs[1].grid()
    axs[0].grid()
    axs[0].tick_params(axis='both', labelsize=20)
    axs[1].tick_params(axis='both', labelsize=20)


    plot_jpg = '/mnt/hdd_1_500gb/jbranco/sims/Felix_obs.png'
    plt.savefig(plot_jpg)


    plot_jpg = '/mnt/hdd_1_500gb/jbranco/sims/Felix_obs.pgf'
    plt.savefig(plot_jpg)

    fig, axs = plt.subplots(2, figsize=(13, 9))

    axs[1].plot(branco_obs_time0, branco_obs_y0, '+', markersize=3 , label='UAV 0')
    axs[1].plot(branco_obs_time1, branco_obs_y1, '+', markersize=3, label='UAV 1')
    axs[0].plot(branco_obs_time0, branco_obs_x0, '+', markersize=3 , label='UAV 0')
    axs[0].plot(branco_obs_time1, branco_obs_x1, '+', markersize=3, label='UAV 1')
    axs[1].set_ylim(-50, 50)
    axs[0].set_ylim(-50, 50)

    axs[1].set_xlabel('Tempo (s)', fontsize=30, fontweight='bold')
    axs[1].set_ylabel('Y (m)', fontsize=20, fontweight='bold')
    axs[0].set_ylabel('X (m)', fontsize=20, fontweight='bold')
    axs[1].grid()
    axs[0].grid()
    axs[0].tick_params(axis='both', labelsize=20)
    axs[1].tick_params(axis='both', labelsize=20)

    handles, labels = axs[0].get_legend_handles_labels()

    fig.legend(handles, labels, loc='upper right', fontsize=25)


    plot_jpg = '/mnt/hdd_1_500gb/jbranco/sims/branco_obs.png'
    plt.savefig(plot_jpg)


    plot_jpg = '/mnt/hdd_1_500gb/jbranco/sims/branco_obs.pgf'
    plt.savefig(plot_jpg)


    print('MAX Felix y: ', np.max(felix_obs_y))
    print('MAX Felix x: ', np.max(felix_obs_x))
    print('MAX Branco 0 y: ', np.max(branco_obs_y0))
    print('MAX branco 0 x: ', np.max(branco_obs_x0))
    print('MAX Branco 1 y: ', np.max(branco_obs_y1))
    print('MAX branco 1 x: ', np.max(branco_obs_x1))







          



PLOTS(bag_branco='/mnt/hdd_1_500gb/jbranco/sims/BRANCO1698573578/Branco1698573578.bag', 
      bag_felix='/mnt/hdd_1_500gb/jbranco/sims/FELIX1698574004/FELIX1698574004.bag')




#!/usr/bin/python3

import time
import math
from pymavlink import mavutil
import cv2
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from threading import Thread

from pymavlink.dialects.v20 import common as mavlink2
from scipy.ndimage.filters import gaussian_filter
from pyray import *

matplotlib.use('Agg')

class DataPlotter:
    def __init__(self):
        self.running = True
        self.mavlink_thread = None
        self.plot_thread = None
        self.plot_image_thread = None

        self.range_data = None
        self.ned_data = None

        self.dpi = 90
        self.screen_width = 1024
        self.screen_height = 512

        self.x = 0
        self.y = 0
        self.dist = 0
        self.new_image = True
        # Set up data
        self.data_size = 100
        
        if os.path.exists("data.npy"):
            self.data_arr = np.load("data.npy")
        else:
            self.data_arr = np.zeros((self.data_size,self.data_size))
            for yyy in range(0, self.data_size):
                for xxx in range(0,self.data_size):
                    self.data_arr[yyy][xxx] = np.nan
            np.save("data.npy", self.data_arr)

            


    def run_mavlink_thread(self):
        # connect to the flight controller
        conn = mavutil.mavlink_connection('udp:127.0.0.1:14551', dialect="common")

        # wait for a heartbeat
        conn.wait_heartbeat()

        # inform user
        print("Connected to system:", conn.target_system, ", component:", conn.target_component)

        message = conn.mav.command_long_encode(
                conn.target_system,  # Target system ID
                conn.target_component,  # Target component ID
                mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                0,  # Confirmation
                mavlink2.MAVLINK_MSG_ID_LOCAL_POSITION_NED,  # param1: Message ID to be streamed
                1000000, # param2: Interval in microseconds
                0,       # param3 (unused)
                0,       # param4 (unused)
                0,       # param5 (unused)
                0,       # param5 (unused)
                0        # param6 (unused)
                )
        conn.mav.send(message)
        last_msg = time.time()
        while self.running:
            conn.recv_match(blocking=False)

            if time.time() - last_msg > 0.1:
                last_msg = time.time()
                try:
                    self.range_data = (conn.messages['DISTANCE_SENSOR'].to_dict())
                    self.ned_data = (conn.messages['LOCAL_POSITION_NED'].to_dict())
                    print(self.ned_data)
                    if self.range_data is not None and self.ned_data is not None:
                        if self.ned_data['x'] > 0.0 and self.ned_data['x'] < self.data_size and self.ned_data['y'] > 0.0 and self.ned_data['y'] < self.data_size:
                            self.x = self.ned_data['y']
                            self.y = self.ned_data['x']

                            self.dist = self.range_data['current_distance'] * 0.01
                            print("{},{},{},{}".format(time.time(),self.x,self.y,self.dist))


                            try:
                                kernel_size = 2
                                for yyy in range(int(self.y - kernel_size), int(self.y + kernel_size + 1)):
                                    for xxx in range(int(self.x - kernel_size), int(self.x + kernel_size + 1)):
                                        if np.isnan(self.data_arr[yyy][xxx]) or (int(self.x) == xxx and int(self.y) == yyy):
                                            self.data_arr[yyy][xxx] = self.dist
                                np.save("data.npy", self.data_arr)
                            except Exception as e:
                                print(e)
                except Exception as e:
                    print(e)
            time.sleep(0.001)

    def run_plot_image_thread(self):
        levels = range(0, 200, 5)
        while self.running:
            print("Regenerating plot...")
            try:

                # Make the plot
                fig = plt.figure(figsize=(self.screen_width/self.dpi, self.screen_height/self.dpi), dpi=self.dpi)
                ax = fig.add_subplot(1,1,1)
                ax.set_aspect('equal')

                data_arr_filtered = gaussian_filter(self.data_arr, sigma=0)
                im = ax.imshow(data_arr_filtered, cmap='GnBu_r', aspect=0.5, origin='lower')
                cb = plt.colorbar(im, label="Depth [m]")

                params = dict(linestyles='solid', colors=['black'], alpha=0.4)
                cs = ax.contour(data_arr_filtered, levels=levels, **params)
                ax.clabel(cs, fmt='%d')

                plt.style.use('dark_background')
                plt.tight_layout()
                plt.title("Measured Bathymetry (NED Coordinates, Depth in meters)")
                plt.savefig('plot.png',pad_inches=0, bbox_inches='tight')
                plt.clf()
                plt.close('all')
                self.new_image = True
            except Exception as e:
                print(e)



    def run_plot_window_thread(self):
        set_config_flags(FLAG_WINDOW_RESIZABLE)
        init_window(1024,512, "Measured Bathymetry Contour")

        texture = None
        while self.running and not window_should_close():
            self.screen_width = get_screen_width()
            self.screen_height = get_screen_height()
            if self.new_image is True:
                try:
                    if texture is not None:
                        unload_texture(texture)
                    image = load_image("plot.png")
                    texture = load_texture_from_image(image)
                    set_texture_filter(texture, TEXTURE_FILTER_BILINEAR)
                    unload_image(image)
                    self.new_image = False
                except Exception as e:
                    print(e)

            begin_drawing()
            clear_background(BLACK)
            
            if texture is not None:
                draw_texture_pro(texture, Rectangle(0,0,texture.width,texture.height), Rectangle(20,20,self.screen_width-40,self.screen_height-40), Vector2(0.0,0.0), 0.0, WHITE)

            end_drawing()
        close_window()
    
    def run(self):
        self.mavlink_thread = Thread(target = self.run_mavlink_thread)
        self.plot_thread = Thread(target = self.run_plot_window_thread)
        self.plot_image_thread = Thread(target = self.run_plot_image_thread)
        self.mavlink_thread.start()
        self.plot_thread.start()
        self.plot_image_thread.start()

if __name__ == "__main__":
    app = DataPlotter()
    app.run()
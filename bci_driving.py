#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).

# Brain-Computer Interface Project
# Cognitve experiment
# Enrique Tomás Martínez Beltrán - https://enriquetomasmb.com
# Github repository: https://github.com/enriquetomasmb/bci-driving


"""
    Cognitve experiment.
    Enrique Tomás Martínez Beltrán - https://enriquetomasmb.com
    
"""

from __future__ import print_function

VERSION = '1.0beta'

# ==============================================================================
# -- find module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
from termcolor import colored


try:
    sys.path.append(glob.glob('../simulator/WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

import time
from configparser import ConfigParser
from random import randint
import threading
import multiprocessing
from bci_npc import main as spawn_npcs
from mne_realtime import StimServer
from pylsl import StreamInfo, StreamOutlet, local_clock
from bci_realtime import start_recording

import random
import operator

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]
        self.timer = 0
        self.simulation_type = 0

    def init_simulation(self):
        init_timer = time.time()
        self.timer = init_timer
        self.hud.timer = init_timer

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager._transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        global spawn_point
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def set_weather(self, weather):
        self.player.get_world().set_weather(getattr(carla.WeatherParameters, weather))

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, vehicle):
        physics_control = vehicle.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        vehicle.apply_physics_control(physics_control)

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot, path):
        
        self.reverseFlag = 0
        # 0 neutral
        # 1 izq
        # 2 der
        self.camera_prev = 0 

        # control de comienzo de la simulación (solo se ha pulsado 1 vez a la Z)
        self.start_simulation = 0
        
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        # print(self._joystick)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('config.ini')
        self._steer_idx = int(
            self._parser.get('UMU Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('UMU Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('UMU Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('UMU Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('UMU Wheel', 'handbrake'))

        # Modificación para los cambios
        self._first_fear_idx = int(self._parser.get('UMU Wheel', 'first_gear'))
        self._second_fear_idx = int(self._parser.get('UMU Wheel', 'second_gear'))
        self._third_fear_idx = int(self._parser.get('UMU Wheel', 'third_gear'))
        self._fourth_fear_idx = int(self._parser.get('UMU Wheel', 'fourth_gear'))
        self._fifth_fear_idx = int(self._parser.get('UMU Wheel', 'fifth_gear'))
        self._control.manual_gear_shift = False
        
        
    def parse_events(self, client, world, clock): 

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == self._reverse_idx:
                    if self._control.reverse:
                        self._control.gear = 0
                    else:
                        self._control.gear = -1
                        if self.reverseFlag == 0:
                            world.camera_manager.toggle_camera()                      
                        self.reverseFlag = 1
                
                # MIRAR A LA DERECHA
                elif event.button == 1:
                    world.camera_manager.toggle_camera2()
                    self.camera_prev = 2
                
                # MIRAR A LA IZQUIERDA
                elif event.button == 2:
                    world.camera_manager.toggle_camera3()                      
                    self.camera_prev = 1

            elif event.type == pygame.JOYBUTTONUP:
                self._control.gear = 0
                if self.reverseFlag == 1:
                    world.camera_manager.toggle_camera()
                    self.reverseFlag = 0
                elif self.camera_prev == 1:
                    world.camera_manager.toggle_camera3()
                elif self.camera_prev == 2:
                    world.camera_manager.toggle_camera2()

            elif event.type == pygame.JOYHATMOTION:
                if event.value == (-1, 0):
                    world.camera_manager.toggle_camera3()
                    self.camera_prev = 1
                elif event.value == (1, 0):
                    world.camera_manager.toggle_camera2()
                    self.camera_prev = 2
                elif event.value == (0, 0):
                    if self.camera_prev == 1:
                        world.camera_manager.toggle_camera3()
                    elif self.camera_prev == 2:
                        world.camera_manager.toggle_camera2()
                else: 
                    self.camera_prev = 0

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                elif event.key == K_z and not self.start_simulation:
                    world.init_simulation()
                    world.simulation_type = 1
                    global qdistraction
                    qdistraction.put(1)
                    print(colored("Comenzando simulación 1...", 'green'))
                    global queue
                    queue.put(True)
                    self.start_simulation = 1
                elif event.key == K_x and not self.start_simulation:
                    world.init_simulation()
                    world.simulation_type = 2
                    global qdistraction
                    qdistraction.put(2)
                    print(colored("Comenzando simulación 2...", 'green'))
                    global queue
                    queue.put(True)
                    self.start_simulation = 1
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' % ('Manual' if self._control.manual_gear_shift else 'Automatic'))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        if self._joystick.get_init():
            numAxes = self._joystick.get_numaxes()
            jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
            # print (jsInputs)
            # Primero: Volante izq derecha | Segundo: Freno | Tercero: Acelerador
            # print (jsInputs[self._steer_idx], jsInputs[self._throttle_idx], jsInputs[self._brake_idx])
            jsButtons = [float(self._joystick.get_button(i)) for i in
                         range(self._joystick.get_numbuttons())]
            
            K1 = 0.12  # 1 --- 0.55
            steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

            K2 = 1.6  # 1.6
            throttleCmd = K2 + (2.05 * math.log10(
                -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
            if throttleCmd <= 0:
                throttleCmd = 0
            elif throttleCmd > 1:
                throttleCmd = 1
    
            brakeCmd = 1.6 + (2.05 * math.log10(
                -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
            if brakeCmd <= 0:
                brakeCmd = 0
            elif brakeCmd > 1:
                brakeCmd = 1
    
            self._control.steer = steerCmd
            self._control.brake = brakeCmd
            self._control.throttle = throttleCmd
    
            #toggle = jsButtons[self._reverse_idx]
            self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

            #print(self._control.steer,  self._control.brake, self._control.hand_brake)
        
        


    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._font_math = pygame.font.Font(mono, 24 if os.name == 'nt' else 26)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self._distraction_text = DisplayDistractionText(self._font_math, (width, 40), (0, 300))
        self._images = DisplayImage((400, 700), (width-800, 100))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        # =====TIEMPO DE SIMULACIÓN =========
        self.timer = 0
        self._distractions = []
        self.math_control = False

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        self._images.tick(world, clock)
        self._distraction_text.tick(world, clock)
        if not self._show_info:
            return
        v = world.player.get_velocity()
        c = world.player.get_control()
        self._info_text = [
            'Tiempo restante: % 12s' % datetime.timedelta(seconds=int(10*60-(time.time()-self.timer))) if self.timer != 0 else 'Pulsa "Z" para comenzar la simulación 1 | Pulsa "X" para comenzar la simulación 2',#self.simulation_time
            'Math',
            'Box'
        ]

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)
    
    def distraction(self, text, seconds=4.0):
        self._distraction_text.set_text(text, seconds=seconds)
    
    def dimage(self, image, seconds=2.0):
        self._images.set_image(image, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            v_offset = 900
            bar_h_offset = 170
            bar_width = 136

            count = 0
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:
                    if count == 0:
                        surface = self._font_mono.render(item, True, (255, 255, 255))
                        display.blit(surface, (10, 10))# 8
                count += 1
                v_offset += 18
        
        self._notifications.render(display)
        self._distraction_text.render(display)
        self._images.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- DisplayImage ----------------------------------------------------------------
# ==============================================================================


class DisplayImage(object):
    def __init__(self, dim, pos):
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_image(self, image, seconds=2.0):
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0,0,0))
        # self.surface.blit(image, (0, 0))
        self.surface.blit(pygame.transform.scale(image, (80, 80)), (0, 0))

    def set_dim_pos(self, dim, pos):
        self.dim = dim
        self.pos = pos

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


class DisplayDistractionText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=4.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def set_dim_pos(self, dim, pos):
        self.dim = dim
        self.pos = pos

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

        self.last_collision = datetime.datetime.now()

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        #enriquetomasmb
        time_event = datetime.datetime.now()
        if (time_event - self.last_collision) > datetime.timedelta(seconds=3):
            # alert_print('c', actor_type, time_event)
            # self.hud.dimage(image)
            # global stim_server
            # stim_server.push_sample([69], time_event.timestamp())
            global qev
            if "vehicle" in actor_type:
                qev.put((21, time_event.timestamp()))
            elif "walker" in actor_type:
                qev.put((22, time_event.timestamp()))
            else:
                qev.put((23, time_event.timestamp()))
            self.last_collision = time_event

        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================

# Solid, SolidSolid, SolidBroken

class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))
        self.last_collision = datetime.datetime.now()
    @staticmethod
    def _on_invasion(weak_self, event):
        global qev
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))
        time_event = datetime.datetime.now()
        if (time_event - self.last_collision) > datetime.timedelta(seconds=3):

            if "SolidSolid" or "SolidBroken" or "Solid" in text:
                qev.put((11, time_event.timestamp()))
        self.last_collision = datetime.datetime.now()


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=2.8, z=1.0),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self._surface = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        
        self._counterCamera = [0,0,0]
#        self._counterCamera2 = 0
        
        # *************CÁMARA EN PRIMERA PERSONA **************
        self._camera_transforms = [
        carla.Transform(carla.Location(x=-0.05,y=-0.45, z=1.17), carla.Rotation(pitch=5)),
        carla.Transform(carla.Location(x=1.2, z=1.2))]
        # ============== CÁMARA REVERSA ============
        self._camera_transformsR = [
        carla.Transform(carla.Location(x=-0.1,y = -0.47, z=1.25), carla.Rotation(pitch=1,yaw=180)),
        carla.Transform(carla.Location(x=1.2, z=1.2))]
        # ============== CÁMARA DERECHA ============
        self._camera_transformsD = [
        carla.Transform(carla.Location(x=0.3,y = -0.37, z=1.1), carla.Rotation(pitch=1,yaw=90)),
        carla.Transform(carla.Location(x=1.2, z=1.2))]
        # ============== CÁMARA IZQUIERDA ============
        self._camera_transformsI = [
        carla.Transform(carla.Location(x=0.3,y = -0.37, z=1.1), carla.Rotation(pitch=1,yaw=-90)),
        carla.Transform(carla.Location(x=1.2, z=1.2))]
        

        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        if self._counterCamera[0] == 0:
            self.sensor.set_transform(self._camera_transformsR[0])
            self._counterCamera[0] += 1
        else:
            self.sensor.set_transform(self._camera_transforms[0])
            self._counterCamera[0] = 0
            
    def toggle_camera2(self):
        if self._counterCamera[1] == 0:
            self.sensor.set_transform(self._camera_transformsD[0])
            self._counterCamera[1] += 1
        else:
            self.sensor.set_transform(self._camera_transforms[0])
            self._counterCamera[1] = 0

    def toggle_camera3(self):
        if self._counterCamera[2] == 0:
            self.sensor.set_transform(self._camera_transformsI[0])
            self._counterCamera[2] += 1
        else:
            self.sensor.set_transform(self._camera_transforms[0])
            self._counterCamera[2] = 0
            
    def set_sensor(self, index, notify=True):
        index = index % len(self._sensors)
        needs_respawn = True if self.index is None \
            else self._sensors[index][0] != self._sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self._sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out/%08d' % image.frame_number)


# ==============================================================================
# -- init_game() ---------------------------------------------------------------
# ==============================================================================

# CarlaUE4.exe -quality-level=Low -no-rendering

def genMath():
    ops = {'+':operator.add,
        '-':operator.sub,
        '*':operator.mul,
        '/':operator.truediv}
    num1 = random.randint(1,20)
    num2 = random.randint(1,10)
    op = random.choice(list(ops.keys()))
    answer = ops.get(op)(num1,num2)
    question = '¿Cuánto es {} {} {}?'.format(num1, op, num2)
    return question, answer

def startMath(hud):
    try:
        width = hud.dim[0]
        height = hud.dim[1]
        global qev
        global qdistraction
        distraction = 0
        while True:
            if not qdistraction.empty():
                distraction = qdistraction.get()
            if distraction == 2:
                q, ans = genMath()
                hud._distraction_text.set_dim_pos((width/4, 40), (60, 60))
                time_event = datetime.datetime.now()
                time.sleep(10)
                hud.distraction('{}'.format(q), seconds=5)
                qev.put((51, time_event.timestamp()))

                q, ans = genMath()
                hud._distraction_text.set_dim_pos((width/4, 40), (width/2 + 60, 60))
                time.sleep(10)
                time_event = datetime.datetime.now()
                hud.distraction('{}'.format(q), seconds=5)
                qev.put((51, time_event.timestamp()))
                time.sleep(20)
            elif distraction == 99:
                return
    finally:
        pass

def startBox(hud):
    try:
        global qev
        global qdistraction
        image = pygame.image.load("resources/azul.jpg")
        distraction = 0
        while True:
            if not qdistraction.empty():
                distraction = qdistraction.get()
            if distraction == 2:
                x = random.randint(100, 900)
                y = random.randint(100, 900)
                hud._images.set_dim_pos((80, 80), (x, y))
                time.sleep(50)
                time_event = datetime.datetime.now()
                hud.dimage(image, seconds=5)
                qev.put((52, time_event.timestamp()))

                x = random.randint(100, 900)
                y = random.randint(100, 900)
                hud._images.set_dim_pos((80, 80), (x, y))
                time_event = datetime.datetime.now()
                hud.dimage(image, seconds=5)
                qev.put((52, time_event.timestamp()))

                x = random.randint(100, 900)
                y = random.randint(100, 900)
                hud._images.set_dim_pos((80, 80), (x, y))
                time_event = datetime.datetime.now()
                hud.dimage(image, seconds=5)
                qev.put((52, time_event.timestamp()))

                x = random.randint(100, 900)
                y = random.randint(100, 900)
                hud._images.set_dim_pos((80, 80), (x, y))
                time_event = datetime.datetime.now()
                hud.dimage(image, seconds=5)
                qev.put((52, time_event.timestamp()))

                time.sleep(50)
            elif distraction == 99:
                return
        # t_end = time.time() + 5
        # while time.time() < t_end:  
        #     hud.distraction('Hola esto es una prueba', seconds=5)
    finally:
        pass

def init_game(args, path, server):
    global spawn_point
    global stim_server
    stim_server = server

    pygame.mixer.pre_init(44100, -16, 2, 4096)
    pygame.mixer.init()
    pygame.init()
    pygame.font.init()
    pygame.display.set_caption('Enrique Tomás Martínez Beltrán - University of Murcia')
    world = None

    try:
        actor_list = []
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        #client.load_world('Town04') #03-default 02-littlecity 01-river2cities 04-autoviacity 05-NOmuyanchas
        #client.load_world('Town04')

        global display
        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        world.set_weather('SoftRainSunset') # https://carla.readthedocs.io/en/stable/carla_settings/
        controller = DualControl(world, args.autopilot, path)

        # ============== SONIDOS ==============
        s00 = pygame.mixer.Sound("resources/0_arranque.wav");
        s00.set_volume(0.25)
        s0 = pygame.mixer.Sound("resources/1_quieto2.wav");
        s0.set_volume(0.15)
        s1 = pygame.mixer.Sound("resources/3_acceleration_slow.wav")
        s1.set_volume(0.05)
        s2 = pygame.mixer.Sound("resources/4_desacceleration.wav")
        s2.set_volume(0.05)
        s3 = pygame.mixer.Sound("resources/2_reverse.wav");
        s3.set_volume(0.05)
        s4 = pygame.mixer.Sound("resources/5_constant2.wav")
        s4.set_volume(0.05)
        # Pitidos
        s6 = pygame.mixer.Sound("resources/6_honking.wav")
        s7 = pygame.mixer.Sound("resources/6_1_honking.wav")
        s_honk = [s6,s7]

        # Images
        # global image
        # image = pygame.image.load("resources/test3.png").convert()

        clock = pygame.time.Clock()
        id_last_red = 0

        s00.play(0)# SONIDO DE ENCENDIDO

        # Bandera para iniciar el timer
        flag = 0
        v_diff = 0
        cambio_pas = 0# comience idealmente en neutro
        control_loop= True
        # Tiempo de simulación
        t_sim = 10

        distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
        dmax = 30;dmin = 0
        m = (0-1)/(dmax-dmin);b = -m*(dmax)

        global qdistraction
        qdistraction = multiprocessing.Queue()
        tr = threading.Thread(target=startMath, args=(hud,qdistraction))
        tr.daemon = True
        tr.start()
        tr2 = threading.Thread(target=startBox, args=(hud,qdistraction))
        tr2.daemon = True
        tr2.start()
        while control_loop:
            t = world.player.get_transform()
            c = world.player.get_control()
            v = world.player.get_velocity()
            
            vehicles = world.world.get_actors().filter('vehicle.*')
            traffic_lights = world.world.get_actors().filter('traffic.traffic_light')

            velocidad = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
            reversa = c.reverse
            
            if flag == 0:
                ini = time.time()
                v_past = velocidad# velocidad pasada
                flag = 1
            
            if  int(round(time.time()-ini,1)*10) >= 5:
                flag = 0
                v_diff = velocidad-v_past
#                print('incremento de: ' + str(v_diff) +' km/h')                
                
            if (velocidad < 2 and not(reversa)):
                s1.stop()
                s2.stop()
                s3.stop()
                s4.stop()
                s0.play(-1)
                
            # ACELERACION
            elif (velocidad > 2 and not(reversa) and v_diff > 0 and abs(v_diff) > 2):
                s0.stop()
                s2.stop()
                s3.stop()
                s4.stop()
                s1.play(-1)
            # DESACELERACION
            elif (velocidad > 2 and not(reversa) and v_diff < 0 and abs(v_diff) > 2):
                s0.stop()
                s1.stop()
                s3.stop()
                s4.stop()
                s2.play(-1)
            # CONSTANTE
            elif (velocidad >2 and not(reversa) and abs(v_diff) < 2):
                s0.stop()
                s1.stop()
                s3.stop()
                s2.stop()
                s4.play(-1)
            
            elif (velocidad > 2 and reversa and abs(v_diff) > 2):
                s0.stop()
                s1.stop()
                s2.stop()
                s4.stop()
                s3.play()
                
            # === Si hay un cambio ===
            if (cambio_pas != c.gear and c.gear != 0):
                s1.stop()
                s2.stop()
                s3.stop()
                s4.stop()
                s0.stop()
                s0.play(-1)
                
            cambio_pas = c.gear
#            print(c.gear)
            
            # Default
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            
            #enriquetomasmb
            if world.player.is_at_traffic_light():
                traffic_light = world.player.get_traffic_light()
                #print(traffic_light)
                if (traffic_light.get_state() == carla.TrafficLightState.Red) and (traffic_light.id != id_last_red):
                    # world.hud.notification("Traffic light changed! Good to go!")
                    time_event = datetime.datetime.now()
                    #alert_print('tl', str(traffic_light.id), time_event)
                    global qev
                    qev.put((15, time_event.timestamp()))
                    id_last_red = traffic_light.id
                    # traffic_light.set_state(carla.TrafficLightState.Green)

            world.tick(clock)
            world.render(display)
            
            
            # ========= TERMINE LA SIMULACIÓN ========
            # World.timer será != 0 cuando se pulse "Z"
            if world.timer != 0:
                start = world.timer
                #print(round(time.time()-start)-1)
                if round(time.time()-start)-1 >= t_sim*60:
                    # MENSAJE DE FIN
                    default_font = 'monotypecorsiva'
                    mono = pygame.font.match_font(default_font)
                    font_mono = pygame.font.Font(mono, 60)                
                    surface = font_mono.render('Fin de la simulación', True, (255, 255, 255))
                    display.blit(surface, (round(args.height/2),round(args.width/2)-300))# 8                    
                    pygame.joystick.quit()
                    # Pasados 5 segundos más, cierre el juego
                    if round(time.time()-start)-1 >= t_sim*60+5:
                        control_loop = False

            pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()
        
        #client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- User --------------------------------------------------------
# ==============================================================================


class User(object):
    def __init__(self):
        self.name = input("Name: ")
        self.age = input("Age: ")
        self.sex = input("Sex (F/M): ")
        self.path = "exp_{}".format(datetime.datetime.now().strftime("%d-%m-%Y-%H-%M-%S"))
    
    def __str__(self) -> str:
        return "User[name={}, age={}, sex={}]".format(self.name,self.age,self.sex)
    
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def alert_print(type_alert, alert, date):
    print(colored("<---------------------------------------------------->", 'green'))
    print(colored("[!] Collision with {}".format(alert), 'green')) if type_alert == 'c' else print(colored("[!] TrafficLightState {}: Red".format(alert), 'green'))
    print(colored("timestamp: {}".format(date.timestamp()), 'green'))
    print(colored("date: {}".format(date), 'green'))
    print(colored("<---------------------------------------------------->", 'green'))


def main():

    banner = """
            ██████╗  ██████╗██╗   ████████╗███████╗███╗   ███╗
            ██╔══██╗██╔════╝██║   ╚══██╔══╝██╔════╝████╗ ████║
            ██████╔╝██║     ██║█████╗██║   █████╗  ██╔████╔██║
            ██╔══██╗██║     ██║╚════╝██║   ██╔══╝  ██║╚██╔╝██║
            ██████╔╝╚██████╗██║      ██║   ██║     ██║ ╚═╝ ██║
            ╚═════╝  ╚═════╝╚═╝      ╚═╝   ╚═╝     ╚═╝     ╚═╝
                                Enrique Tomás Martínez Beltrán
    """
    print(colored(banner, 'yellow'))

    argparser = argparse.ArgumentParser(
        description='Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1920x1080',
        help='window resolution (default: 1920x1080)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.model3',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=0.5,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    logging.info('creating user profile...')
    user = User()

    logging.info('creating experiment...')

    if not os.path.isdir('experiments/' + user.path):
        os.makedirs("experiments/{}/driving".format(user.path))
        os.makedirs("experiments/{}/eeg".format(user.path))
    
    with open('experiments/{}/metadata.txt'.format(user.path), 'w') as out_file:
        out_file.write(str(user))

    try:
        # Creating stim communication
        info = StreamInfo('Estimulo', 'Estimulo', 1, 0, 'int32', 'estimulo12310')
        server = StreamOutlet(info)

        #Creating processes: NPCs and recording EEG
        thread_npc = multiprocessing.Process(target=spawn_npcs)
        #thread_npc.start()
        
        global queue
        queue = multiprocessing.Queue()
        queue.put(False)
        global qev
        qev = multiprocessing.Queue()
        thread_rec = multiprocessing.Process(target=start_recording, args=('openbcigui', 60, queue, qev, "experiments/{}/eeg/".format(user.path)))
        # thread_rec.start()

        # Init driving scenario
        init_game(args, user.path, server)

    except KeyboardInterrupt:
        pass
    finally:
        #stop_driving = True
        print(colored("Closing processes...", 'red'))
        thread_npc.terminate()
        thread_npc.join()
        thread_rec.terminate()
        thread_rec.join()


if __name__ == '__main__':
    stop_driving = False
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print(colored("Thanks for using my driving scenario, see you later!", "green"))
        print(colored("Created by Enrique Tomás Martínez Beltrán", "green"))
    


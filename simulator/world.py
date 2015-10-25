#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2015 sean <sean@wireless-10-145-81-36.public.utexas.edu>
#
# Distributed under terms of the MIT license.

from graphics import *

class World(object):
  def __init__(self, width, height, background_color="green"):
    self._width = width
    self._height = height
    self.background_color = background_color
    self._window = None
    self._roads = set()
    self._cars = set()

  def Simulate(self):
    self._DrawWorld()

  def AddRoad(self, road):
    self._roads.add(road)

  def AddCar(self, car):
    self._cars.add(car)

  def GetWidth(self):
    return self._width

  def GetHeight(self):
    return self._height

  def _DrawWorld(self):
    self._window = GraphWin('CARSTOP simulator', self._width, self._height)

    # Draw Scene
    self._DrawBackground()
    self._DrawRoads()
    self._DrawCars()

    # Wait for input to quit
    self._window.getMouse()
    self._window.close()

  def _DrawBackground(self):
    rect = Rectangle(Point(0, 0), Point(self._width, self._height))
    rect.setFill(self.background_color)
    rect.draw(self._window)

  def _DrawRoads(self):
    for road in self._roads:
      road.Draw(self._window)

  def _DrawCars(self):
    for car in self._cars:
      car.Draw(self._window)

class Road(object):
  def __init__(self, start, end, width, color="black"):
    self.start = start # as a Point
    self.end = end # as a Point
    self.width = width # in pixels
    self.color = color

  def Draw(self, window):
    top_left = Point(self.start.getX(), self.start.getY() - self.width / 2)
    bottom_right = Point(self.end.getX(), self.end.getY() + self.width / 2)
    rect = Rectangle(top_left, bottom_right)
    rect.setFill(self.color)
    rect.draw(window)

class Car(object):
  def __init__(self, position=Point(0, 0), width=10, length=20, direction=0.0,
      velocity=0.0, acceleration=0.0, color="blue"):
    self._position = position
    self._width = width
    self._length = length
    self._direction = direction
    self._velocity = velocity
    self._acceleration = acceleration
    self._color = color

  def Draw(self, window):
    top_left = Point(self._position.getX() - self._length / 2,
        self._position.getY() - self._length / 2)
    bottom_right = Point(self._position.getX() + self._length / 2,
        self._position.getY() + self._width / 2)
    rect = Rectangle(top_left, bottom_right)
    rect.setFill(self._color)
    rect.draw(window)

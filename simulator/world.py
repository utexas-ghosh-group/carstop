'''
This utilizes the tkinter package and the graphics.py code that Sean found
3/22/16
last mod 10/7/16 adding exit keystroke ('q') and placing pause before update
'''

from graphics import *
from numpy import sin, cos, pi


class World(object):
    def __init__(self, width, height, resolution=10, background_color="white"):
        self._width = width
        self._height = height
        self.PPM = resolution # pixels per meter
        self._screenWidth = int(self._width * self.PPM) + 2
        self._screenHeight = int(self._height * self.PPM) + 2
        
        self.background_color = background_color
        self._window = None
        self._roads = []
        self._cars = {}
    
    def Start(self, wait=False):
        self._window = GraphWin('CARSTOP simulator', self._screenWidth,
                                        self._screenHeight)
        self._DrawBackground()
        self._DrawRoads()
        self._window.update()
        
        if wait:
            self._window.getMouse()
        
    def Step(self, allowPause = False, allowExit = False):
        if allowExit and self._window.checkKey() == 'q':
            return True
        if allowPause and not (self._window.checkMouse() is None):
            self._window.getMouse()
        self._DrawCars()
        self._window.update()
        
    def End(self, wait=False):
        if wait:
            self._window.getMouse()
        self._window.close()

    def AddRoad(self, road):
        self._roads.append(road)

    def AddCar(self, car):
        self._cars[car.ID] = car

    def GetWidth(self):
        return self._width

    def GetHeight(self):
        return self._height

    def _DrawBackground(self):
        self._window.config({'background':'white'})
        #rect = Rectangle(Point(0, 0), Point(self._width*self.PPM, self._height*self.PPM))
        #rect.setFill(self.background_color)
        #rect.draw(self._window)

    def _DrawRoads(self):
        for road in self._roads:
            road.Draw(self)

    def _DrawCars(self):
        for car in self._cars.itervalues():
            car.Draw(self)

    def remap(self, x, y):
        x = int(x*self.PPM) + 1
        x = min(max(x, 0), self._screenWidth-1)
        y = int((self._height - y)*self.PPM) + 1
        y = min(max(y, 0), self._screenHeight-1)
        return [x,y]
        
    def moveCar(self, ID, x, y, angle):
        self._cars[ID].move(x,y,angle)

    def removeCar(self, ID):
        self._cars[ID].remove(self)
        del self._cars[ID]
        
    def drawOtherShape(self, shape):
        shape.Draw(self)


class Road(object):
    def __init__(self, startpoint, endpoint, width = 3.):
        mag = ((startpoint[1]-endpoint[1])**2 + (startpoint[0]-endpoint[0])**2)**.5        
        unitv = ( (endpoint[0]-startpoint[0])/mag, (endpoint[1]-startpoint[1])/mag )
        orthv = ( unitv[1] * width / 2., -unitv[0] * width / 2. )
        
        self.points = [[startpoint[0]+orthv[0], startpoint[1]+orthv[1]],
                       [startpoint[0]-orthv[0], startpoint[1]-orthv[1]],
                       [endpoint[0]-orthv[0], endpoint[1]-orthv[1]],
                       [endpoint[0]+orthv[0], endpoint[1]+orthv[1]]]
                       
    def Draw(self, world):
        polygonArgs = []
        for pp in self.points:
            polygonArgs += world.remap(pp[0],pp[1])
        polygonOptions = {"fill":"black", "outline":"white", "width":"1"}
        polygonArgs += [polygonOptions]
        GraphWin.create_polygon(world._window, polygonArgs)


class Car(object):
    def __init__(self, ID, x, y, angle, length=5., width=2.):
        self.ID = ID
        self.length = length
        self.width = width
        self.onGrid = False
        self.move(x, y, angle)
        
    def move(self, x, y, angle):
        ll = self.length
        ww = self.width/2.
        pp = 1.
#        self.polyPoints = [[x-ww*sin(angle), y+ww*cos(angle)],
#                           [x+ww*sin(angle), y-ww*cos(angle)],
#                           [x+ww*sin(angle)-ll*cos(angle), y-ww*cos(angle)-ll*sin(angle)],
#                           [x-ww*sin(angle)-ll*cos(angle), y+ww*cos(angle)-ll*sin(angle)]]
        self.polyPoints = [[x-ww*sin(angle)-pp*cos(angle), y+ww*cos(angle)-pp*sin(angle)],
                           [x, y],
                           [x+ww*sin(angle)-pp*cos(angle), y-ww*cos(angle)-pp*sin(angle)],
                           [x+ww*sin(angle)-ll*cos(angle), y-ww*cos(angle)-ll*sin(angle)],
                           [x-ww*sin(angle)-ll*cos(angle), y+ww*cos(angle)-ll*sin(angle)]]
        self.textPoints = [x-ll/2*cos(angle) , y-ll/2*sin(angle)]
           
    def remove(self, world):
        if self.onGrid:
            world._window.delete(self.polygonID)
            world._window.delete(self.textID)
          
    def Draw(self, world):
        polyPoints = list((world.remap(x,y) for x,y in self.polyPoints))
        textPoints = world.remap(*self.textPoints)

        if self.onGrid:
            polygonArgs = []
            for pp in polyPoints:
                polygonArgs += pp
            world._window.coords(self.polygonID, *polygonArgs)
            world._window.coords(self.textID, *textPoints)
            
        else:
            polygonArgs = []
            for pp in polyPoints:
                polygonArgs += pp
            polygonOptions = {"fill":"yellow", "outline":"", "width":"1"}
            polygonArgs += [polygonOptions]
            self.polygonID = GraphWin.create_polygon(world._window, polygonArgs)
            textOptions = {"fill":"black","text":self.ID[:2]}
            #"font": ("helvetica", 12, "normal")
            self.textID = world._window.create_text(textPoints,textOptions)
            self.onGrid = True
            
            
class IntersectionBox(Road):
    def __init__(self, center, width, thickness = 1.5):
        self.rectanglePoints = [[center[0] - width, center[1] + width], 
                                [center[0] + width, center[1] - width]]
        self.curveTL = [[center[0]-width-thickness, center[1]+width+thickness],
                        [center[0]-width+thickness, center[1]+width-thickness]]
        self.curveTR = [[center[0]+width-thickness, center[1]+width+thickness],
                        [center[0]+width+thickness, center[1]+width-thickness]]
        self.curveBL = [[center[0]-width-thickness, center[1]-width+thickness],
                        [center[0]-width+thickness, center[1]-width-thickness]]
        self.curveBR = [[center[0]+width-thickness, center[1]-width+thickness],
                        [center[0]+width+thickness, center[1]-width-thickness]]
        
    def Draw(self, world):
        rectangleOptions = {'fill':'black','outline':'black','width':'1'}
        repoints = list((world.remap(x,y) for x,y in self.rectanglePoints))
        rectangleBbox = (repoints[0][0], repoints[0][1],
                         repoints[1][0], repoints[1][1])
        world._window.create_rectangle(rectangleBbox, rectangleOptions)
        
        curveOptions = {'fill':world.background_color,'outline':''}
        repoints = list((world.remap(x,y) for x,y in self.curveTL))
        curveTLbbox = (repoints[0][0], repoints[0][1],
                         repoints[1][0], repoints[1][1])
        world._window.create_oval(curveTLbbox, curveOptions)
        
        repoints = list((world.remap(x,y) for x,y in self.curveTR))
        curveTRbbox = (repoints[0][0], repoints[0][1],
                         repoints[1][0], repoints[1][1])
        world._window.create_oval(curveTRbbox, curveOptions)
        
        repoints = list((world.remap(x,y) for x,y in self.curveBL))
        curveBLbbox = (repoints[0][0], repoints[0][1],
                         repoints[1][0], repoints[1][1])
        world._window.create_oval(curveBLbbox, curveOptions)
        
        repoints = list((world.remap(x,y) for x,y in self.curveBR))
        curveBRbbox = (repoints[0][0], repoints[0][1],
                         repoints[1][0], repoints[1][1])
        world._window.create_oval(curveBRbbox, curveOptions)
        
        
class CrashSymbol(object):
    def __init__(self, center, r1 = 2., r2 = 1.):
        self.points = []
        for k in range(16):
            angle = k*pi/8.
            rad = r1
            if k % 2 > 0:
                rad = r2
            self.points += [[center[0] + rad*cos(angle),
                             center[1] + rad*sin(angle)]]
        
    def Draw(self, world):
        polygonOptions = {"fill":"red", "outline":"", "width":"1"}
        polygonArgs = []
        for pp in self.points:
            polygonArgs += world.remap(*pp)
        polygonArgs += [polygonOptions]
        GraphWin.create_polygon(world._window, polygonArgs)
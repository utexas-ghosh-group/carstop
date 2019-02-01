import socket  # Import socket module
import time
import math
import numpy as np
from _thread import start_new_thread

from options import mapfile, TCP_PORT

# This is the socket server

TCP_IP = "192.168.43.83"
def lat_long_distance(point1, point2):
    R = 6373.0
    lat1 = math.radians(point1[0])
    lon1 = math.radians(point1[1])
    lat2 = math.radians(point2[0])
    lon2 = math.radians(point2[1])

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c

    return(distance * 1000)

def danger_calculator(dict_of_v, dict_of_p):
    final_dict = []
    counter1 = 0

    for vehicle in dict_of_v:
        vehicle_GPS = vehicle[1]
        point1 = [float(vehicle_GPS.split("+")[0]),float(vehicle_GPS.split("+")[1])]

        counter2 = 0

        for person in dict_of_p:
            person_GPS = person[1]
            point2 = [float(person_GPS.split("+")[0]), float(person_GPS.split("+")[1])]

            distance_bt = lat_long_distance(point1, point2)

            if distance_bt > 20:
                pass
            elif distance_bt > 5.8:
                if person[2] != "Dang":
                    person[2] = "Caut"
                if vehicle[2] != "Dang":
                    person[2] = "Caut"
            else:
                person[2] = "Dang"
                vehicle[2] = "Dang"
            dict_of_p[counter2] = person
            counter2 = counter2 + 1
        dict_of_v[counter1] = vehicle
        counter1 = counter1 + 1
    final_dict = dict_of_p + dict_of_v
    return  final_dict



def danger_calculator_prelim(lines):
    global centerLongitude
    global centerLatitude
    final_dict = []
    dict_of_vehicles = []
    dict_of_people = []

    for line in lines:

        delimited_line = line.split(",")
        print(delimited_line)
        # time.sleep(1)
        distance = (float(delimited_line[1]))

        angle = float(delimited_line[2])

        centerLatitude2, centerLongitude2 = Sensor_vector_lat_long_convert(distance, centerLatitude, centerLongitude,
                                                                           angle)

        if "car" in line:
            temp_vehicle = str(centerLatitude2) + "+" + str(centerLongitude2) #"#line.split("&")[1].replace("!", "")
            dict_of_vehicles.append(["car", temp_vehicle, "Safe"])
        elif "person" in line:
            temp_person = str(centerLatitude2) + "+" + str(centerLongitude2)
            #temp_person = line.split("&")[1].replace("!", "")
            dict_of_people.append(["person", temp_person, "Safe"])
        else:
            temp_vehicle = str(centerLatitude2) + "+" + str(centerLongitude2) #"#line.split("&")[1].replace("!", "")
            dict_of_vehicles.append(["object", temp_vehicle, "Safe"])

    final_dict = danger_calculator(dict_of_vehicles, dict_of_people)
    return final_dict


def Sensor_vector_lat_long_convert(sensor_x_vec, centerLatitude, centerLongitude, angle):
    angle = angle + 180
    latitude_apropos = 0
    long_apropos = 0
    rad_angle = np.radians(angle)
    print(angle)
    centerLatitudeDMS = decdeg2dms(centerLatitude)
    deg = str(centerLatitudeDMS[0]).split(".")[0]
    minu = str(centerLatitudeDMS[1]).split(".")[0]
    sec = str(centerLatitudeDMS[2])
   # centerLatitudeDMS2 = float(deg + min+sec)

    centerLongitudeDMS = decdeg2dms(centerLongitude)
    deg = str(centerLongitudeDMS[0]).split(".")[0]
    minu = str(centerLongitudeDMS[1]).split(".")[0]
    sec = str(centerLongitudeDMS[2])
  #  centerLongitudeDMS2 = float(deg + min + sec)
    centerLongitudeDMS2 = centerLongitude
    centerLatitudeDMS2 = centerLatitude


    print(centerLatitudeDMS)
    print(centerLongitudeDMS)
    #time.sleep(5)

    sensor_y = sensor_x_vec * math.sin(rad_angle)

    sensor_x = sensor_x_vec * math.cos(rad_angle)

    print(sensor_y)
    print(sensor_x)

    offset = np.radians(centerLatitudeDMS2)

    long_offset = 111320 * math.cos(offset)

    lat_offset = 110540 
    latitude_apropos = sensor_y / lat_offset
    long_apropos = sensor_x / long_offset

    Finallongitude = centerLongitudeDMS2 + long_apropos
    Finallatitude = centerLatitudeDMS2 + latitude_apropos



 #   FinallatitudeDD= dms2dd(Finallatitude)
#    FinallongitudeDD = dms2dd(Finallongitude)

    return Finallatitude, Finallongitude

def decdeg2dms(dd):
   is_positive = dd >= 0
   dd = abs(dd)
   minutes,seconds = divmod(dd*3600,60)
   degrees,minutes = divmod(minutes,60)
   degrees = degrees if is_positive else -degrees
   return (degrees,minutes,seconds)
def dms2dd(degrees, minutes, seconds, direction):
    dd = float(degrees) + float(minutes)/60 + float(seconds)/(60*60)
    if direction == 'E' or direction == 'N':
        dd *= -1
    return dd


#


# So the basic idea of this code is that The server sens "text" modules
# Text Modules are formatted as follows
#   A "#" means that we are starting a new group of detected objects, it will clear all previous markers on the app
#  From here all objects are delimited as follows  $ "Object Found" @ Threat Level & Latitude , Longitude !
# The symbols are how we can tell the difference between two string objects
#0
#  TO handle TCP communication we connect to an device once from there we only disconnect if we want to flush ALL data or to shut down the progr
#  We have a limited number of devices we can connect to on one port so this is important
#
#  So that we do not send too much data we send our data and then wait for a response from the client device. THis ensures that we do not overload the client device with info.

text = "#1$Car@Dang&30.28876,-97.73623!"
text2 ="#1$Crane@Safe&30.28876,-97.73623!"
text3 ="#1$Dozer@Caut&30.28876,-97.73623!"
centerLatitude = 30.3826098
centerLongitude = -97.7262975

import os 


def client_thread(clientsocket, addr):
    global centerLongitude
    global centerLatitude

    while(True):
    
        if (os.stat(mapfile).st_size != 0):

            lines = open(mapfile).read().splitlines()
            print(lines)
            if lines and len(lines[0]) > 1:
                print(lines)
                #time.sleep(1)
                text1 = "#"

                text1= text1 + str(len(lines))
                
                danger_lines = danger_calculator_prelim(lines)

                # Name Distance, angle
                #counter = 0

                for line in danger_lines:
                    try:
                        #print('line: ',line)
                        #time.sleep(2)
                        delimited_line = line
                        #delimited_line = line[counter]
                        
                        #print('delimited_line: ', delimited_line)
                        #time.sleep(1)
                        #distance = (float(delimited_line[1]))
                        #distance = distance * .0254

                        #angle = float(delimited_line[2])
                        #print('angle: ', angle)
                        #print('distance: ', distance)

                        if len(delimited_line) == 3:
                            
                            centerLatitude2 = delimited_line[1].split("+")[0]
                            centerLongitude2 = delimited_line[1].split("+")[1] #Sensor_vector_lat_long_convert(distance, centerLatitude, centerLongitude, angle)
                            #print(centerLatitude2)
                            #print(centerLongitude2)
                            text1 = text1 + "$"
                            text1 = text1 + str(delimited_line[0])
                            text1= text1 + "@"
                            text1 = text1 + str(delimited_line[2])
                            text1 = text1 + "&"
                            text1 = text1 + str(centerLatitude2)
                            text1 = text1 + ","
                            text1 = text1 + str(centerLongitude2)
                            text1 = text1 + "!"
                            clientsocket.send(text1.encode('utf-8'))
                            clientsocket.recv(1024).decode('utf-8')
                            print(text1)
                    except Exception as e:
                        print("ERROR IN DELIMITING")
                        print(e)

                    text1 = "#"
                    text1 = text1 + " "
            else:
                print("Nothing detected!")
        else:
            print("Nothing detected!")


#
#
#
#
# def client_thread(clientsocket,addr):
#
#     counter = 0
#
#     text = "#1$Car@Dang&30.28876,-97.73623!"
#     text2 = "#1$Crane@Safe&30.28876,-97.73623!"
#     text3 = "#1$Dozer@Caut&30.28876,-97.73623!"
#
#     second_protocol_flag = False
#     third_protocol_flag = True
#
#     while (True):
#         if second_protocol_flag == True:
#             # clientsocket.init()
#             #clientsocket.send(t)
#             counter = 0
#             text4 = "#1$Car@Safe&30.28876,-97.73623!"
#             while (counter < 10):
#                 #clientsocket.init()
#                 clientsocket.send(text4.encode('utf-8'))
#                 clientsocket.recv(1024).decode('utf-8')
#                 # 17, 27
#                 new = list(text4)
#                 if (counter == 2):
#                     new[7] = "C"
#                     new[8] = "a"
#                     new[9] = "u"
#                     new[10] = "t"
#                 if (counter == 6):
#                     new[7] = "D"
#                     new[8] = "a"
#                     new[9] = "n"
#                     new[10] = "g"
#                 new[18] = str((counter + 1) % 10)
#                 new[28] = str((counter + 1) % 10)
#                 text4 = ''.join(new)
#                 print(text4)
#                 #clientsocket.disconnect()
#                 counter = counter + 1
#                 print(counter)
#             second_protocol_flag = False
#         elif third_protocol_flag == True:
#             third_protocol_flag = False
#             counter = 0
#             text5 = "#2$Crane@Safe&30.28876,-97.73623!"
#             text4 = " $Car@Safe&30.28876,-97.73623!"
#             text6 = " $Dozer@Dang&30.28966,-97.73623!"
#             while (counter < 10):
#                 #clientsocket.init()
#                 clientsocket.send(text5.encode('utf-8'))
#                 clientsocket.recv(1024).decode('utf-8')
#                 #time.sleep(0.4)
#                 # 17, 27
#
#                 new = list(text5)
#                 if (counter == 2):
#                     new[9] = "C"
#                     new[10] = "a"
#                     new[11] = "u"
#                     new[12] = "t"
#                 if (counter == 6):
#                     new[1] = "3"
#                     new[9] = "D"
#                     new[10] = "a"
#                     new[11] = "n"
#                     new[12] = "g"
#                 new[20] = str((counter + 1) % 10)
#                 # new[29] = str((counter + 1) % 10)
#                 text5 = ''.join(new)
#                 print(text5)
#                 #clientsocket.disconnect()
#
#                 #clientsocket.init()
#                 clientsocket.send(text4.encode('utf-8'))
#                 clientsocket.recv(1024).decode('utf-8')
#                 #time.sleep(0.4)
#                 # 17, 27
#                 new = list(text4)
#                 if (counter == 2):
#                     new[6] = "C"
#                     new[7] = "a"
#                     new[8] = "u"
#                     new[9] = "t"
#                 if (counter == 6):
#                     new[6] = "D"
#                     new[7] = "a"
#                     new[8] = "n"
#                     new[9] = "g"
#                 new[17] = str((counter + 1) % 10)
#                 new[27] = str((counter + 1) % 10)
#                 text4 = ''.join(new)
#                 print(text4)
#                 #clientsocket.disconnect()
#
#                 if counter >= 7:
#                     #clientsocket.init()
#                     clientsocket.send(text6.encode('utf-8'))
#                     clientsocket.recv(1024).decode('utf-8')
#                     #time.sleep(0.4)
#
#                     new = list(text6)
#
#                     # new[19] = str((counter + 1) % 10)
#                     new[29] = str((counter + 1) % 10)
#                     text6 = ''.join(new)
#                     print(text6)
#                     #clientsocket.disconnect()
#
#                 counter = counter + 1
#                 print(counter)
#         else:
#            # clientsocket.init()
#             # Format:$ ITEM @ ThREAT LEVEL & COORDS
#             if (counter % 3) == 0:
#
#                 clientsocket.send(text.encode('utf-8'))
#                 clientsocket.recv(1024).decode('utf-8')
#                 # 17, 27
#                 new = list(text)
#                 new[18] = str((counter + 1) % 10)
#                 new[28] = str((counter + 1) % 10)
#                 text = ''.join(new)
#                 print(text)
#             elif (counter % 2) == 0:
#
#                 clientsocket.send(text3.encode('utf-8'))
#                 clientsocket.recv(1024).decode('utf-8')
#                 # 17, 27
#                 new = list(text3)
#                 new[20] = str((counter + 1) % 10)
#                 new[30] = str((counter + 1) % 10)
#                 text3 = ''.join(new)
#                 print(text3)
#             else:
#                 clientsocket.send(text2.encode('utf-8'))
#                 clientsocket.recv(1024).decode('utf-8')
#                 # 17, 27
#                 new = list(text2)
#                 new[20] = str((counter + 1) % 10)
#                 new[30] = str((counter + 1) % 10)
#                 text2 = ''.join(new)
#                 print(text2)
#
#             #clientsocket.disconnect()
#   '          counter = counter + 1

class Socket_connection():
    def prelim_init(self):

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a socket object
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #self.s = socket.socket()  # Create a socket object
        self.iterations = 0
        #self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = socket.gethostname()  # Get local machine name
        # host = "192.168.137.1"
        print(self.host)
        self.port = 8000  # Reserve a port for your service.
        # self.s.bind(('', self.port))  # Bind to the port
        self.s.bind(('', TCP_PORT))
        #self.s.bind(('', self.port))
        # self.f = open('torecv.jpg', 'wb')
        self.s.listen(5)  # Now wait for client connection.
        print(self.port)
        print("got addr")
        while True:
            self.c, self.addr = self.s.accept()  # Establish connection with client.
            print("got c")
            start_new_thread(client_thread,(self.c,self.addr))
            #break

    def init(self):

        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)  # Create a socket object
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #self.s = socket.socket()  # Create a socket object
        #self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = socket.gethostname()  # Get local machine name
        # host = "192.168.137.1"
        print(self.host)
        # self.port = 12345  # Reserve a port for your service.
        # self.s.bind(('', self.port))  # Bind to the port
        print(self.port)
        self.s.bind(('', TCP_PORT))

        # self.f = open('torecv.jpg', 'wb')
        self.s.listen(5)  # Now wait for client connection.

        print("got addr")

        while True:

            self.c, self.addr = self.s.accept()  # Establish connection with client.
            print("got c")
            break


    def send_data(self, msg):
        print('Got connection from', self.addr)
        print("Receiving...")
        # self.l = self.c.recv(1024)
        # msg = str.encode(msg, '
        self.c.send(msg.encode('utf-8'))

    def get_data(self):
        print('Got connection from', self.addr)
        print("Receiving...")
        self.l = self.c.recv(1024).decode('utf-8')
        # self.c.send(msg.encode('utf-8'))
        if self.l is not None:
            print(self.l)
            return self.l
        print("Ya messed up in Ethernet")
        while (self.l):
            print("Receiving...")
            # self.f.write(l)
            self.l = self.c.recv(1024)
            if self.l is not None:
                print(self.l)
                return self.l
            # Error-checking code, might cause an error, just know that means Im cool if you remove it
            print(self.l)
            # self.f.close()
            # print("Done Receiving")

    def disconnect(self):
        try:
            self.c.close()
            #self.port = self.port + 1
            #if self.port >= 12400:
            #    self.port = self.port - 2400
        except:
            pass

connection = Socket_connection()


connection.prelim_init()
# connection.send_data("$CAR@DANG&30.28876,-97.73623!")
# connection.disconnect()
counter = 0

text = "#$Car@Dang&30.28876,-97.73623!"
text2 ="#$Crane@Safe&30.28876,-97.73623!"
text3 ="#$Dozer@Caut&30.28876,-97.73623!"

second_protocol_flag = False
third_protocol_flag = True

connection.disconnect()

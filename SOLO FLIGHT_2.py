import dataclasses
import threading

from piosdk.piosdk import Pioneer
from edubot_sdk.edubot_sdk import EdubotGCS
import math
import time


@dataclasses.dataclass
class IpPort:
    ip: str
    port: int


class DroneConnectingData:
    drone0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    drone1: IpPort = IpPort(ip="127.0.0.1", port=8001)

class botConnectingData:
    bot0: IpPort = IpPort(ip="127.0.0.1", port=8002)
    bot1: IpPort = IpPort(ip="127.0.0.1", port=8003)

drones =  []
drones.append(Pioneer(method=2, ip=DroneConnectingData.drone0.ip, mavlink_port=DroneConnectingData.drone0.port))
drones.append(Pioneer(method=2, ip=DroneConnectingData.drone1.ip, mavlink_port=DroneConnectingData.drone1.port))


bots =  []
bots.append(EdubotGCS(ip=botConnectingData.bot0.ip, mavlink_port=botConnectingData.bot0.port))
bots.append(EdubotGCS(ip=botConnectingData.bot1.ip, mavlink_port=botConnectingData.bot1.port))


# class RobotConnectingData:
#     robot0: IpPort = IpPort(ip="127.0.0.1", port=8004)
#     robot1: IpPort = IpPort(ip="127.0.0.1", port=8005)



# код программы...

@dataclasses.dataclass
class Point:
    x: float
    y: float




class FlightPlanner:

    @classmethod
    def create_snake_traectory(cls, start_point: Point, fin_point: Point, num_x_point: int = 2, num_y_point: int = 2):

        traektory = []

        dx = (fin_point.x - start_point.x) / num_x_point
        dy = (fin_point.y - start_point.y) / num_y_point

        traektory.append(start_point)
        for x_point in range(num_x_point):
            for y_point in range(num_y_point):
                last_point = traektory[-1]
                if x_point % 2 == 0:
                    y = last_point.y + dy
                else:
                    y = last_point.y - dy

                traektory.append(Point(last_point.x, y))

            last_point = traektory[-1]
            x = last_point.x + dx
            traektory.append(Point(x, last_point.y))

        return traektory

    @staticmethod
    def checkDist(p1, p2, dist):
        d = math.dist(p1, p2)
        return True if d <= dist else False


tr = []
tr.append(FlightPlanner.create_snake_traectory(Point(-4, -4), Point(0, 4), 8, 10))
tr.append(FlightPlanner.create_snake_traectory(Point(0, -4), Point(4, 4), 8, 10))

print(tr)
# rock = Point(x=0.5,y=3.5)


# robot = EdubotGCS(ip=RobotConnectingData.robot0.ip, mavlink_port=RobotConnectingData.robot0.port)

fire_point = [0,0,0]
fire_points = []
def_fire = []
botp = [
    [0,0,0],
    [1,1,1]
]


def search(dnum):
    global fire_point, fire_points
    dnum = int(dnum)
    drones[dnum].arm()
    drones[dnum].takeoff()
    newPoint = True
    i = 0
    pos = None

    while True:
        global fire_point, fire_points
        if newPoint:
            drones[dnum].go_to_local_point(tr[dnum][i].x, tr[dnum][i].y, 1)
            i += 1
            if i >= len(tr[dnum]):
                i = 0

            newPoint = False

        if drones[dnum].point_reached():
            newPoint = True

        p = drones[dnum].get_local_position_lps()
        if p is not None:
            pos = p

        temp = drones[dnum].get_piro_sensor_data()

        if temp is not None:
            if temp >= 40 and not FlightPlanner.checkDist(fire_point[:2], pos[:2], 0.5):
                drones[dnum].fire_detection()
                fire_point = pos
                fire_points.append(fire_point)
                # drones[dnum].go_to_local_point(fire_point[0], fire_point[1], 1)
                print(fire_points)
                time.sleep(5)

                drones[dnum].go_to_local_point(pos[0], pos[1], 1)
                drones[dnum].point_reached()
                if dnum == 1:
                    drones[dnum].go_to_local_point(tr[dnum][i - 1].x, tr[dnum][i - 1].y, 1)
                else:
                    drones[dnum].go_to_local_point(tr[dnum][i - 1].x, tr[dnum][i - 1].y, 1.5)

    time.sleep(0.03)

bb = [False,False]

def detecting(bnum):
    bnum = int(bnum)
    global fire_point, fire_points, botp,bb
    pos = None
    while True:
        global fire_point, fire_points, botp,bb
        for fire in fire_points:
            p = bots[bnum].get_local_position_lps()
            if p is not None:
                pos = p
            bn = 0
            if bnum == 0:
                bn = 1

            if botp[bn][0] == fire_point[0]:
                bb[bnum] == False
                bots[bnum].go_to_local_point(x=pos[0], y=pos[1])

            time.sleep(0.5)

            p = bots[bnum].get_local_position_lps()
            if p is not None:
                pos = p

            print(botp[0], botp[1], fire_point)
            if bb[bnum] == False:
                botp[bnum] = [fire_point[0], fire_point[1], 0]
                bb[bnum] = True
                bots[bnum].go_to_local_point(x=fire_point[0], y=fire_point[1])
                if botp[0] == [1,1,1]:
                    dth2.start()

            if bots[bnum].point_reached():
                print("finished")
                bb[bnum] = False
                fire_points.pop(fire_points.index(fire))
                def_fire.append(fire_point)
                print(fire_points)
                bots[bnum].fire_detection()
                break



        # time.sleep(0.02)
        time.sleep(0.04)


th1 = threading.Thread(target=search, args=("0"))
th2 = threading.Thread(target=search, args=("1"))
dth1 = threading.Thread(target=detecting, args=("0"))
dth2 = threading.Thread(target=detecting, args=("1"))

th1.start()
th2.start()
dth1.start()
# dth2.start()

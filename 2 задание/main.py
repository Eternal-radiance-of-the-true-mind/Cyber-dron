import time
import threading
from piosdk.piosdk import Pioneer
from edubot_sdk.edubot_sdk import EdubotGCS
from sklearn.cluster import KMeans  # pip install scikit-learn
import dataclasses
from pymavlink import mavutil


# Классы для хранения настроек подключения
@dataclasses.dataclass
class IpPort:
    ip: str
    port: int


v = 2 # высота полета


# Координаты посадочных площадок
start_position_dron1 = [-0.41, -4.70]
start_position_dron2 = [-1.56, -4.72]
start_position_dron3 = [-2.76, -4.75]
start_position_dron4 = [-3.94, -4.72]

start_position_robot1 = [3.46, -4.61]
start_position_robot2 = [2.21, -4.67]

# Координаты сортировочных пунктов
punkt_green = [-1.05, 1.16]
punkt_yellow = [1.39, 1.19]
punkt_pink = [-1.05, -1.28]
punkt_blue = [1.39, -1.30]

#Координаты населенных пунктов
Nas_punkt1 = [-2.73, 2.73]
Nas_punkt2 = [2.9, 2.73]
Nas_punkt3 = [2.82, -2.85]
Nas_punkt4 = [-2.73, -2.79]


# Параметры для подключения дронов. В симуляторе смотрим номер порта и здесь смотрим какой объект класса соотвествует
# этим портам
class DroneConnectingData:
    drone0: IpPort = IpPort(ip="127.0.0.1", port=8004)
    drone1: IpPort = IpPort(ip="127.0.0.1", port=8005)
    drone2: IpPort = IpPort(ip="127.0.0.1", port=8002)
    drone3: IpPort = IpPort(ip="127.0.0.1", port=8003)


# Параметры для подключения РТС. В симуляторе смотрим номер порта и здесь смотрим какой объект класса соотвествует
# этим портам
class RobotConnectingData:
    robot0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    robot1: IpPort = IpPort(ip="127.0.0.1", port=8001)


drones = [Pioneer(ip=DroneConnectingData.drone0.ip, mavlink_port=DroneConnectingData.drone0.port),
          Pioneer(ip=DroneConnectingData.drone1.ip, mavlink_port=DroneConnectingData.drone1.port),
          Pioneer(ip=DroneConnectingData.drone2.ip, mavlink_port=DroneConnectingData.drone2.port),
          Pioneer(ip=DroneConnectingData.drone3.ip, mavlink_port=DroneConnectingData.drone3.port)]

robots = [EdubotGCS(ip=RobotConnectingData.robot0.ip, mavlink_port=RobotConnectingData.robot0.port),
          EdubotGCS(ip=RobotConnectingData.robot1.ip, mavlink_port=RobotConnectingData.robot1.port),]


def led_control(self, led_id=255, r=0, g=0, b=0):  # 255 all led
    if led_id not in [255, 0, 1, 2, 3]:
        raise ValueError(
            f"Argument 'led_id' must have one of the following values: 0, 1, 2, 3, 255. But your value is {led_id}.")
    if r < 0 or r > 255 or g < 0 or g > 255 or b < 0 or b > 255:
        raise ValueError(
            f"Arguments 'r', 'g', 'b' must have value in [0, 255]. But your values is r={r}, g={g}, b={b}.")
    return self._send_command_long(command_name='LED', command=mavutil.mavlink.MAV_CMD_USER_1,
                                   param1=led_id, param2=r, param3=g, param4=b,
                                   sending_log_msg=f'sending LED_ID={led_id} RGB=({r}, {g}, {b}) ...')


# Иницииализация дронов
def set_manual_speed(self, vx, vy, vz, yaw_rate):
    """ Set manual speed """
    cmd_name = 'MANUAL_SPEED'

    self.log(msg_type=cmd_name, msg=f'sending speed {{LOCAL, vx:{vx}, vy:{vy}, vz:{vz}, yaw_rate:{yaw_rate}}} ...')
    mask = 0b0000_01_0_111_000_111  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
    vx, vy, vz = vy, vx, -vz  # ENU coordinates to NED coordinates
    return self._send_position_target_local_ned(command_name=cmd_name,
                                                coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                mask=mask, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)


def get_center_clusters(data, num_clasters):
    """ Поиск кластеров в наборе данных """
    kmeans = KMeans(n_clusters=num_clasters, max_iter=100)
    kmeans.fit(data)
    return kmeans.cluster_centers_


def get_local_position_lps(self, get_last_received: bool = False):
    """ Get position LPS"""
    if 'LOCAL_POSITION_NED' in self.msg_archive:
        msg_dict = self.msg_archive['LOCAL_POSITION_NED']
        msg = msg_dict['msg']
        if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
            msg_dict['is_read'].set()
            return [msg.x, msg.y, msg.z]
        else:
            return None
    else:
        return None


def fly_to_point(point=None, point2=None, t=0, i=0):
    drones[i].arm()
    drones[i].takeoff()
    set_manual_speed(drones[i], vx=0.5, vy=0.5, vz=0.5, yaw_rate=0.5)
    flight = False
    while not flight:
        drones[i].go_to_local_point(x=point[0], y=point[1], z=2)
        time.sleep(t)
        if point2 != None:
            drones[i].go_to_local_point(x=point2[0], y=point2[1], z=2)
            time.sleep(t)
        break

    for high in range(2, 0, -1):
        if point2 != None:
            drones[i].go_to_local_point(x=point2[0], y=point2[1], z=high)
        else:
            drones[i].go_to_local_point(x=point[0], y=point[1], z=high)
        time.sleep(2)
    drones[i].land()
    drones[i].disarm()
    time.sleep(5)


def dron1_task():
    fly_to_point(punkt_green, t=15, i=0)
    fly_to_point(Nas_punkt1, t=5, i=0)
    fly_to_point(punkt_green, t=5, i=0)
    fly_to_point(Nas_punkt2, t=15, i=0)
    fly_to_point(punkt_yellow, t=5, i=0)
    fly_to_point(Nas_punkt1, t=15, i=0)
    fly_to_point(punkt_green, t=5, i=0)
    fly_to_point(Nas_punkt2, Nas_punkt3, t=15, i=0)


def dron2_task():
    fly_to_point(punkt_yellow, t=15, i=1)
    fly_to_point(Nas_punkt2, t=5, i=1)
    fly_to_point(punkt_yellow, t=5, i=1)
    fly_to_point(Nas_punkt3, t=15, i=1)
    fly_to_point(punkt_blue, t=5, i=1)
    fly_to_point(Nas_punkt2, t=15, i=1)
    fly_to_point(punkt_yellow, t=5, i=1)
    fly_to_point(Nas_punkt3, Nas_punkt4, t=15, i=1)


def dron3_task():
    fly_to_point(punkt_blue, t=15, i=2)
    fly_to_point(Nas_punkt3, t=5, i=2)
    fly_to_point(punkt_blue, t=5, i=2)
    fly_to_point(Nas_punkt4, t=15, i=2)
    fly_to_point(punkt_pink, t=5, i=2)
    fly_to_point(Nas_punkt3, t=15, i=2)
    fly_to_point(punkt_blue, t=5, i=2)
    fly_to_point(Nas_punkt4, Nas_punkt1, t=15, i=2)


def dron4_task():
    fly_to_point(punkt_pink, t=15, i=3)
    fly_to_point(Nas_punkt4, t=5, i=3)
    fly_to_point(punkt_pink, t=5, i=3)
    fly_to_point(Nas_punkt1, t=15, i=3)
    fly_to_point(punkt_green, t=5, i=3)
    fly_to_point(Nas_punkt4, t=15, i=3)
    fly_to_point(punkt_pink, t=5, i=3)
    fly_to_point(Nas_punkt1, Nas_punkt2, t=15, i=3)




def robot1_task():
    pass


def robot2_task():
    pass


if __name__ == '__main__':
    # Создаем поток для поиска пожаров и запускаем
    dron1_thread = threading.Thread(target=dron1_task)
    dron2_thread = threading.Thread(target=dron2_task)
    dron3_thread = threading.Thread(target=dron3_task)
    dron4_thread = threading.Thread(target=dron4_task)
    robot1_thread = threading.Thread(target=robot1_task)
    robot2_thread = threading.Thread(target=robot2_task)
    dron1_thread.start()
    time.sleep(4)
    dron2_thread.start()
    time.sleep(4)
    dron3_thread.start()
    time.sleep(4)
    dron4_thread.start()
    time.sleep(4)
    #robot1_thread.start()
    #time.sleep(2)
    #robot2_thread.start()

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


centre_fire = []  # массив куда запишутся центры пожаров
start_position_dron2 = [-1.09, -4.76, 2]
start_position_dron1 = [-0.25, -4.76, 2]

# массив координат полета, где есть пожар (если координаты известны)
# mission_point = [
#     [-3.85, -1.06, 1.5],  # fire
#     [-0.86, -1.92, 1.5],
#     [2.02, -1.29, 1.5],
#     [1.23, 0.20, 1.5],
#     [0.68, 1.50, 1.5],  # fire
#     [-0.64, 2.53, 1.5],
#     [-2.39, 2.95, 1.5],
#     [3.40, 2.04, 1.5],
#     [2.28, 3.60, 1.5],
#     [-4.81, 4.36, 1.5],
# ]
flight_point_drone1 = [
    [0, 4, 2],
    [-3.7, -4, 2],
    [-3.7, 4, 2],
    [-3.2, 4, 2],
    [-3.2, -4, 2],
    [-2.5, -4, 2],
    [-2.5, 4, 2],
    [-1.8, 4, 2],
    [-1.8, -4, 2],
    [-1, -4, 2],
    [-1, 4, 2],
    [-0.3, 4, 2],
    [-0.3, -4, 2],
    [-0, -4, 2],
    [-0, 4, 2],
    [-1, -4.76, 2]
]

flight_point_drone2 = [
    [0.5, -4, 2],
    [0.5, 4, 2],
    [1, 4, 2],
    [1, -4, 2],
    [1.5, -4, 2],
    [1.5, 4, 2],
    [2, 4, 2],
    [2, -4, 2],
    [2.5, -4, 2],
    [2.5, 4, 2],
    [3, 4, 2],
    [3, -4, 2],
    [3.5, -4, 2],
    [3.5, 4, 2],
    [4, 4, 2],
    [0, -4.76, 2]
]

flag1 = False
flag2 = False
# Параметры для подключения дронов. В симуляторе смотрим номер порта и здесь смотрим какой объект класса соотвествует
# этим портам
class DroneConnectingData:
    drone0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    drone1: IpPort = IpPort(ip="127.0.0.1", port=8001)
    drone2: IpPort = IpPort(ip="127.0.0.1", port=8002)
    drone3: IpPort = IpPort(ip="127.0.0.1", port=8003)


# Параметры для подключения РТС. В симуляторе смотрим номер порта и здесь смотрим какой объект класса соотвествует
# этим портам
class RobotConnectingData:
    robot0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    robot1: IpPort = IpPort(ip="127.0.0.1", port=8001)
    robot2: IpPort = IpPort(ip="127.0.0.1", port=8006)
    robot3: IpPort = IpPort(ip="127.0.0.1", port=8007)


drones = [Pioneer(ip=DroneConnectingData.drone0.ip, mavlink_port=DroneConnectingData.drone0.port),
          Pioneer(ip=DroneConnectingData.drone1.ip, mavlink_port=DroneConnectingData.drone1.port),
          Pioneer(ip=DroneConnectingData.drone2.ip, mavlink_port=DroneConnectingData.drone2.port),
          Pioneer(ip=DroneConnectingData.drone3.ip, mavlink_port=DroneConnectingData.drone3.port)]

robots = [EdubotGCS(ip=RobotConnectingData.robot0.ip, mavlink_port=RobotConnectingData.robot0.port),
          EdubotGCS(ip=RobotConnectingData.robot1.ip, mavlink_port=RobotConnectingData.robot1.port),
          EdubotGCS(ip=RobotConnectingData.robot2.ip, mavlink_port=RobotConnectingData.robot2.port),
          EdubotGCS(ip=RobotConnectingData.robot3.ip, mavlink_port=RobotConnectingData.robot3.port)]


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
def fire_detection(self):
    """ Мигание красной индикацией """
    self.led_custom(mode=2, color1=[255, 0, 0], timer=5)


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


def check_fire(drone_number):
    while not drones[drone_number].point_reached():
        temp = get_piro_sensor_data(drones[drone_number])
        fire_coord = get_local_position_lps(drones[drone_number])
        if (temp is not None) and (temp > 40):
            print("Ура, мы нашли пожар!")
            print("Координаты пожара", fire_coord)
            time.sleep(4)
            if fire_coord is not None:
                time.sleep(1)
                centre_fire.append(fire_coord)
                print(centre_fire)
                led_control(drones[drone_number],r=200, g=12, b=12)
            else:
                print("Ignoring 'None' value")


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


def get_piro_sensor_data(self, get_last_received: bool = False):
    """ Возвращает температуру с пирометра """

    if 'DISTANCE_SENSOR' in self.msg_archive:
        msg_dict = self.msg_archive['DISTANCE_SENSOR']
        if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
            msg_dict['is_read'].set()
            msg = msg_dict['msg']

            if msg.id == 0 and msg.type == mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN:  # Если данные от пирометра
                current_temp = msg.current_distance
                return current_temp
            else:
                return None
        else:
            return None
    else:
        return None


def dron1_task():
    global centre_fire, flag1
    drones[2].arm()
    drones[2].takeoff()
    set_manual_speed(drones[2], vx=0.5, vy=0.5, vz=0.5, yaw_rate=0.5)
    flight = False
    while not flight:
        for i in flight_point_drone1:
            drones[2].go_to_local_point(x=i[0], y=i[1], z=i[2])
            check_fire(2)
        flight = True
        break
    drones[2].land()
    flag1 = True
    drones[2].disarm()
    print(centre_fire)


def dron2_task():
    global centre_fire, flag2
    drones[3].arm()
    drones[3].takeoff()
    set_manual_speed(drones[3], vx=0.5, vy=0.5, vz=0.5, yaw_rate=0.5)
    flight = False
    while not flight:
        for i in flight_point_drone2:
            drones[3].go_to_local_point(x=i[0], y=i[1], z=i[2])
            check_fire(3)
        flight = True
        break
    drones[3].land()
    flag2 = True
    drones[3].disarm()
    print(centre_fire)


def robot1_task():
    global centre_fire
    race = False
    while not race:
        for i in centre_fire:
            robots[0].go_to_local_point(x=i[0], y=i[1])
            fire_detection(robots[0])
        race = True
        break


def robot2_task():
    global centre_fire
    race = False
    while not race:
        for i in centre_fire:
            robots[1].go_to_local_point(x=i[0], y=i[1])
            fire_detection(robots[1])
        race = True
        break


if __name__ == '__main__':
    # Создаем поток для поиска пожаров и запускаем
    dron1_thread = threading.Thread(target=dron1_task)
    dron2_thread = threading.Thread(target=dron2_task)
    robot1_thread = threading.Thread(target=robot1_task)
    robot2_thread = threading.Thread(target=robot2_task)
    dron2_thread.start()
    time.sleep(2)
    dron1_thread.start()
    if flag1 and flag2:
        robot1_thread.start()
        time.sleep(2)
        robot2_thread.start()

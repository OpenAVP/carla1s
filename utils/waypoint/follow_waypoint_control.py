import math
import time
import random
import carla
import os
import numpy as np
from pure_pursuit import SimpleTest

class Follow_Waypoint_Controller:
    def __init__(self, threshold = 1, target_dis = 0.1):
        """加载初始化信息

        Args:
            threshold (float): 参考点最大间距
            target_dis (float): 参考点期望间距
        """
        self._threshold = threshold
        self._target_dis = target_dis

    @classmethod
    def calculate_yaws(self, x, y):
        """根据路径(x, y)坐标计算路径yaw角

        Args:
            x (np.ndarray): 路径x坐标.
            y (np.ndarray): 路径y坐标.

        Returns:
            yaws (list[float]): 路径yaw角.
        """
        yaws = []
        for i in range(len(x) - 1): 
            dx = x[i + 1] - x[i]
            dy = y[i + 1] - y[i]
            yaw = math.atan2(dx, dy)
            yaw = math.degrees(yaw)
            if yaw > 90:
                yaw = - (yaw - 90)
            else:
                if -90 <= yaw <= 90:
                    yaw = 90 - yaw
                else:
                    yaw = - yaw - 270
            yaws.append(yaw)
        return yaws 

    def save_track(self, new_path, save_path):
        with open(save_path, 'w') as file:
            for waypoint in new_path: 
                file.write(f"{waypoint[0]}, {waypoint[1]}, {np.degrees(waypoint[2])}\n")
        print(f"The driving path has been saved to {save_path}")

    def follow_track(self, data_path = "tf.npy", forward = True, create_ego_car = True, reference_points = [], map_name = '', final_track_save_path = '', draw = True):
        """根据保存的轨迹在Carla模拟器中重放车辆行使记录

        Args:
            data_path (str): 读取的轨迹保存路径.
            forward (bool): 轨迹中自车是否向前行驶.
            create_ego_car (bool): 是否需要创建自车.
            reference_points (List[(x, y)]): 输入轨迹.优先使用输入轨迹进行重放.
            map_name (str): 加载地图名称.如无输入则不在此加载地图.
            final_track_save_path (str): 最终轨迹保存路径.如无数入则不保存.
            draw (bool): 是否绘制行驶轨迹线.
        """
        dT = 0.02

        if not len(reference_points) == 0:

            # 有轨迹输入，采用轨迹输入
            temp_x = np.array([point[0] for point in reference_points])
            temp_y = np.array([point[1] for point in reference_points])
            z = np.array([0])

            # 考虑到参考路径点可能过于稀疏，进行上采样，以方便PurePursuit算法追踪
            path_length = 0
            x_ = []
            y_ = []
            for i in range(0,len(temp_x)-1):
                dis = math.sqrt((temp_x[i]-temp_x[i+1])*(temp_x[i]-temp_x[i+1])+(temp_y[i]-temp_y[i+1])*(temp_y[i]-temp_y[i+1]))
                path_length += dis
                if dis > self._threshold:
                    target_dis = self._target_dis
                    target_points_num = int(dis / target_dis) + 1

                    xp = np.array([temp_x[i],temp_x[i+1]])
                    yp = np.array([temp_y[i],temp_y[i+1]])

                    xn = np.linspace(temp_x[i], temp_x[i+1], target_points_num)
                    yn = np.interp(xn, xp, yp)

                    x_.extend(xn.tolist())
                    y_.extend(yn.tolist())
                else:
                    x_.append(temp_x[i])
                    y_.append(temp_y[i])

            new_len = len(x_)
            x = np.array(x_)
            y = np.array(y_)
            yaw = self.calculate_yaws(x,y)

        elif len(reference_points) == 0 and os.path.exists(data_path):

            # 无轨迹输入，且文件存在，采用文件数据
            transform, time_stamp = SimpleTest.read_data(data_path)

            # 使用插值法修正轨迹
            x, y, z, pitch, yaw, roll = SimpleTest.interpolate(transform, time_stamp, dT)
            new_len = x.shape[0]

            # 轨迹总长度
            path_length = 0
            for i in range(0,new_len-1):
                path_length += math.sqrt((x[i]-x[i+1])*(x[i]-x[i+1])+(y[i]-y[i+1])*(y[i]-y[i+1]))
        else:

            # 无轨迹输入，且文件不存在，报错
            print("Failed to find reference path!")
            return

        # 使用PurePursuit算法修正轨迹
        path = SimpleTest.get_path(x,y)

        if forward == True:
            v = 1
        else:
            v = -1
        state = np.array([x[0], y[0], np.radians(yaw[0]), v]).astype(np.float64)
        test = SimpleTest(initial_state = state)
        new_path = test.offline_test(path, math.floor(path_length / (v * test.get_dt()))-1)

        if not len(final_track_save_path) == 0:
            self.save_track(new_path, final_track_save_path)

        x = [state[0] for state in new_path]
        y = [state[1] for state in new_path]
        yaw = [np.degrees(state[2]) for state in new_path]
        new_len = len(new_path)

        client = carla.Client('localhost', 2000)
        if not len(map_name) == 0:
            client.load_world(map_name)
        world = client.get_world()
        if create_ego_car == True:
            spawn_point = random.choice(world.get_map().get_spawn_points())
            vehicle_bp = world.get_blueprint_library().filter('*vehicle*').filter('vehicle.tesla.*')[0]
            ego_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        else:
            ego_vehicle = world.get_actors().filter('vehicle.*')[0]
        cur_location = carla.Location(x = x[0], y = y[0], z = z[0]-0.0035)

        world.get_spectator().set_transform(carla.Transform(cur_location+carla.Location(z=3),carla.Rotation(pitch=0)))

        for i in range(1,new_len):
            location = carla.Location(x = x[i], y = y[i], z = z[0]-0.0035)
            ego_vehicle.set_transform(carla.Transform(location,carla.Rotation(pitch=0,yaw=yaw[i],roll=0)))

            # velocity = carla.Vector3D(x = (x[i] - cur_location.x) / dT, y = (y[i] - cur_location.y) / dT, z = 0)
            # v = math.sqrt(velocity.x * velocity.x + velocity.y*velocity.y + velocity.z*velocity.z)
            if draw == True:
                world.debug.draw_line(cur_location + carla.Location(z = test.get_car_height() / 2), location + carla.Location(z = test.get_car_height() / 2), thickness=0.1, color=carla.Color(r = 0, g = 0, b = 255), life_time = 10)
            cur_location = location
            time.sleep(dT)

if __name__ == "__main__":
    fwc = Follow_Waypoint_Controller()
    fwc.follow_track() 



import random
import time
import math
import copy
import sys
import os
import logging
import threading
import pygame
import carla
import argparse
import numpy as np
import scipy.interpolate as scipy_interpolate

class DrawLineClass:
    def __init__(self,
                pic_size = 800,
                camera_scaling_param = 25/141,
                camera_init_height = 50,
                refresh_interval = 0.075,
                default_point_height = 0.5,
                view_height_variation_limit = 20,
                floor_height = 0,
                lam = 0.2, 
                c = 2, 
                L = 4.524, 
                H = 2.07642,
                dt = 0.1,
                threshold = 1,
                target_dis = 0.1,
                save_path = 'tf.npy',
                track_save_path = 'track.txt'
                ): 
        """初始化画线相机对象"""
        # 原有的初始化参数
        self.pic_size = pic_size
        self.pygame_size = {
            "image_x": self.pic_size,
            "image_y": self.pic_size
        }
        self._camera_scaling_param = camera_scaling_param
        self._camera_init_height = camera_init_height
        self._refresh_interval = refresh_interval
        self._default_point_height = default_point_height
        self._view_height_variation = 0.0
        self._min_view_height_veriation = -view_height_variation_limit
        self._max_view_height_veriation = view_height_variation_limit
        self._floor_height = floor_height
        self._drawing = True

        # 线程相关参数
        self.lock = threading.Lock()
        self.thread_hold = True
        self.line_points = None
        self.camera_transform = None
        self.world = None
        self.surface = None

        # 保存相关参数
        self.save_path = save_path
        self.track_save_path = track_save_path

    # 数据存取相关方法
    @staticmethod
    def read_data(read_path):
        """读取npy文件保存的车辆Transform和时间戳信息

        Args:
            read_path (str): .npy文件路径
        Returns:
            transform (np.ndarray): 车辆Transform信息,格式为(x, y, z, pitch, yaw, roll).
            time_stamp (np.ndarray): 车辆时间戳信息,格式为(frame, elapsed_seconds, delta_seconds, platform_timestamp).
        """
        data = np.load(read_path, allow_pickle=True)
        data = data.reshape(-1, 10)
        transform = data[:,:6]
        time_stamp = data[:,-4:]
        return transform, time_stamp

    def save_points(self, points, filename, axis_num):
        """保存参考路径到文件"""
        with open(filename, 'w') as file:
            for point in points:
                if axis_num == 3:
                    file.write(f"{point[0]}, {point[1]}, {point[2]}\n")
                if axis_num == 2:
                    file.write(f"{point[0]}, {point[1]}\n")

    def save_yaws(self, yaws, points, filename):
        """保存yaw角到文件"""
        with open(filename, 'w') as file:
            for i in range(len(yaws)): 
                file.write(f"{points[i].x}, {points[i].y}, {points[i].z}, {yaws[i]}\n")
            length = len(points) - 1
            file.write(f"{points[length].x}, {points[length].y}, {points[length].z}\n")
    
    #轨迹处理相关方法
    @staticmethod
    def interpolate(transform, time_stamp, dT, threshold = 10):
        """根据给定的时间间隔插值读取车辆Transform信息

        Args:
            transform (np.ndarray): 车辆Transform信息,格式为(x, y, z, pitch, yaw, roll).
            time_stamp (np.ndarray): 车辆时间戳信息,格式为(frame, elapsed_seconds, delta_seconds, platform_timestamp).
            dT (float): 读取数据时间间隔.

        Returns:
            x_interpolated (np.ndarray): 路径x坐标.
            y_interpolated (np.ndarray): 路径y坐标.
            z_interpolated (np.ndarray): 路径z坐标.
            pitch_interpolated (np.ndarray): 路径pitch角.
            yaw_interpolated (np.ndarray): 路径yaw角.
            roll_interpolated (np.ndarray): 路径roll角.
        """
        time_stamp = time_stamp[:,1]
        time_stamp[:] -= time_stamp[0]

        len = transform.shape[0]
        time_end = time_stamp[len-1]

        new_len = int(time_end / dT)
        new_time_stamp = np.arange(0, new_len * dT, dT)

        x = transform[:,0]
        y = transform[:,1]
        z = transform[:,2]
        pitch = transform[:,3]
        yaw = transform[:,4]
        roll = transform[:,5]

        x_interpolated = np.interp(new_time_stamp, time_stamp, x)
        y_interpolated = np.interp(new_time_stamp, time_stamp, y)
        z_interpolated = np.interp(new_time_stamp, time_stamp, z) - 0.05
        pitch_interpolated = np.interp(new_time_stamp, time_stamp, pitch)
        yaw_interpolated = np.interp(new_time_stamp, time_stamp, yaw)
        roll_interpolated = np.interp(new_time_stamp, time_stamp, roll)

        for i in range(1,new_len-1):
            if abs(yaw_interpolated[i] - yaw_interpolated[i-1]) > threshold and abs(yaw_interpolated[i] - yaw_interpolated[i+1]) > threshold:
                yaw_interpolated[i] = yaw_interpolated[i-1]

        return x_interpolated, y_interpolated, z_interpolated, pitch_interpolated, yaw_interpolated, roll_interpolated

    @staticmethod
    def get_path(x, y):
        """将x,y路径分量拼接成路径

        Args:
            x (np.ndarray): 路径x坐标.
            y (np.ndarray): 路径y坐标.

        Returns:
            (np.ndarray): 路径(x, y)坐标.
        """
        return np.concatenate((x.reshape(-1,1),y.reshape(-1,1)),axis=1)
    
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

    def convert_to_ego_car(self, line_points, ego_vehicle):
        """将参考点列表从Carla世界坐标系转换为主车坐标系

        Args:
            line_points (List[carla.Location[]): 需要移动的pygame参考点列表.
            ego_vehicle (carla.Vehicle): 自车.
            
        Returns:
            points_of_ego_vehicle_coordinate_system (List[(x, y)]): 基于自车坐标系的参考点位置列表(x, y).
        """
        ego_car_matrix = ego_vehicle.get_transform().get_matrix()
        ego_car_matrix = np.mat(ego_car_matrix)
        car_matrix = ego_car_matrix.I
        points_of_ego_vehicle_coordinate_system = []

        for point in line_points:
            car_P = np.dot(car_matrix, point)
            car_P = car_P[:2]
            points_of_ego_vehicle_coordinate_system.append((car_P[0,0], car_P[0,1]))

        return points_of_ego_vehicle_coordinate_system
    
    def calculate_new_position(self, pos):
        """计算视角高度变化后新的位置坐标"""
        return (
            (pos[0] - self.pic_size/2) * ((self._camera_init_height + self._view_height_variation)/self._camera_init_height) + self.pic_size/2,
            (pos[1] - self.pic_size/2) * ((self._camera_init_height + self._view_height_variation)/self._camera_init_height) + self.pic_size/2
        )
    
    def interpolate_b_spline_path(self, x, y, n_path_points, degree=3):
        """调用B样条函数实现曲线平滑

        Args:
            x (list[float]): 曲线x坐标列表.
            y (list[float]): 曲线y坐标列表.
            n_path_points (int): 插值曲线采样点数量.
            degree (int): B样条曲线阶数.

        Returns:
            spl_i_x(travel) (np.ndarray): 插值后的曲线x坐标列表.
            spl_i_y(travel) (np.ndarray): 插值后的曲线y坐标列表.
        """
        ipl_t = np.linspace(0.0, len(x) - 1, len(x))
        l,r = [(2,0.0)], [(2,0.0)]
        spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree, bc_type=(l,r))
        spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree, bc_type=(l,r))
        travel = np.linspace(0.0, len(x) - 1, n_path_points)
        return spl_i_x(travel), spl_i_y(travel)

    def interpolate_path(self, path, sample_rate, times):
        """路径插值"""
        if len(path) == 0:
            return path
        choices = np.arange(0, len(path), sample_rate)
        if len(path)-1 not in choices:
            choices = np.append(choices, len(path)-1)

        way_point_x = [path[index].x for index in choices]
        way_point_y = [path[index].y for index in choices]

        for i in range(times):
            way_point_x, way_point_y = self.interpolate_b_spline_path(way_point_x, way_point_y, len(path))
        new_path = np.vstack([way_point_x, way_point_y]).T
        return new_path

    def altitude_check(self, points, threshold_dz=1):
        """检查并平滑高度变化"""
        if len(points) < 3:
            return points

        smoothed_points = [
            carla.Location(x=points[i].x, y=points[i].y, 
                         z=(points[i-1].z + points[i+1].z) / 2)
            if i > 0 and i < len(points) - 1 and (
                abs(points[i].z - points[i-1].z) > threshold_dz and
                abs(points[i].z - points[i+1].z) > threshold_dz
            )
            else points[i]
            for i in range(len(points))
        ]
        return smoothed_points

    # 画线面板相关方法
    def pygame_callback(self, image):
        """相机传感器回调，将相机的原始数据重塑为 2D RGB,并应用于 PyGame 表面"""
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        img = img[:, :, :3]
        img = img[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))

    def move_line_points(self, line_points, axis, value):
        """键盘输入控制视角移动时，对应控制绘图数据移动

        Args:
            line_points (List[carla.Location[]): 需要移动的pygame参考点列表.
            axis (str): 在pygame中移动的方向.
            value (int): 在pygame中移动的像素数.

        Returns:
            line_points (List[carla.Location[]): 移动后的pygame参考点列表.
        """
        if axis == 'x':
            line_points = [carla.Location(point.x + value/self._camera_scaling_param, point.y, point.z) for point in line_points]
        elif axis == 'y':
            line_points = [carla.Location(point.x, point.y + value/self._camera_scaling_param, point.z) for point in line_points]
        return line_points

    def handle_keydown_event(self, event, line_points, camera_transform, ego_vehicle):
        """处理键盘按下事件"""
        if event.key == pygame.K_c:
            # 保存轨迹点
            carla_points = self.get_reference_points()
            self.save_points(carla_points, "point_in_carla.txt", 3)
            ego_car_points = self.convert_to_ego_car(carla_points, ego_vehicle)
            self.save_points(ego_car_points, "point_in_ego_car_system.txt", 2)
            logging.info("data saved.")
        
        elif event.key == pygame.K_h:
            # 重置视角高度
            self._view_height_variation = 0
            camera_transform.location.z = self._camera_init_height
        
        elif event.key == pygame.K_p:
            # 清除所有点
            line_points.clear()
        
        elif event.key == pygame.K_y:
            # 保存yaw角
            carla_points = self.get_reference_points()
            yaws = self.calculate_yaws(carla_points)
            self.save_yaws(yaws, line_points, "yaws.txt")
            logging.info("yaws saved.")
        
        elif event.key == pygame.K_x:
            # 切换是否绘制
            self._drawing = not self._drawing
            self.set_whether_draw(self._drawing)
            if self._drawing:
                logging.info("Start drawing lines in carla.")
            else:
                logging.info("Stop drawing lines in carla.")
        
        elif event.key == pygame.K_b:
            # 平滑路径
            interpolated_path = self.interpolate_path(line_points, sample_rate=5, times=3)
            final_interpolated_path = []

            for point in interpolated_path:
                z = self.get_z_coordinate(point, camera_transform) if self.use_z else self._floor_height
                new_point = carla.Location(point[0], point[1], z)
                final_interpolated_path.append(new_point)

            line_points.clear()
            line_points.extend(final_interpolated_path)
            line_points = self.altitude_check(line_points)

    def handle_mousewheel_event(self, event):
        """处理鼠标滚轮事件"""
        if event.y == -1:  # 向下滚动
            self._view_height_variation += 0.5
        else:  # 向上滚动
            self._view_height_variation -= 0.5
        self._view_height_variation = max(self._min_view_height_veriation, min(self._view_height_variation, self._max_view_height_veriation))

    # DrawInCarla相关方法
    def get_car_height(self):
        """获取车高
        
        Returns:
            _H (float): 车辆高度.
        """
        return self._H

    def set_whether_draw(self, draw):
        self._drawing = draw

    def draw_in_carla_thread(self):
        """实现绘图进程"""
        while self.thread_hold:
            with self.lock:
                camera_transform = self.camera_transform
                line_points = self.line_points
            if not (line_points is None or len(line_points) == 0 or camera_transform is None):
                carla_points = self.convert_PIL_points_to_carla(line_points, camera_transform)
                self.draw_in_carla(carla_points)
            time.sleep(self._refresh_interval)

    def start_draw_thread(self):
        self.thread = threading.Thread(target=self.draw_in_carla_thread)
        self.thread_hold = True
        self.thread.start()

    def stop_draw_thread(self):
        self.thread_hold = False
        self.thread.join()

    def get_reference_points(self):
        """获取参考点列表"""
        return self.convert_PIL_points_to_carla(self.line_points, self.camera_transform)

    def update_line_points(self, line_points):
        with self.lock:
            self.line_points = line_points

    def update_camera_transform(self, camera_transform):
        with self.lock:
            self.camera_transform = camera_transform

    def convert_PIL_points_to_carla(self, line_points, camera_transform):
        """将pygame坐标系转化为Carla坐标系"""
        result_points = []
        for point in line_points:
            x_0 = point.x
            y_0 = point.y
            x_1 = self.pic_size - y_0 - self.pic_size/2
            y_1 = x_0 - self.pic_size/2
            A = np.array([x_1,y_1])

            s_x = self._camera_scaling_param
            s_y = self._camera_scaling_param

            scale_matrix = np.array([[s_x, 0], 
                                   [0, s_y]])

            Location = camera_transform.location
            t_x = Location.x
            t_y = Location.y
            translation_matrix = np.array([[1, 0, t_x], 
                                         [0, 1, t_y], 
                                         [0, 0, 1]])

            angle = np.radians(camera_transform.rotation.yaw)
            rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], 
                                      [np.sin(angle), np.cos(angle)]])

            scaled_A = np.dot(scale_matrix, A)
            translated_A = np.dot(translation_matrix, np.append(scaled_A, 1))
            translated_A = translated_A[:2] 
            rotated_A = np.dot(rotation_matrix, translated_A)

            rotated_A = np.append(rotated_A, point.z)
            rotated_A = np.append(rotated_A, 1)
            
            result_points.append(rotated_A)

        return result_points
    
    def draw_in_carla(self, reference_points):
        """在Carla中绘制参考点和连线"""
        debug = self.world.debug

        if self._drawing:
            # 绘制参考点
            for point in reference_points:
                if self.use_z:
                    debug.draw_point(carla.Location(x=point[0], y=point[1], z=point[2]), 
                                   size=0.08, 
                                   color=carla.Color(r=255, g=0, b=0), 
                                   life_time=self._refresh_interval*2)
                else:
                    debug.draw_point(carla.Location(x=point[0], y=point[1], z=self._floor_height), 
                                   size=0.08, 
                                   color=carla.Color(r=255, g=0, b=0), 
                                   life_time=self._refresh_interval*2)
            
            # 绘制连线
            for i in range(len(reference_points) - 1):
                if self.use_z:
                    debug.draw_line(
                        carla.Location(x=reference_points[i][0], y=reference_points[i][1], z=reference_points[i][2]),
                        carla.Location(x=reference_points[i + 1][0], y=reference_points[i + 1][1], z=reference_points[i + 1][2]),
                        thickness=0.1,
                        color=carla.Color(r=255, g=0, b=0),
                        life_time=self._refresh_interval*2
                    )
                else:
                    debug.draw_line(
                        carla.Location(x=reference_points[i][0], y=reference_points[i][1], z=self._floor_height),
                        carla.Location(x=reference_points[i + 1][0], y=reference_points[i + 1][1], z=self._floor_height),
                        thickness=0.1,
                        color=carla.Color(r=255, g=0, b=0),
                        life_time=self._refresh_interval*2
                    )

    def is_label_valid(self, label):
        """判断语义标签是否合法

        Args:
            label (carla.CityObjectLabel): Carla语义标签.

            完整语义标签对照表可参考网址 : https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
            其中，
            6 : Poles
            7 : TrafficLight
            8 : TrafficSign
            如果label为invalid_labels中的任何一种,则说明标签不合法.

        Returns:
            label (bool): 标签是否合法.
        """
        invalid_labels = {6, 7, 8}
        return label not in invalid_labels

    def get_z_coordinate(self, point, camera_transform):
        """计算pygame坐标系下的点在Carla中的对应点高度

        Args:
            point (x, y): pygame坐标系下的点(x, y).
            camera_transform (carla.Transform): 相机transform矩阵.

        Returns:
            z (float): Carla中对应点高度.
        """
        points = []
        points.append(carla.Location(x=point[0], y=point[1], z=self._default_point_height))
        result = self.convert_PIL_points_to_carla(points, camera_transform)
        result_point = result[0]

        point_lists = self.world.cast_ray(
            carla.Location(result_point[0], result_point[1], self._camera_init_height),
            carla.Location(result_point[0], result_point[1], -5)
        )
        z = self._default_point_height
        if len(point_lists) != 0:
            for point in point_lists:
                if self.is_label_valid(point.label):
                    z = point.location.z + self._default_point_height
                    break
        return z

    def start_draw(self):
        """启动主程序"""
        # 连接到客户端并检索世界对象
        client = carla.Client('localhost', 2000)
        self.world = client.get_world()

        # 获取车辆并设置自动驾驶
        if actor_id == -1:
            # 获取地图的刷出点
            spawn_point = random.choice(self.world.get_map().get_spawn_points())
            vehicle_bp = self.world.get_blueprint_library().filter('*vehicle*').filter('vehicle.tesla.*')[0]
            ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        else:
            actor_list = self.world.get_actors()
            actor = actor_list.find(actor_id)
            if actor is None:
                logging.warning("ERROR ACTOR ID!")
                sys.exit()
            else:
                ego_vehicle = actor_list.find(actor_id)

        self.world.get_spectator().set_transform(
            carla.Transform(
                ego_vehicle.get_transform().location + carla.Location(z=self._camera_init_height),
                carla.Rotation(pitch=-90)
            )
        )

        # 生成摄像头
        image_size_x = int(self.pygame_size.get("image_x"))
        image_size_y = int(self.pygame_size.get("image_y"))
        camera_transform = carla.Transform(
            carla.Location(
                x=ego_vehicle.get_transform().location.x,
                y=ego_vehicle.get_transform().location.y,
                z=self._camera_init_height
            ), 
            carla.Rotation(pitch=-90.0, yaw=0, roll=0)
        )
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('fov', "110")
        camera_bp.set_attribute('image_size_x', str(image_size_x))
        camera_bp.set_attribute('image_size_y', str(image_size_y))
        camera = self.world.spawn_actor(camera_bp, camera_transform)

        # 采集carla世界中camera的图像
        camera.listen(lambda image: self.pygame_callback(image))
        camera_transform = camera.get_transform()

        # 将相机图像加载到pygame表面
        init_image = np.random.randint(0, 255, (self.pygame_size.get("image_y"), self.pygame_size.get("image_x"), 3), dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))

        # 初始化pygame显示
        pygame.init()
        gameDisplay = pygame.display.set_mode(
            (self.pygame_size.get("image_x"), self.pygame_size.get("image_y")),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )

        line_points = []
        drawing = False

        self.start_draw_thread()

        crashed = False
        while not crashed:
            # 等待同步
            self.world.tick()

            # 按帧更新渲染的 Camera 画面
            gameDisplay.blit(self.surface, (0, 0))

            # 处理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    crashed = True

                # 处理键盘事件
                key_list = pygame.key.get_pressed()
                if key_list[pygame.K_w]:
                    camera_transform.location.x += 1
                    line_points = self.move_line_points(line_points,'y',1)
                if key_list[pygame.K_s]:
                    camera_transform.location.x -= 1
                    line_points = self.move_line_points(line_points,'y',-1)
                if key_list[pygame.K_a]:
                    camera_transform.location.y -= 1
                    line_points = self.move_line_points(line_points,'x',1)
                if key_list[pygame.K_d]:
                    camera_transform.location.y += 1
                    line_points = self.move_line_points(line_points,'x',-1)

                if event.type == pygame.KEYDOWN:
                    self.handle_keydown_event(event, line_points, camera_transform, ego_vehicle)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    drawing = True
                elif event.type == pygame.MOUSEBUTTONUP:
                    drawing = False
                    line_points = self.altitude_check(line_points)
                elif event.type == pygame.MOUSEMOTION and drawing:
                    new_pos = self.calculate_new_position(event.pos)
                    z = self.get_z_coordinate(new_pos, camera_transform) if self.use_z else self._floor_height
                    point_xyz = carla.Location(new_pos[0], new_pos[1], z)
                    line_points.append(point_xyz)
                elif event.type == pygame.MOUSEWHEEL:
                    self.handle_mousewheel_event(event)
                    camera_transform.location.z = self._camera_init_height + self._view_height_variation

                self.update_line_points(line_points)
                self.update_camera_transform(camera_transform)
                camera.set_transform(camera_transform)

            pygame.display.flip()

        # 清理
        ego_vehicle.destroy()
        camera.stop()
        self.stop_draw_thread()
        pygame.quit()

    # 接口方法
    def draw_and_drive(self, actor_id=-1, use_z=False):
        """启动绘图窗口实现画线操作
        
        Args:
            actor_id (int): 要查找的车辆ID,-1表示创建新车辆
            use_z (bool): 是否考虑z轴高度
        """
        logging.info('Start draw and drive function...')

        self.use_z = use_z
        self.start_draw()  # 启动绘图窗口

    def get_reference_points_in_carla(self, format='list'):
        """获取参考点列表

        Args:
            format (str): 输出是什么格式.可选list格式或np.ndarray格式.
        Returns:
            reference_points (list/np.ndarray): 绘制的参考点列表,每个点用(x, y)表示
        """
        reference_points = self.get_reference_points()
        reference_points = [(point[0], point[1]) for point in reference_points]

        if format == 'list' or format == 'List':
            return reference_points
        elif format == 'ndarray' or format == 'np.ndarray' or format == 'np':
            return np.array(reference_points)

if __name__ == "__main__":
    logging.basicConfig(level=logging.NOTSET)

    parser = argparse.ArgumentParser()
    parser.add_argument('--actor_id', type=int, default=-1, help='ID of actor')
    parser.add_argument('--use_z', type=bool, default=False, help='Whether to enable Z-coordinate')
    parser.add_argument('--read_path', type=str, default='tf.npy', help='Read the trajectory from this path to play back')
    parser.add_argument('--save_path', type=str, default='tf.npy', help='Save path for the final trajectory')
    parser.add_argument('--track_save_path', type=str, default='track.txt', help='Path to save the track along the reference line in the scribing tool')
    args = parser.parse_args()
    actor_id = args.actor_id
    use_z = args.use_z

    # 创建draw_lines实例
    manager = DrawLineClass(save_path=args.save_path, track_save_path=args.track_save_path)

    # 启动绘图窗口并控制车辆
    manager.draw_and_drive(actor_id=-1, use_z=use_z)

    print(manager.get_reference_points_in_carla('list'))

    print(manager.get_reference_points_in_carla('ndarray'))

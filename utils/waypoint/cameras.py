import random
import math
import sys
import logging
import pygame
import carla
import argparse
import numpy as np
from follow_waypoint_control import Follow_Waypoint_Controller
import scipy.interpolate as scipy_interpolate
from draw_line_thread import DrawInCarlaThread

class Cameras:
    def __init__(self,
                world_name = 'Town01',
                pic_size = 800,
                camera_scaling_param = 25/141,
                camera_init_height = 50,
                refresh_interval = 0.075,
                default_point_height = 0.5,
                view_height_variation_limit = 20,
                floor_height = 0
                ): 
        """初始化画线相机对象

        Args:
            world_name (str): 需要加载的Carla地图名称.
            pic_size (int): 画幅尺寸.
            camera_scaling_param (float): pygame-carla初始图像缩放参数.
            camera_init_height (float): 相机初始高度.
            refresh_interval (float): 绘制刷新间隔.
            default_point_height (float): 默认绘制参考点高度.户外场景建议为50.
            view_height_variation_limit (float): 相机高度变化极限值.
            floor_height (float): 场景中的可行驶路面高度.考虑到carla无正交相机,高度变化会导致少许定位误差,因此默认使用场景中路面不存在高度差.
        """
        self.world_name = world_name
        self.pic_size = pic_size
        self.pygame_size = {
            "image_x": self.pic_size,
            "image_y": self.pic_size
        }

        # 在画线工具和Carla绘图子线程间共享数据
        self.params = {
            'Line_points':None,
            'Camera_transform':None
        }

        self._camera_scaling_param = camera_scaling_param

        self._camera_init_height = camera_init_height 

        self._refresh_interval = refresh_interval

        self._default_point_height = default_point_height

        # 视角高度变化量
        self._view_height_variation = 0.0
        self._min_view_height_veriation = -view_height_variation_limit
        self._max_view_height_veriation = view_height_variation_limit

        self._floor_height = floor_height

        # 是否在Carla中显示线条
        self._drawing = True

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
            points_of_ego_vehicle_coordinate_system.append((car_P[0,0],car_P[0,1]))

        return points_of_ego_vehicle_coordinate_system

    def save_points(self, points, filename, axis_num):
        with open(filename, 'w') as file:
            for point in points:
                if axis_num == 3:
                    file.write(f"{point[0]}, {point[1]}, {point[2]}\n")
                if axis_num == 2:
                    file.write(f"{point[0]}, {point[1]}\n")

    def save_yaws(self, yaws, points, filename):
        with open(filename, 'w') as file:
            for i in range(len(yaws)): 
                file.write(f"{points[i].x}, {points[i].y}, {points[i].z}, {yaws[i]}\n")
            length = len(points) - 1
            file.write(f"{points[length].x}, {points[length].y}, {points[length].z}\n")

    def calculate_yaws(self, points):
        yaws = []
        for i in range(len(points) - 1): 
            dx = points[i + 1][0] - points[i][0]
            dy = points[i + 1][1] - points[i][1]
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
        l,r=[(2,0.0)],[(2,0.0)]
        spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree,bc_type=(l,r))
        spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree,bc_type=(l,r))
        travel = np.linspace(0.0, len(x) - 1, n_path_points)
        return spl_i_x(travel), spl_i_y(travel)

    def interpolate_path(self, path, sample_rate, times):
        if len(path) == 0:
            return path
        choices = np.arange(0,len(path),sample_rate)
        if len(path)-1 not in choices:
            choices =  np.append(choices , len(path)-1)

        way_point_x = [path[index].x for index in choices]
        way_point_y = [path[index].y for index in choices]

        for i in range(times):
            way_point_x, way_point_y = self.interpolate_b_spline_path(way_point_x, way_point_y, len(path))
        new_path = np.vstack([way_point_x,way_point_y]).T
        return new_path

    def altitude_check(self, points, threshold_dz = 1):
        if len(points) < 3:
            return points

        smoothed_points = [
            carla.Location(x = points[i].x, y = points[i].y, z = (points[i - 1].z + points[i + 1].z) / 2)
            if i > 0 and i < len(points) - 1 and (
                abs(points[i].z - points[i - 1].z) > threshold_dz and
                abs(points[i].z - points[i + 1].z) > threshold_dz
            )
            else points[i]
            for i in range(len(points))
        ]
        return smoothed_points

    def start(self):

        # 连接到客户端并检索世界对象
        client = carla.Client('localhost', 2000)
        client.load_world(self.world_name)
        world = client.get_world()

        # 地下车库高度：-2.2～2.6

        # 获取车辆并设置自动驾驶
        if actor_id == -1:

            # 获取地图的刷出点
            spawn_point = random.choice(world.get_map().get_spawn_points())
            vehicle_bp = world.get_blueprint_library().filter('*vehicle*').filter('vehicle.tesla.*')[0]
            ego_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            # ego_vehicle.set_autopilot(True)
        else:
            actor_list = world.get_actors()
            actor = actor_list.find(actor_id)
            if actor == None:
                logging.warning("ERROR ACTOR ID!")
                sys.exit()
            else:
                ego_vehicle = actor_list.find(actor_id)

        world.get_spectator().set_transform(carla.Transform(ego_vehicle.get_transform().location+carla.Location(z=self._camera_init_height),carla.Rotation(pitch=-90)))

        # 生成摄像头
        image_size_x = int(self.pygame_size.get("image_x"))
        image_size_y = int(self.pygame_size.get("image_y"))
        camera_transform = carla.Transform(carla.Location(x=ego_vehicle.get_transform().location.x,y=ego_vehicle.get_transform().location.y,z=self._camera_init_height), 
                                        carla.Rotation(pitch=-90.0, yaw=0, roll=0))
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('fov', "110")
        camera_bp.set_attribute('image_size_x', str(image_size_x))
        camera_bp.set_attribute('image_size_y', str(image_size_y))
        camera = world.spawn_actor(camera_bp, camera_transform)

        # 采集carla世界中camera的图像
        camera.listen(lambda image: self.pygame_callback(image))
        camera_transform = camera.get_transform()

        # 将相机图像加载到pygame表面
        init_image = np.random.randint(0, 255, (self.pygame_size.get("image_y"), self.pygame_size.get("image_x"), 3), dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))

        # 初始化pygame显示
        pygame.init()
        gameDisplay = pygame.display.set_mode((self.pygame_size.get("image_x"), self.pygame_size.get("image_y")),pygame.HWSURFACE | pygame.DOUBLEBUF)

        line_points = []
        drawing = False

        draw_thread = DrawInCarlaThread(world, self._refresh_interval, self.pic_size, self._camera_scaling_param, self._camera_init_height, self._default_point_height, use_z, floor_height=self._floor_height)
        draw_thread.start()

        crashed = False

        while not crashed:

            # 等待同步
            world.tick()

            # 按帧更新渲染的 Camera 画面
            gameDisplay.blit(self.surface, (0, 0))

            # 获取 pygame 事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    crashed = True

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
                    if event.key == pygame.K_c:
                        carla_points = draw_thread.convert_PIL_points_to_carla(line_points, camera_transform)
                        self.save_points(carla_points, "point_in_carla.txt", 3)
                        ego_car_points = self.convert_to_ego_car(carla_points, ego_vehicle)
                        self.save_points(ego_car_points, "point_in_ego_car_system.txt", 2)
                        logging.info("data saved")
                    elif event.key == pygame.K_h:
                        self._view_height_variation = 0
                        camera_transform.location.z = self._camera_init_height
                    elif event.key == pygame.K_p:
                        line_points = []
                    elif event.key == pygame.K_y:
                        carla_points = draw_thread.convert_PIL_points_to_carla(line_points, camera_transform)
                        yaws = self.calculate_yaws(carla_points)
                        self.save_yaws(yaws,line_points,"yaws.txt")
                        logging.info("yaws saved")
                    elif event.key == pygame.K_x:
                        self._drawing = not self._drawing
                        draw_thread.set_whether_draw(self._drawing)
                        if self._drawing == True:
                            print("Start drawing lines in carla.")
                        else:
                            print("Stop drawing lines in carla.")
                    elif event.key == pygame.K_b:
                        interpolated_path = self.interpolate_path(line_points, sample_rate = 5, times = 3)
                        final_interpolated_path = []

                        for point in interpolated_path:
                            if use_z:
                                z = draw_thread.get_z_coordinate(point, camera_transform)
                            else:
                                z = self._floor_height
                            new_point = carla.Location(point[0], point[1], z)
                            final_interpolated_path.append(new_point)

                        line_points = final_interpolated_path
                        line_points = self.altitude_check(line_points)
                    elif event.key == pygame.K_e:

                        # 前进启动追踪
                        reference_points = draw_thread.get_reference_points()
                        fwc = Follow_Waypoint_Controller()
                        fwc.follow_track(forward = True, create_ego_car = False, reference_points = reference_points, final_track_save_path = args.save_path, draw = self._drawing) 

                    elif event.key == pygame.K_q:

                        # 倒车启动追踪
                        reference_points = draw_thread.get_reference_points()
                        fwc = Follow_Waypoint_Controller()
                        fwc.follow_track(forward = False, create_ego_car = False, reference_points = reference_points, final_track_save_path = args.save_path, draw = self._drawing)

                # 绘制线条
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    drawing = True
                elif event.type == pygame.MOUSEBUTTONUP:
                    drawing = False
                    line_points = self.altitude_check(line_points)
                elif event.type == pygame.MOUSEMOTION and drawing:
                    new_pos = ((event.pos[0] - self.pic_size/2)*((self._camera_init_height + self._view_height_variation)/self._camera_init_height) + self.pic_size/2 , 
                            (event.pos[1] - self.pic_size/2)*((self._camera_init_height + self._view_height_variation)/self._camera_init_height) + self.pic_size/2)
                    if use_z:
                        z = draw_thread.get_z_coordinate(new_pos, camera_transform)
                    else:
                        z = self._floor_height
                    point_xyz = carla.Location(new_pos[0], new_pos[1], z)
                    line_points.append(point_xyz)
                elif event.type == pygame.MOUSEWHEEL:

                    # 根据鼠标滚轮的方向调整缩放因子
                    if event.y == -1:  
                        self._view_height_variation += 0.5
                    else:  
                        self._view_height_variation -= 0.5
                    self._view_height_variation = max(self._min_view_height_veriation, min(self._view_height_variation, self._max_view_height_veriation))
                    camera_transform.location.z = self._camera_init_height + self._view_height_variation

                draw_thread.update_line_points(line_points)
                draw_thread.update_camera_transform(camera_transform)
                camera.set_transform(camera_transform)

            pygame.display.flip()

        ego_vehicle.destroy()
        camera.stop()
        draw_thread.stop()

        pygame.quit()

if __name__ == "__main__":
    logging.basicConfig(level=logging.NOTSET)

    parser = argparse.ArgumentParser()
    parser.add_argument('--actor_id', type=int, default=-1, help='ID of actor')
    parser.add_argument('--use_z', type=bool, default=False, help='Whether to enable Z-coordinate')
    parser.add_argument('--save_path', type=str, default='', help='Save path for the final trajectory')
    args = parser.parse_args()
    actor_id = args.actor_id
    use_z = args.use_z

    cameras = Cameras()
    cameras.start()

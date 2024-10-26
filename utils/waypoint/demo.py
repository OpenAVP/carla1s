from carla1s import CarlaContext, ManualExecutor
from carla1s.actors.vehicle import Vehicle

from carla1s.utils.waypoint import Waypoints

FIXED_DELTA_SECONDS = 0.1

with CarlaContext() as cc, ManualExecutor(cc, fixed_delta_seconds=FIXED_DELTA_SECONDS) as exe:
    cc.reload_world('Town01')
    
    
    ego_vehicle: Vehicle = (cc.actor_factory
        .create(Vehicle, from_blueprint='vehicle.tesla.model3')
        .with_name("ego_vehicle")
        .with_transform(cc.get_spawn_point(0))
        .build())
    
    cc.all_actors_spawn().all_sensors_listen()
    exe.wait_ticks(1)
    
    wpts = Waypoints.from_file(
        file_path='./waypoints.npy',
        delta_seconds=FIXED_DELTA_SECONDS,
        forward=True,
        keep_last=False,
    )

    tf = wpts[0]
    cc.world.get_spectator().set_transform(tf.as_carla_transform_obj())
    for tf in wpts:
        exe.tick()
        # print("type of tf:",type(tf))
        
        ego_vehicle.set_transform(tf.as_carla_transform_obj())
        cc.logger.info(f"Pose: {ego_vehicle.get_transform()}")


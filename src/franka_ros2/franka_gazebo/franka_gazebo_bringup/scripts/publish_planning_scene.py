#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        time.sleep(1.0)
        self.publish_scene()

    def create_box(self, id_name, dimensions, position):
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = id_name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        # Gazebo'daki nesnelerin boyutları [x, y, z] ve merkez konumları [x, y, z]
        # Masa: 0.8m derinlik, 0.6m genişlik, 0.4m yükseklik (Merkez Z = 0.2m)
        table = self.create_box('table', [0.8, 0.6, 0.4], [0.0, 0.0, 0.2])

        # Raflar: 0.5m x 0.4m, 1.2m yükseklik (Merkez Z = 0.6m)
        shelf1 = self.create_box('shelf_1', [0.5, 0.4, 1.2], [0.0, -1.2, 0.6])
        shelf2 = self.create_box('shelf_2', [0.5, 0.4, 1.2], [0.0, 1.2, 0.6])

        # Hedef Küp: 0.04m küp, Raf 1'in 2. katında (Z = 0.485m)
        box = self.create_box('target_box', [0.04, 0.04, 0.04], [0.0, -1.2, 0.485])

        scene.world.collision_objects = [table, shelf1, shelf2, box]

        for _ in range(5):
            self.scene_pub.publish(scene)
            self.get_logger().info('Çevre nesneleri RViz/MoveIt sahnesine aktarıldı.')
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Franka FR3 Duo Gripper Açma/Kapama Test Scripti
Canlı /joint_states geri bildirimiyle parmak hareketlerini ekrana anlık basar.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        self.left_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper_controller/commands', 10
        )
        self.right_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper_controller/commands', 10
        )
        self.current_left_pos = 0.0
        self.current_right_pos = 0.0
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_cb, 10
        )
        self.get_logger().info('Gripper test düğümü başlatıldı.')

    def joint_states_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == 'left_fr3v2_finger_joint1':
                self.current_left_pos = pos
            elif name == 'right_fr3v2_finger_joint1':
                self.current_right_pos = pos

    def hold_position(self, pos, duration_sec, arm='both'):
        """
        Belirtilen pozisyonu süre boyunca yayınlar ve canlı eklem değerini ekrana basar.
        """
        msg = Float64MultiArray()
        msg.data = [float(pos)]

        start_time = time.time()
        last_print = 0
        while (time.time() - start_time) < duration_sec:
            if arm in ('left', 'both'):
                self.left_pub.publish(msg)
            if arm in ('right', 'both'):
                self.right_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Her 0.5 saniyede bir canlı durumu terminale yaz
            if time.time() - last_print > 0.5:
                status_str = f"  -> [Canlı Simülasyon] Sol Parmak: {self.current_left_pos*1000:.1f} mm | Sağ Parmak: {self.current_right_pos*1000:.1f} mm"
                print(status_str)
                last_print = time.time()
                
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    tester = GripperTester()

    time.sleep(1.0)

    try:
        total_cycles = 3
        print("\n" + "=" * 65)
        print("   FRANKA FR3 DUO GRIPPER TESTİ (CANLI POZİSYON GERİ BİLDİRİMLİ)")
        print("   Hedef: 40.0 mm (TAM AÇIK) <---> 0.0 mm (TAM KAPALI)")
        print("=" * 65 + "\n")

        for cycle in range(1, total_cycles + 1):
            print(f"\n>>> [Döngü {cycle}/{total_cycles}] GRIPPER'LAR AÇILIYOR (Hedef: 40 mm / 0.04 m)...")
            tester.hold_position(pos=0.04, duration_sec=5.0, arm='both')

            print(f"\n>>> [Döngü {cycle}/{total_cycles}] GRIPPER'LAR KAPANIYOR (Hedef: 0 mm / 0.00 m)...")
            tester.hold_position(pos=0.00, duration_sec=5.0, arm='both')

        print("\n" + "=" * 65)
        print("   TEST TAMAMLANDI (Gripper'lar kapalı konuma getirildi)")
        print("=" * 65 + "\n")

    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu.")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

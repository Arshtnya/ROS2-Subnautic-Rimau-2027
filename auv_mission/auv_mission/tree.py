"""Menjalankan behaviour tree misi.

Struktur:
    rencana (Selector)
      ├── misi    : arm -> selam -> arah -> maju -> ... -> disarm
      └── darurat : stop -> naik -> disarm   (jalan kalau misi gagal)
"""
import rclpy
import py_trees
import py_trees_ros
from std_msgs.msg import Bool, Float32MultiArray

from .common import State
from .leaves import SetArmed, SetDepth, SetHeading, Gerak, Stop


def main():
    rclpy.init()
    node = rclpy.create_node("auv_mission")
    state = State()

    # Baca data robot
    def on_state(m):
        d = list(m.data) + [0.0] * 4   # heading, pitch, roll, depth
        state.heading, state.pitch, state.roll, state.depth = d[:4]

    def on_armed(m):
        state.armed = m.data
    
    def reset(m):
        if m.data:
            root.stop(py_trees.common.Status.INVALID)   # reset semua node ke awal
            tree.timer.reset()                          # nyalakan lagi tick (kalau tadi sudah berhenti)
            node.get_logger().info("Reset Ulang")

    node.create_subscription(Bool, "/auv/rst", reset, 10)
    node.create_subscription(Float32MultiArray, "/auv/state", on_state, 10)
    node.create_subscription(Bool, "/auv/armed", on_armed, 10)

    # Misi utama
    misi = py_trees.composites.Sequence("misi", memory=True)
    misi.add_children([
        SetArmed(node, state, True),
        # SetDepth(node, state, 0.8),
        # SetHeading(node, state, 0.0),
        Gerak(node, 6.0, surge=0.3),
        # SetHeading(node, state, 180.0),
        Gerak(node, 6.0, surge=0.3),
        Gerak(node, 3.0, sway=0.5),
        Stop(node),
        # SetDepth(node, state, 0.0),
        SetArmed(node, state, False),
    ])

    # Darurat (daun harus objek baru, jangan pakai ulang dari misi)
    darurat = py_trees.composites.Sequence("darurat", memory=True)
    darurat.add_children([
        Stop(node),
        SetDepth(node, state, 0.0),
        SetArmed(node, state, False),
    ])

    # Akar: coba misi dulu, kalau gagal baru darurat
    root = py_trees.composites.Selector("rencana", memory=True)
    root.add_children([misi, darurat])

    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(node=node, timeout=15.0)

    # Berhenti ngetick kalau sudah ada hasil akhir
    # def cek_selesai(t):
    #     if t.root.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
    #         node.get_logger().info("selesai: " + t.root.status.name)
    #         t.timer.cancel()

    # tree.add_post_tick_handler(cek_selesai)
    tree.tick_tock(period_ms=100.0)   # 10x per detik

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown(destroy_node=True)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
"""Merakit dan menjalankan behaviour tree misi kualifikasi (py_trees_ros).

Jalankan di robot:   ros2 run auv_mission mission
Uji tanpa robot:     ros2 launch auv_mission mission.launch.py   (fake=true)

Lihat live dengan viewer:
    sudo apt install ros-$ROS_DISTRO-py-trees-ros ros-$ROS_DISTRO-py-trees-ros-viewer
    py-trees-tree-viewer                        # terminal 1
    ros2 launch auv_mission mission.launch.py   # terminal 2

Struktur pohon:
    rencana (Selector = fallback)
      ├── misi   (Sequence): arm -> selam 0.8m -> arah 0 -> maju -> ...
      └── darurat(Sequence): stop -> naik ke 0m -> disarm
"""
import rclpy
import py_trees
import py_trees_ros
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32MultiArray

from .common import State
from .leaves import SetArmed, SetDepth, SetHeading, MoveSurge, Stop


def main():
    rclpy.init()
    node = rclpy.create_node("auv_mission")

    state = State()

    # ---------- telinga ----------
    def on_state(m):
        d = list(m.data) + [0.0, 0.0, 0.0, 0.0]   # heading, pitch, roll, depth
        state.heading, state.pitch = d[0], d[1]
        state.roll, state.depth = d[2], d[3]

    def on_armed(m):
        state.armed = m.data

    node.create_subscription(Float32MultiArray, "/auv/state", on_state, 10)
    node.create_subscription(Bool, "/auv/armed", on_armed, 10)

    # ---------- LANGKAH 1: misi utama (Sequence) ----------
    misi = py_trees.composites.Sequence("misi", memory=True)
    misi.add_children([
        SetArmed(node, state, True),
        SetDepth(node, state, 0.8),
        SetHeading(node, state, 0.0),
        MoveSurge(node, state, 0.3, 6.0),
        SetHeading(node, state, 180.0),
        MoveSurge(node, state, 0.3, 6.0),
        Stop(node, state),
        SetDepth(node, state, 0.0),
        SetArmed(node, state, False),
    ])

    misi2 = py_trees.composites.Sequence("misi2", memory=True)
    misi2.add_children([
        SetArmed(node, state, True),
        SetDepth(node, state, 0.8),
        SetHeading(node, state, 0.0),
        MoveSurge(node, state, 0.3, 6.0),
        Stop(node, state),
        SetDepth(node, state, 0.0),
        SetArmed(node, state, False),
    ])


    # ---------- LANGKAH 2: darurat (Sequence) ----------
    # Daun WAJIB benda baru (tak boleh dipakai di dua tempat).
    darurat = py_trees.composites.Sequence("darurat", memory=True)
    darurat.add_children([
        Stop(node, state),             # hentikan gerak
        SetDepth(node, state, 0.0),    # naik ke permukaan
        SetArmed(node, state, False),  # matikan
    ])

    # ---------- LANGKAH 3: akar = Selector (fallback) ----------
    # Coba `misi` dulu. Kalau gagal, pindah ke `darurat`.
    root = py_trees.composites.Selector("rencana", memory=True)
    root.add_children([misi, misi2, darurat])

    # ---------- bungkus jadi tree ROS (pakai node kita) ----------
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(node=node, timeout=15.0)

    try:
        node.set_parameters([
            Parameter("default_snapshot_stream", Parameter.Type.BOOL, True)])
    except Exception as e:
        node.get_logger().warn("gagal nyalakan snapshot stream: %s" % e)

    node.get_logger().info("Pohon misi:\n" + py_trees.display.unicode_tree(root))

    # ---------- LANGKAH 4: berhenti otomatis begitu ada hasil akhir ----------
    selesai = {"v": False}

    def cek_selesai(tree_obj):
        if selesai["v"]:
            return
        if tree_obj.root.status in (py_trees.common.Status.SUCCESS,
                                    py_trees.common.Status.FAILURE):
            selesai["v"] = True
            tree_obj.node.get_logger().info("selesai: " + tree_obj.root.status.name)
            if tree_obj.timer is not None:
                tree_obj.timer.cancel()

    tree.add_post_tick_handler(cek_selesai)

    # ---------- tick pohon 10x/detik ----------
    tree.tick_tock(period_ms=100.0)

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

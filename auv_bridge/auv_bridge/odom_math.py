"""Hitungan posisi robot (odometri) — murni matematika, tanpa ROS.

Robot tidak punya sensor kecepatan, jadi kita "tebak" posisi:
    posisi baru = posisi lama + (arah kompas & perintah gerak) x selang waktu
Ini namanya dead-reckoning; makin lama makin melenceng (drift) tapi cukup
buat pondasi. Dipisah dari bridge_node.py supaya gampang dites tanpa ROS.
"""
import math


def integrate(x, y, heading_deg, surge, sway, speed_scale, dt):
    """Tambah posisi (x, y) sejauh gerakan selama `dt` detik.

    x, y        : posisi sekarang (meter), frame odom (0,0 = titik start).
    heading_deg : arah hadap dari kompas (derajat, naik searah jarum jam).
    surge, sway : perintah gerak -1..1 (surge=maju, sway=kiri positif).
    speed_scale : kecepatan saat perintah = 1 (meter/detik).
    dt          : selang waktu (detik).
    """
    th = math.radians(-heading_deg)      # kompas searah jarum jam -> CCW (matematika)
    vx = surge * speed_scale             # kecepatan maju (sumbu x robot)
    vy = sway * speed_scale              # kecepatan samping (sumbu y robot, kiri +)
    x += (vx * math.cos(th) - vy * math.sin(th)) * dt
    y += (vx * math.sin(th) + vy * math.cos(th)) * dt
    return x, y

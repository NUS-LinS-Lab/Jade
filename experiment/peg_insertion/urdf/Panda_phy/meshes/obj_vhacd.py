import pybullet as p

p.connect(p.DIRECT)
name_in = "./collision/hand.obj"
name_out = "./collision/hand_vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log, alpha=0.04, resolution=50000)
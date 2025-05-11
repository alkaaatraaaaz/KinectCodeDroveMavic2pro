from controller import Robot
import socket
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Obtenir les moteurs (⚠ adapte les noms si différents dans Webots)
front_left = robot.getDevice("front left propeller")
front_right = robot.getDevice("front right propeller")
rear_left = robot.getDevice("rear left propeller")
rear_right = robot.getDevice("rear right propeller")
motors = [front_left, front_right, rear_left, rear_right]

# Préparation des moteurs
for m in motors:
    m.setPosition(float('inf'))  # mode vitesse
    m.setVelocity(0.0)

# Socket UDP pour recevoir les bits de la Kinect
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 8888))
sock.setblocking(False)

def set_motor_speeds(up=0.0, forward=0.0, lateral=0.0):
    base = 8.0  # poussée de base augmentée

    # Coefficients de vitesse
    up_gain = 3.0
    forward_gain = 2.0
    lateral_gain = 2.0

    front_left.setVelocity(base - forward * forward_gain + lateral * lateral_gain + up * up_gain)
    front_right.setVelocity(base - forward * forward_gain - lateral * lateral_gain + up * up_gain)
    rear_left.setVelocity(base + forward * forward_gain + lateral * lateral_gain + up * up_gain)
    rear_right.setVelocity(base + forward * forward_gain - lateral * lateral_gain + up * up_gain)

print("🟢 Contrôleur actif — en écoute sur UDP 127.0.0.1:8888...")

last_active = time.time()
bits = [0] * 6  # Valeur par défaut si aucun message reçu

while robot.step(timestep) != -1:
    # Réception des bits UDP
    try:
        while True:
            data, _ = sock.recvfrom(1024)
    except BlockingIOError:
        pass

    if 'data' in locals() and len(data) >= 6:
        b = [int(x) for x in data[:6]]
        print("Bits reçus :", b)

    # Mapping des bits aux mouvements
    up = 1.0 if bits[4] else (-1.0 if bits[5] else 0.0)
    forward = 1.0 if bits[0] else (-1.0 if bits[1] else 0.0)
    lateral = 1.0 if bits[3] else (-1.0 if bits[2] else 0.0)

    # Appliquer les vitesses
    set_motor_speeds(up, forward, lateral)

    # Sécurité : stop si plus de données depuis 3 sec
    if time.time() - last_active > 3:
        set_motor_speeds(0, 0, 0)

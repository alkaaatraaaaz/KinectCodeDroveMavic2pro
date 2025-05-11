from controller import Robot
import socket
import time
import struct

# Paramètres Webots
TIME_STEP = 32  # 32 ms = ~30 FPS
robot = Robot()

# Initialisation des moteurs
motor_names = ['front left propeller', 'front right propeller', 
               'rear left propeller', 'rear right propeller']
motors = []
for name in motor_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))  # mode vitesse
    motor.setVelocity(0.0)
    motors.append(motor)

# UDP : réception des commandes Kinect
UDP_IP = "127.0.0.1"
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # socket non bloquant

# État du drone
started = False
startup_counter = 0
startup_duration = 100  # 100 steps ≈ 3s

def interpret_data(data):
    if len(data) < 6:
        return [70] * 4  # défaut

    commands = [bool(b) for b in data[:6]]
    topLeft, left, bottomLeft, topRight, right, bottomRight = commands

    # Logique de contrôle
    if topLeft:
        return [80, 80, 80, 80]  # Décollage
    elif bottomLeft:
        return [60, 60, 60, 60]  # Atterrissage
    elif left:
        return [70, 50, 70, 50]  # Tourne gauche
    elif right:
        return [50, 70, 50, 70]  # Tourne droite
    elif topRight:
        return [90, 90, 90, 90]  # Avance rapide
    elif bottomRight:
        return [70, 70, 70, 70]  # Ralentit
    else:
        return [75, 75, 75, 75]  # Stable

# Boucle principale Webots
while robot.step(TIME_STEP) != -1:
    if not started:
        # Phase de démarrage
        setpoint = 75
        for motor in motors:
            motor.setVelocity(setpoint)
        startup_counter += 1
        if startup_counter >= startup_duration:
            started = True
            print("🚀 Drone prêt à recevoir les commandes Kinect.")
        continue

    # Réception de tous les paquets disponibles pour vider le buffer
    latest_data = None
    while True:
        try:
            data, addr = sock.recvfrom(128)
            latest_data = data
        except BlockingIOError:
            break  # plus rien à lire

    if latest_data:
        # Interpréter les données
        velocities = interpret_data(latest_data)
        for motor, vel in zip(motors, velocities):
            motor.setVelocity(vel)

        # Affichage latence si timestamp envoyé (8 octets en double float à la fin)
        if len(latest_data) >= 14:
            try:
                sent_time = struct.unpack('d', latest_data[6:14])[0]
                now = time.time()
                latency_ms = (now - sent_time) * 1000
                print(f"📡 Commande reçue - Latence : {latency_ms:.1f} ms")
            except:
                pass

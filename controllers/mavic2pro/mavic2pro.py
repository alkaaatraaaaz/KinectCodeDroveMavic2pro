from controller import Robot
import socket
import time
import sys
import math
import select

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class DroneController:
    # Constantes PID
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 60.0
    K_PITCH_P = 40.0

    # Paramètres de mouvement Kinect
    MOVEMENT_GAIN = 25.0
    MAX_ROLL_DISTURBANCE = 0.5
    MAX_PITCH_DISTURBANCE = 0.3
    MAX_YAW_DISTURBANCE = 0.4

    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Capteurs
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)

        # Moteurs
        self.motors = {
            'front_left': self.robot.getDevice("front left propeller"),
            'front_right': self.robot.getDevice("front right propeller"),
            'rear_left': self.robot.getDevice("rear left propeller"),
            'rear_right': self.robot.getDevice("rear right propeller")
        }
        for motor in self.motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Camera pitch
        self.camera_pitch_motor = self.robot.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.0)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 8888))
        self.sock.setblocking(False)

        # États internes
        self.bits = [0] * 6
        self.last_active = time.time()
        self.packets_received = 0
        self.signal_count = 0
        self.last_packet_time = time.time()
        self.current_pose = [0, 0, 0, 0, 0, 0]
        self.roll_disturbance = 0
        self.pitch_disturbance = 0
        self.yaw_disturbance = 0
        self.vertical_disturbance = 0
        self.target_altitude = 2.0

        print("🟢 Contrôleur Drone actif — UDP 127.0.0.1:8888...")
        print(f"Mode debug: {'ON' if debug_mode else 'OFF'}")
        print("-" * 100)
        if self.debug_mode:
            self.print_monitor_header()

    def print_monitor_header(self):
        print("\n📊 MONITORING KINECT:")
        print("┌─────────────────────────────────────────────┐")
        print("│  Zone Layout:    │  Signals Mapping:       │")
        print("│  [TL] [TR]       │  TL: Monter ↑           │")
        print("│  [L ] [R ]       │  L:  Gauche ←           │")
        print("│  [BL] [BR]       │  BL: Descendre ↓        │")
        print("│                  │  TR: Avancer →          │")
        print("│                  │  R:  Droite →           │")
        print("│                  │  BR: Reculer ←          │")
        print("└─────────────────────────────────────────────┘\n")

    def print_status_line(self):
        zone_names = ['TL', 'L', 'BL', 'TR', 'R', 'BR']
        active_zones = [zone_names[i] for i, bit in enumerate(self.bits) if bit]
        x, y, alt = self.current_pose[:3]
        status = (
            f"\rZones: {', '.join(active_zones) if active_zones else 'Aucune':<15} | "
            f"Alt: {alt:.2f}m | Pos: ({x:.1f}, {y:.1f}) | Paquets: {self.packets_received}"
        )
        sys.stdout.write(status + " " * 20)
        sys.stdout.flush()

    def print_full_monitor(self):
        if not self.debug_mode:
            return
        sys.stdout.write('\033[H\033[J')
        self.print_monitor_header()
        visual = self.visualize_kinect_zones()
        print("📡 ÉTAT KINECT EN TEMPS RÉEL:")
        print("=" * 60)
        for line in visual:
            print(line)
        print("=" * 60)
        x, y, alt, roll, pitch, yaw = self.current_pose
        print(f"\n🚁 ÉTAT DU DRONE:")
        print(f"Position: X={x:.2f}m, Y={y:.2f}m, Alt={alt:.2f}m")
        print(f"Orientation: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")
        print(f"\n🔧 DEBUG - Bits bruts: {self.bits}")
        print(f"Paquets reçus: {self.packets_received}")

    def visualize_kinect_zones(self):
        tl = "██" if self.bits[0] else "░░"
        tr = "██" if self.bits[3] else "░░"
        l = "██" if self.bits[1] else "░░"
        r = "██" if self.bits[4] else "░░"
        bl = "██" if self.bits[2] else "░░"
        br = "██" if self.bits[5] else "░░"
        return [
            "╔═══════════════════╗",
            f"║  {tl}     ║     {tr}  ║",
            f"║  TL     ║     TR  ║",
            "║         ║         ║",
            f"║  {l}     ║     {r}  ║",
            f"║  L      ║     R   ║",
            "║         ║         ║",
            f"║  {bl}     ║     {br}  ║",
            f"║  BL     ║     BR  ║",
            "╚═══════════════════╝"
        ]

    def update_pose(self):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        x, y, altitude = self.gps.getValues()
        self.current_pose = [x, y, altitude, roll, pitch, yaw]

    def process_kinect_data(self, data):
        if len(data) >= 6:
            self.bits = [int(b) for b in data[:6]]
            self.last_active = time.time()
            self.packets_received += 1
            self.signal_count += sum(self.bits)

            now = time.time()
            #print(f"⏱️ Intervalle entre paquets : {now - self.last_packet_time:.3f} s")
            self.last_packet_time = now

            self.vertical_disturbance = self.MOVEMENT_GAIN if self.bits[0] else -self.MOVEMENT_GAIN if self.bits[2] else 0
            self.roll_disturbance = -self.MAX_ROLL_DISTURBANCE if self.bits[1] else self.MAX_ROLL_DISTURBANCE if self.bits[4] else 0
            self.pitch_disturbance = self.MAX_PITCH_DISTURBANCE if self.bits[3] else -self.MAX_PITCH_DISTURBANCE if self.bits[5] else 0

    def compute_motor_speeds(self):
        roll, pitch, yaw = self.current_pose[3:6]
        altitude = self.current_pose[2]
        roll_acc, pitch_acc, _ = self.gyro.getValues()

        alt_error = self.target_altitude - altitude + self.K_VERTICAL_OFFSET
        clamped_error = clamp(alt_error, -1, 1)
        cubed_error = clamped_error ** 3

        roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acc + self.roll_disturbance
        pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acc + self.pitch_disturbance
        yaw_input = self.yaw_disturbance
        vertical_input = self.K_VERTICAL_P * cubed_error + self.vertical_disturbance

        fl = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        fr = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        rl = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
        rr = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

        self.motors['front_left'].setVelocity(fl)
        self.motors['front_right'].setVelocity(-fr)
        self.motors['rear_left'].setVelocity(-rl)
        self.motors['rear_right'].setVelocity(rr)

    def run(self):
        monitor_update_time = time.time()
        while self.robot.step(self.timestep) != -1:
            self.update_pose()

            # Traitement de tous les paquets reçus
            try:
                while True:
                    data, _ = self.sock.recvfrom(1024)
                    self.process_kinect_data(data)
            except BlockingIOError:
                pass

            self.compute_motor_speeds()

            if self.debug_mode:
                if time.time() - monitor_update_time > 0.5:
                    self.print_full_monitor()
                    monitor_update_time = time.time()
            else:
                self.print_status_line()

            if time.time() - self.last_active > 5:
                self.roll_disturbance = 0
                self.pitch_disturbance = 0
                self.yaw_disturbance = 0
                self.vertical_disturbance = 0

    def cleanup(self):
        for motor in self.motors.values():
            motor.setVelocity(0.0)
        self.sock.close()
        print("\n🔴 Contrôleur arrêté")

if __name__ == "__main__":
    DEBUG_MODE = True
    controller = DroneController(debug_mode=DEBUG_MODE)
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.cleanup()

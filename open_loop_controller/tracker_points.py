#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool

def normalize_angle_deg(angle):

    angle = (angle + 180) % 360 - 180
    return angle

class TrackerPoint(Node):
    def __init__(self):
        super().__init__("tracker_points")   
        self.get_logger().info("Turtle tracker node has started")

        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)

        self.sub = self.create_subscription(Pose, "/next_point", self.callback_points, 10)
        self.sub_finished = self.create_subscription(Bool, "/path_finished", self.callback_finished, 10)

        self.create_timer(0.01, self.state_machine)

        self.msg_arrived = Bool()

        self.declare_parameter("robot_pose", [5.0, 5.0, 0.0])
        self.declare_parameter("robot_speed", [0.2, 0.4])

        pose = self.get_parameter("robot_pose").get_parameter_value().double_array_value
        speed = self.get_parameter("robot_speed").get_parameter_value().double_array_value

        self.x_robot, self.y_robot, self.theta_robot = pose
        self.v, self.w = speed

        # Objetivo
        self.x_target = 0.0
        self.y_target = 0.0
        self.theta_target = 0.0  # en grados
        self.distance_target = 0.0

        # Temporizador y máquina de estados
        self.t0 = time.time()
        self.state = "state0"
        self.action_finished = False

    def callback_finished(self, msg):
        if msg.data:
            self.get_logger().info("Path generation finished")
            self.destroy_node()
            return

    def callback_points(self, msg):
        # Nueva posición objetivo
        self.x_target = msg.x
        self.y_target = msg.y

        # Distancia en lazo abierto
        self.distance_target = math.sqrt(
            (self.x_target - self.x_robot)**2 + 
            (self.y_target - self.y_robot)**2
        )

        # Ángulo deseado en radianes => a grados
        raw_angle_deg = math.degrees(
            math.atan2(
                self.y_target - self.y_robot, 
                self.x_target - self.x_robot
            )
        )
        # Normalizamos a [-180,180]
        self.theta_target = normalize_angle_deg(raw_angle_deg)

    def state_machine(self):
        """
        Máquina de estados sencilla:
          - state0: gira
          - state1: avanza
          - stop:  (sin usar en este ejemplo)
        """
        if self.action_finished:
            # Reinicia timer solo cuando acabas un paso
            self.t0 = time.time()
            self.action_finished = False
            # Cambia de estado
            if self.state == "state0":
                self.state = "state1"
            elif self.state == "state1":
                self.state = "state0"

        if self.state == "stop":
            return

        # Diferencia angular en grados, normalizada
        angle_diff = self.theta_target - self.theta_robot
        angle_diff = normalize_angle_deg(angle_diff)

        if self.state == "state0":
            # Gira la diferencia de ángulos
            self.turn(angle_diff)
        elif self.state == "state1":
            # Avanza la distancia deseada
            self.advance(self.distance_target)

    def advance(self, desired_distance):
        msg = Twist()
        # Tiempo transcurrido en este paso
        t = time.time() - self.t0
        distance_traveled = self.v * t

        if distance_traveled <= desired_distance:
            # Aún hay que seguir avanzando
            msg.linear.x = self.v
            self.pub.publish(msg)
            self.pub_cmd.publish(msg)
            self.msg_arrived.data = False
            self.pub_arrived.publish(self.msg_arrived)
        else:
            # Se cumplió (o superó) la distancia
            msg.linear.x = 0.0
            self.pub.publish(msg)
            self.pub_cmd.publish(msg)
            self.get_logger().info("Distance condition reached")

            # Actualizamos pose interna (en lazo abierto)
            self.x_robot = self.x_target
            self.y_robot = self.y_target
            self.theta_robot = self.theta_target

            # Indicar que acabamos y pasamos al siguiente estado
            self.action_finished = True
            self.msg_arrived.data = True
            self.pub_arrived.publish(self.msg_arrived)
    
    def turn(self, desired_angle_deg):
        """
        Gira en lazo abierto: 
          - Calculamos lo que hemos girado multiplicando la velocidad angular 
            (en grados/s) por el tiempo, y controlamos dirección según el signo.
        """
        msg = Twist()
        t = time.time() - self.t0

        # w rad/s => w*(180/pi) grados/s
        w_deg_s = self.w * (180.0 / math.pi)

        # Definir dirección del giro
        direction = 1.0 if desired_angle_deg > 0 else -1.0

        # Cuántos grados hemos girado hasta ahora (en lazo abierto)
        angle_traveled_deg = w_deg_s * t * direction

        # Publicar velocidad angular en rad/s, con signo adecuado
        msg.angular.z = direction * self.w

        self.get_logger().info(
            f"Turning: traveled {angle_traveled_deg:.2f}°, target {desired_angle_deg:.2f}°"
        )

        # Comparamos magnitudes
        if abs(angle_traveled_deg) <= abs(desired_angle_deg):
            # Seguimos girando
            self.pub.publish(msg)
            self.pub_cmd.publish(msg)
        else:
            # Ya cumplimos el giro
            msg.angular.z = 0.0
            self.pub.publish(msg)
            self.pub_cmd.publish(msg)
            self.get_logger().info("Angle condition reached")
            self.action_finished = True


def main(args=None):
    rclpy.init(args=args)
    nodeh = TrackerPoint()
    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

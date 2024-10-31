# coding=utf-8

from vpython import sphere, vector, rate, box, color, arrow, cylinder, compound, scene, label, canvas
import random
import time

# Константы
GRAVITY = vector(0, -9.81, 0)  # Вектор силы тяжести
DT = 0.01  # Шаг времени для симуляции
flight_phase = ""
scene.width = 1000
scene.height = 600
class IMUSensor:
    """Симулирует датчик IMU с акселерометром и гироскопом, включая погрешности измерений."""
    def __init__(self, drone):
        self.drone = drone

    def read_accelerometer(self):
        """Возвращает ускорение с некоторой погрешностью измерения."""
        error = vector(random.uniform(-0.1, 0.1),
                       random.uniform(-0.1, 0.1),
                       random.uniform(-0.1, 0.1))
        acceleration = self.drone.acceleration + error
        return acceleration

    def read_gyroscope(self):
        """Возвращает угловую скорость с некоторой погрешностью измерения."""
        error = vector(random.uniform(-0.01, 0.01),
                       random.uniform(-0.01, 0.01),
                       random.uniform(-0.01, 0.01))
        angular_velocity = self.drone.angular_velocity + error
        return angular_velocity

class LaserRangefinder:
    """Симулирует лазерный дальномер, измеряющий расстояние до пола."""
    def __init__(self, drone, floor):
        self.drone = drone
        self.floor = floor

    def read_distance(self):
        """Возвращает расстояние до пола."""
        distance = self.drone.position.y - self.floor.pos.y
        return max(distance, 0)

class GPS:
    """Симулирует точный GPS-датчик, предоставляющий точные координаты."""
    def __init__(self, drone):
        self.drone = drone

    def read_position(self):
        """Возвращает точное положение."""
        return self.drone.position

class Motor:
    """Представляет один мотор дрона."""
    def __init__(self, position, axis):
        # Визуальное представление мотора и пропеллера
        self.body = cylinder(pos=position, axis=axis, radius=0.02, color=color.gray(0.5))
        self.propeller = cylinder(pos=position + axis, axis=axis, radius=0.05, color=color.blue)
        self.thrust = vector(0, 0, 0)
        self.speed = 0  # Скорость мотора от 0 до 255

    def set_speed(self, speed):
        """Устанавливает скорость мотора, которая преобразуется в тягу."""
        self.speed = max(0, min(speed, 255))
        # Преобразование скорости (0-255) в тягу (0-20 Н)
        thrust_magnitude = (self.speed / 255.0) * 20
        self.thrust = vector(0, thrust_magnitude, 0)

    def update_visual(self, orientation):
        """Обновляет визуальные компоненты на основе ориентации дрона."""
        # Для простоты, здесь можно добавить вращение пропеллеров
        pass  # Заглушка для будущих визуальных обновлений

class Drone:
    """Представляет квадрокоптер дрона."""
    def __init__(self, initial_position):
        # Рама дрона
        arm_length = 0.2
        arm_thickness = 0.02

        # Создание четырех лучей
        arm1 = cylinder(pos=vector(0, 0, 0), axis=vector(arm_length, 0, arm_length),
                        radius=arm_thickness, color=color.red)
        arm2 = cylinder(pos=vector(0, 0, 0), axis=vector(-arm_length, 0, arm_length),
                        radius=arm_thickness, color=color.green)
        arm3 = cylinder(pos=vector(0, 0, 0), axis=vector(-arm_length, 0, -arm_length),
                        radius=arm_thickness, color=color.blue)
        arm4 = cylinder(pos=vector(0, 0, 0), axis=vector(arm_length, 0, -arm_length),
                        radius=arm_thickness, color=color.yellow)

        # Создание корпуса дрона
        body_sphere = sphere(pos=vector(0, 0, 0), radius=0.05, color=color.black)

        # Объединение всех частей в один объект
        self.visual = compound([arm1, arm2, arm3, arm4, body_sphere])
        self.visual.pos = initial_position

        self.position = initial_position
        self.velocity = vector(0, 0, 0)
        self.acceleration = vector(0, 0, 0)
        self.orientation = vector(0, 0, 0)  # Углы Эйлера: pitch, roll, yaw
        self.angular_velocity = vector(0, 0, 0)

        # Инициализация моторов на концах лучей
        self.motors = [
            Motor(position=initial_position + arm1.axis, axis=vector(0, 0.02, 0)),
            Motor(position=initial_position + arm2.axis, axis=vector(0, 0.02, 0)),
            Motor(position=initial_position + arm3.axis, axis=vector(0, 0.02, 0)),
            Motor(position=initial_position + arm4.axis, axis=vector(0, 0.02, 0)),
        ]

        # Датчики
        self.imu = IMUSensor(self)
        self.gps = GPS(self)
        self.laser = None  # Будет установлено после создания пола

        # Параметры PID-регулятора
        self.altitude_error_integral = 0
        self.previous_altitude_error = 0

        # Отладочные метки
        self.debug_label = label(pos=vector(0, 0, 0), text='', height=10, box=False, opacity=0)
        self.debug_label.visible = False  # Скрываем стандартную метку
        self.debug_text = ""

    def update_physics(self):
        """Обновляет физику дрона на основе тяги моторов и гравитации."""
        # Суммирование тяг от всех моторов
        total_thrust = vector(0, 0, 0)
        for motor in self.motors:
            # Поворот вектора тяги в соответствии с ориентацией дрона
            thrust = self.rotate_vector(motor.thrust, self.orientation)
            total_thrust += thrust

        # Вычисление результирующей силы
        net_force = total_thrust + GRAVITY

        # Обновление ускорения
        self.acceleration = net_force  # Предполагаем массу = 1 кг для простоты

        # Обновление скорости и положения
        self.velocity += self.acceleration * DT
        self.position += self.velocity * DT

        # Обновление угловой скорости и ориентации (упрощенно)
        # Здесь можно добавить расчеты момента для более реалистичной модели

        # Обновление визуального положения
        self.visual.pos = self.position

        # Обновление положений моторов
        for i, arm in enumerate([vector(0.2, 0, 0.2), vector(-0.2, 0, 0.2),
                                 vector(-0.2, 0, -0.2), vector(0.2, 0, -0.2)]):
            self.motors[i].body.pos = self.position + self.rotate_vector(arm, self.orientation)
            self.motors[i].propeller.pos = self.motors[i].body.pos + vector(0, 0.02, 0)

    def rotate_vector(self, vec, angles):
        """Поворачивает вектор на заданные углы Эйлера (pitch, roll, yaw)."""
        # Для простоты учитываем только поворот вокруг yaw
        from math import cos, sin, radians
        yaw = radians(angles.z)
        rotated_vec = vector(
            vec.x * cos(yaw) - vec.z * sin(yaw),
            vec.y,
            vec.x * sin(yaw) + vec.z * cos(yaw)
        )
        return rotated_vec

    def set_motor_speeds(self, pitch, roll, yaw, throttle):
        """
        Устанавливает скорости моторов на основе входов pitch, roll, yaw и throttle.
        Эти входы преобразуются в тяги для каждого мотора.
        """
        # Преобразование входов в скорости моторов (0 до 255)
        base_speed = throttle
        pitch_adjust = pitch
        roll_adjust = roll
        yaw_adjust = yaw

        # Расчет скоростей моторов
        motor_speeds = [
            base_speed + pitch_adjust - roll_adjust + yaw_adjust,  # Передний правый мотор
            base_speed + pitch_adjust + roll_adjust - yaw_adjust,  # Передний левый мотор
            base_speed - pitch_adjust + roll_adjust + yaw_adjust,  # Задний левый мотор
            base_speed - pitch_adjust - roll_adjust - yaw_adjust,  # Задний правый мотор
        ]

        # Установка скоростей моторов и обновление тяг
        for i, speed in enumerate(motor_speeds):
            self.motors[i].set_speed(speed)

    def maintain_altitude(self, target_altitude):
        """
        Тут вы должны написать свой пид регулятор, который будет записывать свое воздействие в переменную pid_output
        """

        #throttle = pid_output + 31 так должно быть

        throttle = 36 #временная затычка, дабы хоть как-то работало

        throttle = max(min(throttle, 255), 0)

        # Обновление скоростей моторов
        self.set_motor_speeds(pitch=0, roll=0, yaw=0, throttle=throttle)

    def update_debug_info(self):
        """Обновляет отладочную информацию, отображаемую на экране."""
        motor_speeds = [int(motor.speed) for motor in self.motors]
        position = self.position
        orientation = self.orientation

        debug_text = (
            f"Скорости моторов: {motor_speeds}\n"
            f"Положение: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}\n"
            f"Ориентация: Roll={orientation.x:.2f}, Pitch={orientation.y:.2f}, Yaw={orientation.z:.2f}"
        )
        self.debug_text = debug_text

def create_coordinate_axes():
    """Создает визуальные координатные оси в симуляции с отметками каждые 10 метров."""
    axis_length = 50
    axis_thickness = 0.2

    # X-ось (красная)
    arrow(pos=vector(-axis_length/2, 0, 0), axis=vector(axis_length, 0, 0),
          shaftwidth=axis_thickness, color=color.red)
    # Y-ось (зеленая) - под полом
    arrow(pos=vector(0, 0.1, 0), axis=vector(0, axis_length, 0),
          shaftwidth=axis_thickness, color=color.green)
    # Z-ось (синяя)
    arrow(pos=vector(0, 0, -axis_length/2), axis=vector(0, 0, axis_length),
          shaftwidth=axis_thickness, color=color.blue)


def main():
    """Основная функция для запуска симуляции."""
    # Установка белого фона
    scene.background = color.white

    # Создание пола с коллизией на y=0
    floor = box(pos=vector(0, 0, 0), size=vector(100, 0.1, 100), color=color.gray(0.9))

    # Создание координатных осей
    create_coordinate_axes()

    # Инициализация дрона на высоте y=0 метров
    drone = Drone(initial_position=vector(1, 0, 1))
    drone.laser = LaserRangefinder(drone, floor)

    # Перемещение отладочной информации в сторону
    debug_canvas = canvas(width=300, height=200, align='right', background=color.white)
    debug_label = label(pos=vector(0, 0, 0), text='', height=12, box=False, opacity=0, canvas=debug_canvas)

    # Временные переменные для управления полетом
    flight_phase = 'takeoff'  # 'takeoff', 'hover', 'landing', 'landed'
    hover_time = 0
    cnt = 0

    # Симуляционный цикл
    while True:
        rate(100)  # Ограничение симуляции до 100 итераций в секунду
        print(flight_phase, end = " ")
        # Чтение датчиков (для демонстрации)
        accelerometer_data = drone.imu.read_accelerometer()
        gyroscope_data = drone.imu.read_gyroscope()
        gps_position = drone.gps.read_position()
        altitude = drone.laser.read_distance()
        drone.maintain_altitude(5.0)

        # Обновление физики
        drone.update_physics()

        # Обновление отладочной информации
        drone.update_debug_info()
        debug_label.text = drone.debug_text

        # Обнаружение столкновения с полом
        if drone.position.y <= floor.pos.y + floor.size.y / 2:
            drone.velocity.y = 0
            drone.position.y = floor.pos.y + floor.size.y / 2

if __name__ == "__main__":
    main()

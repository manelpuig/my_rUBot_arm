# Driver Real per a Braç Robòtic 6 DOF

Aquest paquet permet controlar un braç robòtic real des de ROS 2 Humble seguint una arquitectura bioinspirada.

## 🚀 Instal·lació i Ús

### 1. Dependències (Raspberry Pi 4)
```bash
sudo apt update
sudo apt install python3-serial
````
### 2. Compilació
Dins del teu workspace de ROS 2:
````bash
colcon build --packages-select my_robot_arm_driver
source install/setup.bash
````

###3. Firmware
Obre l'Arduino IDE.

Carrega el fitxer arduino/arm_controller_sg90/arm_controller_sg90.ino a l'Arduino Nano ESP32.
Assegura't de tenir instal·lada la llibreria Servo.

### 4. Execució
Inicia el pont de comunicació:
````bash
ros2 run my_robot_arm_driver serial_bridge
````
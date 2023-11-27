import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import threading

class DronVisualizer(QWidget):
    def __init__(self):
        super().__init__()

        self.fig = plt.Figure()
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)

        self.setLayout(layout)

        self.roll, self.pitch, self.yaw = 0, 0, 0

        # Conectar a través de serial (ajusta el puerto y la velocidad según tus necesidades)
        self.serial_connection = serial.Serial('COM9', 115200, timeout=1)

        # Iniciar el hilo para la lectura en segundo plano
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        # Actualizar la figura en un bucle de temporizador
        self.update_timer = self.startTimer(50)  # Tiempo en milisegundos

    def read_serial(self):
        while True:
            try:
                # Leer datos desde el puerto serial (ajusta según el formato de tus datos)
                line = self.serial_connection.readline().decode('utf-8').strip()
                roll, pitch, yaw = map(float, line.split(','))

                # Actualizar los valores de roll, pitch y yaw
                self.roll = -np.radians(roll)
                self.pitch = -np.radians(pitch)
                self.yaw = -np.radians(yaw)


            except (serial.SerialException, ValueError):
                pass

    def draw_dron(self):
        # Limpiar la figura
        self.ax.cla()

        # Rotar el dron según los valores de roll, pitch y yaw
        rotation_matrix = self.rotation_matrix(self.roll, self.pitch, self.yaw)

        cuerpo = np.array([[-0.5, 0.5, 0.5, -0.5, -0.5],
                           [-0.5, -0.5, 0.5, 0.5, -0.5],
                           [0, 0, 0, 0, 0]])

        vertices = np.array([[-0.5, -0.5, 0],
                             [-0.5, 0.5, 0],
                             [-1, 0, 0]])

        # Utilizar la función generar_circulo para crear un círculo con diferentes parámetros
        circulo_params1 = [(0.5, 1, 1), (0.5, -1, -1)]

        for params in circulo_params1:
            radio, desplazamiento_x, desplazamiento_y = params
            circle_points = generar_circulo(radio, desplazamiento_x, desplazamiento_y)
            rotated_circle = np.dot(rotation_matrix, circle_points)
            self.ax.plot3D(rotated_circle[0], rotated_circle[1], rotated_circle[2], 'r-')

        circulo_params2 = [(0.5, 1, -1), (0.5, -1, 1)]

        for params in circulo_params2:
            radio, desplazamiento_x, desplazamiento_y = params
            circle_points = generar_circulo(radio, desplazamiento_x, desplazamiento_y)
            rotated_circle = np.dot(rotation_matrix, circle_points)
            self.ax.plot3D(rotated_circle[0], rotated_circle[1], rotated_circle[2], 'k-')

        rotated_triangle = np.dot(rotation_matrix, vertices.T).T
        #rotated_total = np.dot(rotation_matrix, total)
        rotated_cuerpo = np.dot(rotation_matrix, cuerpo)

        self.ax.plot3D(rotated_cuerpo[0], rotated_cuerpo[1], rotated_cuerpo[2], 'b-')
        self.ax.plot3D(rotated_triangle[[0, 1, 2, 0], 0],
                       rotated_triangle[[0, 1, 2, 0], 1],
                       rotated_triangle[[0, 1, 2, 0], 2], 'b-')

        # Establecer títulos de los ejes
        self.ax.set_xlabel('Eje X')
        self.ax.set_ylabel('Eje Y')
        self.ax.set_zlabel('Eje Z')

        # Establecer límites y relación de aspecto
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-1, 1])
        self.ax.set_box_aspect([1, 1, 1])

        # Actualizar la figura
        self.canvas.draw()

    def rotation_matrix(self, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])

        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))
        return rotation_matrix

    def timerEvent(self, event):
        # Dibujar el dron con la nueva orientación en cada evento de temporizador
        self.draw_dron()

    def closeEvent(self, event):
        # Detener el hilo y cerrar la conexión serial al cerrar la ventana
        self.read_thread.join()
        self.serial_connection.close()

def generar_circulo(radio, desplazamiento_x, desplazamiento_y, num_puntos=100):
    theta = np.linspace(0, 2 * np.pi, num_puntos)
    x = radio * np.cos(theta) + desplazamiento_x
    y = radio * np.sin(theta) + desplazamiento_y
    circle_points = np.array([x, y, np.zeros_like(x)])
    return circle_points

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow = QMainWindow()
    mainWindow.setGeometry(100, 100, 800, 600)

    centralWidget = DronVisualizer()
    mainWindow.setCentralWidget(centralWidget)

    mainWindow.show()
    sys.exit(app.exec_())

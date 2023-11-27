# Para inicializar una ventana en python de QT5 comandos basicos
from PyQt5.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QWidget, QPushButton, QDialog, QLabel
from PyQt5.uic import loadUi
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice, Qt, QSize, QTimer
from PyQt5.QtGui import QPixmap, QIcon
from collections import deque


from datetime import datetime

import pyqtgraph as pg
import numpy as np
import sys
import csv
import time


class MyApp(QMainWindow):

    def __init__(self):
        super(MyApp, self).__init__()
        loadUi('InterfazV4.ui', self)

        #Timer grafica
        # Create a QTimer
        self.timer = QTimer(self)
        # Connect the timeout signal to the graficar function
        self.timer.timeout.connect(self.graficar)
        # Set the timeout interval to 100 milliseconds (0.1 seconds)
        self.timer.start(20)

        self.Ver3D.clicked.connect(self.show_popup)

        # Define la imagen de fondo
        self.Imgfondo = QPixmap('FondoInterfazV6.png')
        self.Imgfondo = self.Imgfondo.scaled(self.Fondo.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Fondo.setPixmap(self.Imgfondo)

        # Inserción de Imagenes
        self.Drone = QPixmap('Drone.png')
        self.Drone = self.Drone.scaled(self.DroneImagen.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.DroneImagen.setPixmap(self.Drone)

        # Seleccion de Control
        self.Control_Menu.currentIndexChanged.connect(self.CambioControl)
        self.Control_Menu.addItem("Control Proporcional, Integral & Derivativo (PID)")
        self.Control_Menu.addItem("Control Regulador Lineal Cuadrático (LQR)")
        self.Control_Menu.addItem("Control de Superficie Deslizante (SMC)")

        self.nombre_archivo = None
        self.tiempo_inicial = None
        self.flag_graf = "0"
        self.flag_csv = "0"
        self.flag_check_R_Med = "0"
        self.flag_check_P_Med = "0"
        self.flag_check_Y_Med = "0"
        self.flag_check_R_Set = "0"
        self.flag_check_P_Set = "0"
        self.flag_check_Y_Set = "0"
        self.flag_check_LC1 = "0"
        self.flag_check_LC2 = "0"
        self.flag_check_LC3 = "0"
        self.flag_check_LC4 = "0"
        self.flag_ControlMenu = "0"
        self.flag_EnviarDatos = "0"
        self.flag_TipoSetpoint = "0"

        # Variables Medidas
        self.Roll_Med = 0
        self.Pitch_Med = 0
        self.Yaw_Med = 0
        self.M1_Med = 0
        self.M2_Med = 0
        self.M3_Med = 0
        self.M4_Med = 0
        self.acc = 0
        self.Val_Tiempo = 200
        self.nuevo_val_tiempo = 0

        # Ganancias PID
        self.Val_R_Kp = 2.5
        self.Val_P_Kp = 2.5
        self.Val_Y_Kp = 0.0
        self.Val_R_Ki = 0.07
        self.Val_P_Ki = 0.07
        self.Val_Y_Ki = 0.0
        self.Val_R_Kd = 30.0
        self.Val_P_Kd = 30.0
        self.Val_Y_Kd = 0.0
        self.Val_R_Set = 3.8
        self.Val_P_Set = -2.1
        self.Val_Y_Set = 0.0
        self.Val_R_Periodo = 0.0
        self.Val_P_Periodo = 0.0
        self.Val_Y_Periodo = 0.0

        self.Val_Porcentaje = 0.8

        self.R_Kp.valueChanged.connect(self.valor_cambiado)
        self.P_Kp.valueChanged.connect(self.valor_cambiado)
        self.Y_Kp.valueChanged.connect(self.valor_cambiado)
        self.R_Ki.valueChanged.connect(self.valor_cambiado)
        self.P_Ki.valueChanged.connect(self.valor_cambiado)
        self.Y_Ki.valueChanged.connect(self.valor_cambiado)
        self.R_Kd.valueChanged.connect(self.valor_cambiado)
        self.P_Kd.valueChanged.connect(self.valor_cambiado)
        self.Y_Kd.valueChanged.connect(self.valor_cambiado)
        self.R_Set.valueChanged.connect(self.valor_cambiado)
        self.P_Set.valueChanged.connect(self.valor_cambiado)
        self.Y_Set.valueChanged.connect(self.valor_cambiado)
        self.R_Periodo.valueChanged.connect(self.valor_cambiado)
        self.P_Periodo.valueChanged.connect(self.valor_cambiado)
        self.Y_Periodo.valueChanged.connect(self.valor_cambiado)
        self.Porcentaje.valueChanged.connect(self.valor_cambiado)

        # Botones Generales
        # Botón "Guardar"
        self.setup_button(self.Guardar, "Guardar1.png", "Guardar2.png", "Guardar3.png", 208, 40)

        # Botón "Reset"
        self.setup_button(self.Reset, "Reset1.png", "Reset2.png", "Reset3.png", 208, 42)

        # Botón "Calibración"
        self.setup_button(self.Calibrar, "Calibrar1.png", "Calibrar2.png", "Calibrar3.png", 208, 42)

        self.Guardar.clicked.connect(self.guardar_valores)
        self.Reset.clicked.connect(self.reset_valores)
        #self.Calibrar.clicked.connect(self.calibrar_sensor)
        self.TiempoBoton.clicked.connect(self.tiempo_valor)

        # Control Connect
        self.setup_button(self.Actualizar, "Actualizar1.png", "Actualizar2.png", "Actualizar3.png", 209, 23)

        # Botón "Reset"
        self.setup_button(self.Conectar, "Conectar1.png", "Conectar2.png", "Conectar3.png", 209, 23)

        # Botón "Calibración"
        self.setup_button(self.Desconectar, "Desconectar1.png", "Desconectar2.png", "Desconectar3.png", 209, 23)

        self.serial = QSerialPort()
        self.Actualizar.clicked.connect(self.read_ports)
        self.Conectar.clicked.connect(self.serial_connect)
        self.Desconectar.clicked.connect(lambda: self.serial.close())

        # Botón "Tren de Pulsos"
        self.BotonTipoSetpoint.clicked.connect(self.generar_trendepulsos)
        self.Periodo.hide()
        self.R_Periodo.hide()
        self.P_Periodo.hide()
        self.Y_Periodo.hide()

        self.serial.readyRead.connect(self.read_data)
        # self.Acelerador.valueChanged.connect(self.accelerator_pwm)
        self.Acelerador.sliderReleased.connect(self.accelerator_pwm)

        self.x1 = list(np.linspace(0, self.Val_Tiempo, self.Val_Tiempo))
        self.y1 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y2 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y3 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y4 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y5 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y6 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y7 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y8 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y9 = list(np.linspace(0, 0, self.Val_Tiempo))
        self.y10 = list(np.linspace(0, 0, self.Val_Tiempo))

        # Grafica
        self.setup_button(self.Inicio, "Iniciar1.png", "Iniciar2.png", "Iniciar3.png", 106, 24)
        # Botón "Reset"
        self.setup_button(self.Parar, "Parar1.png", "Parar2.png", "Parar3.png", 106, 24)


        self.Inicio.clicked.connect(self.iniciar_grafica)
        self.Parar.clicked.connect(self.parar_grafica)
        pg.setConfigOption('background', '#ffffff')
        pg.setConfigOption('foreground', '#2c2c2c')
        self.plt = pg.PlotWidget()
        self.graph_layout.addWidget(self.plt)

        # Checkboxes
        self.R_Check_Med.toggled.connect(self.check_toggle)
        self.P_Check_Med.toggled.connect(self.check_toggle)
        self.Y_Check_Med.toggled.connect(self.check_toggle)
        self.R_Check_Set.toggled.connect(self.check_toggle)
        self.P_Check_Set.toggled.connect(self.check_toggle)
        self.Y_Check_Set.toggled.connect(self.check_toggle)
        self.LC_Check_1.toggled.connect(self.check_toggle)
        self.LC_Check_2.toggled.connect(self.check_toggle)
        self.LC_Check_3.toggled.connect(self.check_toggle)
        self.LC_Check_4.toggled.connect(self.check_toggle)

        self.read_ports()

    def generar_trendepulsos(self):
        self.flag_TipoSetpoint = not self.flag_TipoSetpoint

        if self.flag_TipoSetpoint:
            self.Periodo.hide()
            self.R_Periodo.hide()
            self.P_Periodo.hide()
            self.Y_Periodo.hide()
            print("Pulsos mode")
            self.BotonTipoSetpoint.setText("Pulsos")
            self.Setpoint.setText("Setpoint")
        else:
            self.Periodo.show()
            self.R_Periodo.show()
            self.P_Periodo.show()
            self.Y_Periodo.show()
            print("Setpoints mode")
            self.BotonTipoSetpoint.setText("Setpoints")
            self.Setpoint.setText("Amplitud")

    def tiempo_valor(self):
        nuevo_val_tiempo = int(self.MaxTiempo.text())
        if nuevo_val_tiempo > 0:
            self.Val_Tiempo = nuevo_val_tiempo
            self.x1 = list(np.linspace(0, self.Val_Tiempo, self.Val_Tiempo))

            # Redimensionar y conservar el historial de las listas self.y1, self.y2, ..., self.y10
            for i in range(1, 11):
                y_list = deque(getattr(self, f"y{i}"), maxlen=self.Val_Tiempo)
                if len(y_list) < self.Val_Tiempo:
                    y_list.extend([0] * (self.Val_Tiempo - len(y_list)))
                else:
                    y_list = deque(list(y_list), maxlen=self.Val_Tiempo)
                setattr(self, f"y{i}", list(y_list))

            print(
                f"Se actualizó el eje x con {self.Val_Tiempo} muestras y se redimensionaron las listas self.y1, self.y2, ..., self.y10 conservando el historial.")
        else:
            print("El valor debe ser mayor que cero.")

    def setup_button(self, button, normal_image_path, hover_image_path, clicked_image_path, Size1, Size2):
        # Establecer la imagen normal
        normal_image = QPixmap(normal_image_path)
        normal_image = normal_image.scaled(QSize(Size1, Size2))
        button.setIcon(QIcon(normal_image))
        button.setIconSize(QSize(Size1, Size2))

        # Establecer la imagen cuando el cursor está sobre el botón
        hover_image = QPixmap(hover_image_path)
        hover_image = hover_image.scaled(QSize(Size1, Size2))

        # Establecer la imagen cuando está clickeado
        clicked_image = QPixmap(clicked_image_path)
        clicked_image = clicked_image.scaled(QSize(Size1, Size2))

        # Conectar señales a funciones
        button.enterEvent = lambda event, b=button, h=hover_image: self.enter_button(event, b, h)
        button.leaveEvent = lambda event, b=button, n=normal_image: self.leave_button(event, b, n)
        button.clicked.connect(lambda: self.clicked_button(button, clicked_image))

    def enter_button(self, event, button, hover_image):
        button.setIcon(QIcon(hover_image))

    def leave_button(self, event, button, normal_image):
        button.setIcon(QIcon(normal_image))

    def clicked_button(self, button, clicked_image):
        button.setIcon(QIcon(clicked_image))

    def guardar_valores(self):

        print("Valor de Kp Roll:", self.Val_R_Kp)
        print("Valor de Kp Pitch:", self.Val_P_Kp)
        print("Valor de Kp Yaw:", self.Val_Y_Kp)
        print("Valor de Ki Roll:", self.Val_R_Ki)
        print("Valor de Ki Pitch:", self.Val_P_Ki)
        print("Valor de Ki Yaw:", self.Val_Y_Ki)
        print("Valor de Kd Roll:", self.Val_R_Kd)
        print("Valor de Kd Pitch:", self.Val_P_Kd)
        print("Valor de Kd Yaw:", self.Val_Y_Kd)
        print("Valor de Setpoint Roll:", self.Val_R_Set)
        print("Valor de Setpoint Pitch:", self.Val_P_Set)
        print("Valor de Setpoint Yaw:", self.Val_Y_Set)
        print("Valor de Porcentaje:", self.Val_Porcentaje)

        self.flag_EnviarDatos = "2"
        self.send_data()


    def reset_valores(self):
        Val_R_Kp, Val_P_Kp, Val_Y_Kp = 0.0, 0.0, 0.0
        Val_R_Ki, Val_P_Ki, Val_Y_Ki = 0.0, 0.0, 0.0
        Val_R_Kd, Val_P_Kd, Val_Y_Kd = 0.0, 0.0, 0.0
        Val_R_Set, Val_P_Set, Val_Y_Set = 0.0, 0.0, 0.0
        Val_Porcentaje = 0.0

        self.R_Kp.setValue(Val_R_Kp)
        self.P_Kp.setValue(Val_P_Kp)
        self.Y_Kp.setValue(Val_Y_Kp)
        self.R_Ki.setValue(Val_R_Ki)
        self.P_Ki.setValue(Val_P_Ki)
        self.Y_Ki.setValue(Val_Y_Ki)
        self.R_Kd.setValue(Val_R_Kd)
        self.P_Kd.setValue(Val_P_Kd)
        self.Y_Kd.setValue(Val_Y_Kd)
        self.R_Set.setValue(Val_R_Set)
        self.P_Set.setValue(Val_P_Set)
        self.Y_Set.setValue(Val_Y_Set)
        self.Porcentaje.setValue(Val_Porcentaje)

    def valor_cambiado(self, value):
        if self.sender() == self.R_Kp:
            self.Val_R_Kp = value
        if self.sender() == self.P_Kp:
            self.Val_P_Kp = value
        if self.sender() == self.Y_Kp:
            self.Val_Y_Kp = value
        if self.sender() == self.R_Ki:
            self.Val_R_Ki = value
        if self.sender() == self.P_Ki:
            self.Val_P_Ki = value
        if self.sender() == self.Y_Ki:
            self.Val_Y_Ki = value
        if self.sender() == self.R_Kd:
            self.Val_R_Kd = value
        if self.sender() == self.P_Kd:
            self.Val_P_Kd = value
        if self.sender() == self.Y_Kd:
            self.Val_Y_Kd = value
        if self.sender() == self.R_Set:
            self.Val_R_Set = value
        if self.sender() == self.P_Set:
            self.Val_P_Set = value
        if self.sender() == self.Y_Set:
            self.Val_Y_Set = value
        elif self.sender() == self.Porcentaje:
            self.Val_Porcentaje = value
    def check_toggle(self):
        if self.R_Check_Med.isChecked():
            self.flag_check_R_Med = "1"
        else:
            self.flag_check_R_Med = "0"

        if self.P_Check_Med.isChecked():
            self.flag_check_P_Med = "1"
        else:
            self.flag_check_P_Med = "0"

        if self.Y_Check_Med.isChecked():
            self.flag_check_Y_Med = "1"
        else:
            self.flag_check_Y_Med = "0"

        if self.R_Check_Set.isChecked():
            self.flag_check_R_Set = "1"
        else:
            self.flag_check_R_Set = "0"

        if self.P_Check_Set.isChecked():
            self.flag_check_P_Set = "1"
        else:
            self.flag_check_P_Set = "0"

        if self.Y_Check_Set.isChecked():
            self.flag_check_Y_Set = "1"
        else:
            self.flag_check_Y_Set = "0"

        if self.LC_Check_1.isChecked():
            self.flag_check_LC1 = "1"
        else:
            self.flag_check_LC1 = "0"

        if self.LC_Check_2.isChecked():
            self.flag_check_LC2 = "1"
        else:
            self.flag_check_LC2 = "0"

        if self.LC_Check_3.isChecked():
            self.flag_check_LC3 = "1"
        else:
            self.flag_check_LC3 = "0"

        if self.LC_Check_4.isChecked():
            self.flag_check_LC4 = "1"
        else:
            self.flag_check_LC4 = "0"

    def read_ports(self):
        self.baudrates = ['1200', '2400', '4800', '9600',
                          '19200', '38400', '115200']

        portlist = []
        ports = QSerialPortInfo().availablePorts()
        for i in ports:
            portlist.append(i.portName())

        self.cb_list_ports.clear()
        self.cb_list_baudrates.clear()
        self.cb_list_ports.addItems(portlist)
        self.cb_list_baudrates.addItems(self.baudrates)
        self.cb_list_baudrates.setCurrentText('9600')

    def serial_connect(self):
        self.serial.waitForReadyRead(100)
        self.port = self.cb_list_ports.currentText()
        self.baud = self.cb_list_baudrates.currentText()
        self.serial.setBaudRate(int(self.baud))
        self.serial.setPortName(self.port)
        self.serial.open(QIODevice.ReadWrite)

    def read_data(self):
        if not self.serial.canReadLine():
            return

        rx = self.serial.readLine()
        rx = str(rx, 'utf-8').strip()

        # Dividir los datos en una lista
        data_list = rx.split(',')

        if len(data_list) != 7:
            return

        # Convertir los valores a números flotantes
        y1, y2, y3, y4, y5, y6, y7 = map(float, data_list)
        self.Roll_Med = y1
        self.Pitch_Med = y2
        self.Yaw_Med = y3
        self.M1_Med = y4
        self.M2_Med = y5
        self.M3_Med = y6
        self.M4_Med = y7

    def graficar(self):
        # Actualizar gráficas según las casillas de verificación
        plot_colors = ['#da0037', '#FF5733', '#BC970B', '#28A70E', '#109772', '#106697', '#4F1097', '#E590A5',
                       '#FFA28E', '#DACD9B']
        plots = []

        if (self.flag_graf == "1"):
            self.y1 = self.y1[1:]
            self.y1.append(self.Roll_Med)  # Graficar el 1 valor
            self.y2 = self.y2[1:]
            self.y2.append(self.Pitch_Med)  # Graficar el 2 valor
            self.y3 = self.y3[1:]
            self.y3.append(self.Yaw_Med)  # Graficar el 3 valor
            self.y4 = self.y4[1:]
            self.y4.append(self.M1_Med)  # Graficar el 4 valor
            self.y5 = self.y5[1:]
            self.y5.append(self.M2_Med)  # Graficar el 4 valor
            self.y6 = self.y6[1:]
            self.y6.append(self.M3_Med)  # Graficar el 4 valor
            self.y7 = self.y7[1:]
            self.y7.append(self.M4_Med)  # Graficar el 4 valor
            self.y8 = self.y8[1:]
            self.y8.append(self.Val_R_Set)  # Graficar el 4 valor
            self.y9 = self.y9[1:]
            self.y9.append(self.Val_P_Set)  # Graficar el 4 valor
            self.y10 = self.y10[1:]
            self.y10.append(self.Val_Y_Set)  # Graficar el 4 valor

            self.y_values = [self.y1, self.y2, self.y3, self.y4, self.y5, self.y6, self.y7, self.y8, self.y9, self.y10]
            checkbox_flags = [self.flag_check_R_Med, self.flag_check_P_Med, self.flag_check_Y_Med, self.flag_check_LC1,
                              self.flag_check_LC2, self.flag_check_LC3, self.flag_check_LC4, self.flag_check_R_Set,
                              self.flag_check_P_Set, self.flag_check_Y_Set]
            legend_names = ["Valor 1", "Valor 2", "Valor 3", "Valor 4", "Valor 5", "Valor 6", "Valor 7", "Valor 8",
                            "Valor 9", "Valor 10"]

            self.plt.clear()
            for i in range(len(legend_names)):
                if checkbox_flags[i] == "1":
                    self.plt.plot(self.x1, self.y_values[i], pen=pg.mkPen(plot_colors[i], width=2), name=legend_names[i])
                    plots.append(i)

            if not plots:
                self.plt.clear()

            self.guardar_csv()

        self.PWM_motores()
        self.actualizar_valores()

    def guardar_csv(self):
        if self.flag_graf == "1":
            # Verificar si es la primera vez que se guarda en el CSV
            if self.tiempo_inicial is None:
                self.tiempo_inicial = time.time()  # Guardar el tiempo inicial
                # Crear un nombre único para el archivo CSV con la fecha y hora de inicio
                self.nombre_archivo = "data_{}.csv".format(datetime.now().strftime('%Y%m%d_%H%M%S'))
                print(self.nombre_archivo)
                self.EstadoCSV.setText("Guardando datos en archivo: {}.".format(self.nombre_archivo))

            # Asegurarse de que self.nombre_archivo no sea None antes de abrir el archivo
            if self.nombre_archivo is not None:
                # Obtener el tiempo transcurrido desde el tiempo inicial con tres decimales
                tiempo_transcurrido = round(time.time() - self.tiempo_inicial, 3)

                # Convertir el tiempo transcurrido a minutos y segundos
                minutos, segundos = divmod(tiempo_transcurrido, 60)

                with open(self.nombre_archivo, 'a', newline='') as csvfile:
                    # Crear un objeto escritor CSV
                    writer = csv.writer(csvfile)

                    # Escribir los datos en el archivo CSV como una nueva fila
                    writer.writerow(["{:02}:{:06.3f}".format(int(minutos), segundos),
                                     self.Roll_Med, self.Pitch_Med, self.Yaw_Med,
                                     self.M1_Med, self.M2_Med, self.M3_Med, self.M4_Med,
                                     self.Val_R_Set, self.Val_P_Set, self.Val_Y_Set, self.Val_Y_Kp, self.Val_Y_Ki, self.Val_Y_Kd] )

    def iniciar_grafica(self):
        self.flag_graf = "1"
        # Si ya se ha iniciado, mantener el nombre del archivo existente

    def parar_grafica(self):
        self.flag_graf = "0"
        self.tiempo_inicial = None  # Reiniciar el tiempo al desactivar la flag
        self.nombre_archivo = None  # Reiniciar el nombre delu archivo
        self.EstadoCSV.setText("Se guardaron datos en archivo CSV.")

    def send_data(self):
        if (self.flag_EnviarDatos == "1"):
            # Para enviar solo acelerador
            data = str(self.flag_EnviarDatos) + "," + str(self.acc)

        if (self.flag_EnviarDatos == "2"):
            # Para enviar valores de ganancias
            data = (str(self.flag_EnviarDatos) + "," + str(self.Val_R_Kp) + "," + str(self.Val_R_Ki) + "," + str(
                self.Val_R_Kd) + "," + str(self.Val_P_Kp) + "," + str(self.Val_P_Ki) + "," + str(
                self.Val_P_Kd) + "," + str(self.Val_Y_Kp) + "," + str(self.Val_Y_Ki) + "," + str(
                self.Val_Y_Kd) + "," + str(self.Val_R_Set) + "," + str(self.Val_P_Set) + "," + str(self.Val_Y_Set) + "," + str(self.Val_Porcentaje))

        data = data + "\n"
        print(data)

        if self.serial.isOpen():
            self.serial.write(data.encode())

    def accelerator_pwm(self):
        self.acc = self.Acelerador.value()
        self.flag_EnviarDatos = "1"
        self.send_data()

        self.val_Acc.setAlignment(Qt.AlignCenter)
        self.val_Acc.setText(str(self.acc))

    def PWM_motores(self):
        Val_PWM_M1 = self.M1_Med
        Val_PWM_M2 = self.M2_Med
        Val_PWM_M3 = self.M3_Med
        Val_PWM_M4 = self.M4_Med
        self.M1.setValue(int(Val_PWM_M1))
        self.M2.setValue(int(Val_PWM_M2))
        self.M3.setValue(int(Val_PWM_M3))
        self.M4.setValue(int(Val_PWM_M4))

    def actualizar_valores(self):
        Roll_Medido = str(self.Roll_Med)
        Pitch_Medido = str(self.Pitch_Med)
        Yaw_Medido = str(self.Yaw_Med)

        self.Med_Roll.setText(Roll_Medido)
        self.Med_Pitch.setText(Pitch_Medido)
        self.Med_Yaw.setText(Yaw_Medido)

    def CambioControl(self):
        seleccion = self.Control_Menu.itemText(self.Control_Menu.currentIndex())

        if seleccion == "Control Proporcional, Integral & Derivativo (PID)":
            self.Kd.show()
            self.R_Kd.show()
            self.Y_Kd.show()
            self.P_Kd.show()
            self.R_Kp.setDisabled(False)
            self.R_Ki.setDisabled(False)
            self.Y_Kp.setDisabled(False)
            self.Y_Ki.setDisabled(False)

            self.Kp.setText("Kp")
            self.Ki.setText("Ki")
            self.Kd.setText("Kd")
            self.flag_ControlMenu = "1"

            Val_R_Kp, Val_P_Kp, Val_Y_Kp = 2.5, 2.5, 4
            Val_R_Ki, Val_P_Ki, Val_Y_Ki = 0.07, 0.07, 0.01
            Val_R_Kd, Val_P_Kd, Val_Y_Kd = 30.0, 30.0, 0.0
            Val_Porcentaje = 0.6

            self.R_Kp.setValue(Val_R_Kp)
            self.P_Kp.setValue(Val_P_Kp)
            self.Y_Kp.setValue(Val_Y_Kp)
            self.R_Ki.setValue(Val_R_Ki)
            self.P_Ki.setValue(Val_P_Ki)
            self.Y_Ki.setValue(Val_Y_Ki)
            self.R_Kd.setValue(Val_R_Kd)
            self.P_Kd.setValue(Val_P_Kd)
            self.Y_Kd.setValue(Val_Y_Kd)
            self.Porcentaje.setValue(Val_Porcentaje)

        if seleccion == "Control Regulador Lineal Cuadrático (LQR)":
            self.Kd.show()
            self.R_Kd.show()
            self.Y_Kd.show()
            self.P_Kd.show()
            self.R_Kp.setDisabled(False)
            self.R_Ki.setDisabled(False)
            self.Y_Kp.setDisabled(False)
            self.Y_Ki.setDisabled(False)

            self.Kp.setText("Ki")
            self.Ki.setText("K1")
            self.Kd.setText("K2")
            self.flag_ControlMenu = "2"

            Val_R_Kp, Val_P_Kp, Val_Y_Kp = 0.005, 0.005, 0.01
            Val_R_Ki, Val_P_Ki, Val_Y_Ki = 2.0, 2.0, 0.0001
            Val_R_Kd, Val_P_Kd, Val_Y_Kd = 0.5, 0.5, 0.001
            Val_Porcentaje = 0.3

            self.R_Kp.setValue(Val_R_Kp)
            self.P_Kp.setValue(Val_P_Kp)
            self.Y_Kp.setValue(Val_Y_Kp)
            self.R_Ki.setValue(Val_R_Ki)
            self.P_Ki.setValue(Val_P_Ki)
            self.Y_Ki.setValue(Val_Y_Ki)
            self.R_Kd.setValue(Val_R_Kd)
            self.P_Kd.setValue(Val_P_Kd)
            self.Y_Kd.setValue(Val_Y_Kd)
            self.Porcentaje.setValue(Val_Porcentaje)


        if seleccion == "Control de Superficie Deslizante (SMC)":
            self.Kp.setText("Lambda")
            self.Ki.setText("K")
            self.Kd.hide()
            self.R_Kd.hide()
            self.Y_Kd.hide()
            self.P_Kd.hide()
            self.R_Kp.setValue(0)
            self.R_Ki.setValue(0)
            self.Y_Kp.setValue(0)
            self.Y_Ki.setValue(0)
            self.R_Kp.setDisabled(True)
            self.R_Ki.setDisabled(True)
            self.Y_Kp.setDisabled(True)
            self.Y_Ki.setDisabled(True)

    def show_popup(self):
        popup = PopupWindow(self)
        popup.exec_()


class PopupWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle('Ventana Emergente')
        self.setGeometry(200, 200, 300, 200)

        layout = QVBoxLayout()



        close_button = QPushButton('Cerrar', self)
        close_button.clicked.connect(self.accept)

        layout.addWidget(close_button)
        self.setLayout(layout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    my_app = MyApp()
    my_app.show()
    sys.exit(app.exec_())

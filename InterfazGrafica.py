# Para inicializar una ventana en python de QT5 comandos basicos
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.uic import loadUi
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice, Qt, QSize
from PyQt5.QtGui import QPixmap, QIcon

from datetime import datetime

import pyqtgraph as pg
import numpy as np
import sys
import csv
import time


class MyApp(QMainWindow):

    def __init__(self):
        super(MyApp, self).__init__()
        loadUi('InterfazV2.ui', self)

        # Define la imagen de fondo
        self.Imgfondo = QPixmap('FondoInterfazV5.png')
        self.Imgfondo = self.Imgfondo.scaled(self.Fondo.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Fondo.setPixmap(self.Imgfondo)
        self.tiempo_inicial = time.time()

        # Inserción de Imagenes
        self.Image1 = QPixmap('1.png')
        self.Image2 = QPixmap('2.png')
        self.Image3 = QPixmap('3.png')
        self.Image4 = QPixmap('4.png')
        self.Image5 = QPixmap('5.png')
        self.Image6 = QPixmap('6.png')
        self.Image7 = QPixmap('7.png')

        self.Image1 = self.Image1.scaled(self.Punto1.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Image2 = self.Image2.scaled(self.Punto2.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Image3 = self.Image3.scaled(self.Punto3.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Image4 = self.Image4.scaled(self.Punto4.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Image5 = self.Image5.scaled(self.Punto5.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Image6 = self.Image6.scaled(self.Punto6.size(), aspectRatioMode=Qt.KeepAspectRatio)
        self.Image7 = self.Image7.scaled(self.Punto7.size(), aspectRatioMode=Qt.KeepAspectRatio)

        self.Punto1.setPixmap(self.Image1)
        self.Punto2.setPixmap(self.Image2)
        self.Punto3.setPixmap(self.Image3)
        self.Punto4.setPixmap(self.Image4)
        self.Punto5.setPixmap(self.Image5)
        self.Punto6.setPixmap(self.Image6)
        self.Punto7.setPixmap(self.Image7)

        # Seleccion de Control
        self.Control_Menu.currentIndexChanged.connect(self.CambioControl)
        self.Control_Menu.addItem("Control Proporcional, Integral & Derivativo (PID)")
        self.Control_Menu.addItem("Control Regulador Lineal Cuadrático (LQR)")

        self.flag_graf = "0"
        self.flag_csv = "0"
        self.flag_check_R = "0"
        self.flag_check_P = "0"
        self.flag_check_Y = "0"
        self.flag_check_LC1 = "0"
        self.flag_check_LC2 = "0"
        self.flag_check_LC3 = "0"
        self.flag_check_LC4 = "0"
        self.flag_ControlMenu = "0"
        self.flag_EnviarDatos = "0"

        # Variables Medidas
        self.Roll_Med = 0
        self.Pitch_Med = 0
        self.Yaw_Med = 0
        self.M1_Med = 0
        self.M2_Med = 0
        self.M3_Med = 0
        self.M4_Med = 0
        self.acc = 0

        # Ganancias PID
        self.Val_R_Kp = 0.0
        self.Val_P_Kp = 0.0
        self.Val_Y_Kp = 0.0
        self.Val_R_Ki = 0.0
        self.Val_P_Ki = 0.0
        self.Val_Y_Ki = 0.0
        self.Val_R_Kd = 0.0
        self.Val_P_Kd = 0.0
        self.Val_Y_Kd = 0.0
        self.Val_R_Set = 0.0
        self.Val_P_Set = 0.0
        self.Val_Y_Set = 0.0
        self.Val_Porcentaje = 0.0

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
        # self.Calibrar.clicked.connect(self.calibrar_sensor)

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

        self.serial.readyRead.connect(self.read_data)
        # self.Acelerador.valueChanged.connect(self.accelerator_pwm)
        self.Acelerador.sliderReleased.connect(self.accelerator_pwm)

        self.x1 = list(np.linspace(0, 100, 100))
        self.x2 = list(np.linspace(0, 100, 100))
        self.x3 = list(np.linspace(0, 100, 100))
        self.x4 = list(np.linspace(0, 100, 100))
        self.x5 = list(np.linspace(0, 100, 100))
        self.x6 = list(np.linspace(0, 100, 100))
        self.x7 = list(np.linspace(0, 100, 100))
        self.y1 = list(np.linspace(0, 0, 100))
        self.y2 = list(np.linspace(0, 0, 100))
        self.y3 = list(np.linspace(0, 0, 100))
        self.y4 = list(np.linspace(0, 0, 100))
        self.y5 = list(np.linspace(0, 0, 100))
        self.y6 = list(np.linspace(0, 0, 100))
        self.y7 = list(np.linspace(0, 0, 100))

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
        self.R_Check.toggled.connect(self.check_toggle)
        self.P_Check.toggled.connect(self.check_toggle)
        self.Y_Check.toggled.connect(self.check_toggle)
        self.LC_Check_1.toggled.connect(self.check_toggle)
        self.LC_Check_2.toggled.connect(self.check_toggle)
        self.LC_Check_3.toggled.connect(self.check_toggle)
        self.LC_Check_4.toggled.connect(self.check_toggle)

        self.read_ports()

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
        if self.R_Check.isChecked():
            self.flag_check_R = "1"
        else:
            self.flag_check_R = "0"

        if self.P_Check.isChecked():
            self.flag_check_P = "1"
        else:
            self.flag_check_P = "0"

        if self.Y_Check.isChecked():
            self.flag_check_Y = "1"
        else:
            self.flag_check_Y = "0"

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

        # print(f"Valor 1: {self.Roll_Med}, Valor 2: {self.Pitch_Med}, Valor 3: {self.Yaw_Med}, Valor 4: {self.M1_Med}, Valor 5: {self.M2_Med}, Valor 6: {self.M3_Med}, Valor 7: {self.M4_Med}")

        # Actualizar gráficas según las casillas de verificación
        plot_colors = ['#da0037', '#FF5733', '#BC970B', '#28A70E', '#109772', '#106697', '#4F1097']
        plots = []

        if (self.flag_graf == "1"):
            self.y1 = self.y1[1:]
            self.y1.append(y1)  # Graficar el 1 valor
            self.y2 = self.y2[1:]
            self.y2.append(y2)  # Graficar el 2 valor
            self.y3 = self.y3[1:]
            self.y3.append(y3)  # Graficar el 3 valor
            self.y4 = self.y4[1:]
            self.y4.append(y4)  # Graficar el 4 valor
            self.y5 = self.y5[1:]
            self.y5.append(y5)  # Graficar el 4 valor
            self.y6 = self.y6[1:]
            self.y6.append(y6)  # Graficar el 4 valor
            self.y7 = self.y7[1:]
            self.y7.append(y7)  # Graficar el 4 valor

            y_values = [self.y1, self.y2, self.y3, self.y4, self.y5, self.y6, self.y7]
            checkbox_flags = [self.flag_check_R, self.flag_check_P, self.flag_check_Y, self.flag_check_LC1,
                              self.flag_check_LC2, self.flag_check_LC3, self.flag_check_LC4]
            legend_names = ["Valor 1", "Valor 2", "Valor 3", "Valor 4", "Valor 5", "Valor 6", "Valor 7"]

            self.plt.clear()
            for i in range(7):
                if checkbox_flags[i] == "1":
                    self.plt.plot(self.x1, y_values[i], pen=pg.mkPen(plot_colors[i], width=2), name=legend_names[i])
                    plots.append(i)

            if not plots:
                self.plt.clear()

            self.guardar_csv()

        self.PWM_motores()
        self.actualizar_valores()

    def guardar_csv(self):
        if (self.flag_graf == "1"):
            with open('data.csv', 'a', newline='') as csvfile:
                # Create a CSV writer object
                writer = csv.writer(csvfile)

                # timestamp = datetime.now().strftime('%M:%S.%f')
                # timestamp = round(time.time(), 3)
                # Obtener el tiempo transcurrido desde el tiempo inicial con tres decimales
                tiempo_transcurrido = round(time.time() - self.tiempo_inicial, 3)

                # Convertir el tiempo transcurrido a minutos y segundos
                minutos, segundos = divmod(tiempo_transcurrido, 60)

                # Escribir los datos en el archivo CSV como una nueva fila
                writer.writerow(
                    ["{:02}:{:06.3f}".format(int(minutos), segundos), self.Roll_Med, self.Pitch_Med, self.Yaw_Med,
                     self.M1_Med, self.M2_Med, self.M3_Med, self.M4_Med])

    def iniciar_grafica(self):
        self.flag_graf = "1"

    def parar_grafica(self):
        self.flag_graf = "0"

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

    ''' def accelerator_pwm(self, event):
        if self.serial.isOpen():
            self.Acelerador.setValue(event)
            self.val_Acc.setText(str(event))
            # acc = '1,' + str(event)
            self.acc = str(event)
            #self.send_data(acc)
            self.flag_EnviarDatos = "1"
            self.send_data()

            self.val_Acc.setAlignment(Qt.AlignCenter)
            self.val_Acc.setText(str(event))
        '''

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
            self.Kp.setText("Kp")
            self.Ki.setText("Ki")
            self.Kd.setText("Kd")
            self.flag_ControlMenu = "1"

        if seleccion == "Control Regulador Lineal Cuadrático (LQR)":
            self.Kp.setText("Ki")
            self.Ki.setText("K1")
            self.Kd.setText("K2")
            self.flag_ControlMenu = "2"


if __name__ == '__main__':
    app = QApplication(sys.argv)
    my_app = MyApp()
    my_app.show()
    sys.exit(app.exec_())

# Para inicializar una ventana en python de QT5 comandos basicos
import time

from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.uic import loadUi
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice

import pyqtgraph as pg
import numpy as np
import sys


class MyApp(QMainWindow):

    def __init__(self):
        super(MyApp, self).__init__()
        loadUi('InterfazV2.ui', self)

        self.flag_graf = "0"
        self.flag_check_R = "0"
        self.flag_check_P = "0"
        self.flag_check_Y = "0"
        self.flag_check_LC1 = "0"
        self.flag_check_LC2 = "0"
        self.flag_check_LC3 = "0"
        self.flag_check_LC4 = "0"

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

        # Botones Generales
        self.Guardar.clicked.connect(self.guardar_valores)
        self.Reset.clicked.connect(self.reset_valores)
        #self.Calibrar.clicked.connect(self.calibrar_sensor)

        # Control Connect
        self.serial = QSerialPort()
        self.Actualizar.clicked.connect(self.read_ports)
        self.Conectar.clicked.connect(self.serial_connect)
        self.Desconectar.clicked.connect(lambda: self.serial.close())

        self.serial.readyRead.connect(self.read_data)
        self.Acelerador.valueChanged.connect(self.accelerator_pwm)

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

    def reset_valores(self):
        Val_R_Kp, Val_P_Kp, Val_Y_Kp = 0.0, 0.0, 0.0
        Val_R_Ki, Val_P_Ki, Val_Y_Ki = 0.0, 0.0, 0.0
        Val_R_Kd, Val_P_Kd, Val_Y_Kd = 0.0, 0.0, 0.0
        Val_R_Set, Val_P_Set, Val_Y_Set = 0.0, 0.0, 0.0

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

    def valor_cambiado(self,value):
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
        elif self.sender() == self.Y_Set:
            self.Val_Y_Set = value

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
        print(rx)

        # Dividir los datos en una lista
        data_list = rx.split(',')

        if len(data_list) != 7:
            # Asegúrate de que haya exactamente 3 valores
            return

        # Convertir los valores a números flotantes
        x1, x2, x3, x4, x5, x6, x7 = map(float, data_list)

        #print(f"Valor 1: {x1}, Valor 2: {x2}, Valor 3: {x3}, Valor 4: {x4}, Valor 5: {x5}, Valor 6: {x6}, Valor 7: {x7}")

        if self.flag_graf == "1":
            self.y1 = self.y1[1:]
            self.y1.append(x1)  # Graficar el 1 valor
            self.y2 = self.y2[1:]
            self.y2.append(x2)  # Graficar el 2 valor
            self.y3 = self.y3[1:]
            self.y3.append(x3)  # Graficar el 3 valor
            self.y4 = self.y4[1:]
            self.y4.append(x4)  # Graficar el 4 valor
            self.y5 = self.y5[1:]
            self.y5.append(x5)  # Graficar el 4 valor
            self.y6 = self.y6[1:]
            self.y6.append(x6)  # Graficar el 4 valor
            self.y7 = self.y7[1:]
            self.y7.append(x7)  # Graficar el 4 valor

            if self.flag_check_R == "1" and self.flag_check_P == "0" and self.flag_check_Y == "0" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")

            if self.flag_check_R == "0" and self.flag_check_P == "1" and self.flag_check_Y == "0" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")
                # self.plt.addLegend()

            if self.flag_check_R == "0" and self.flag_check_P == "0" and self.flag_check_Y == "1" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")
                # self.plt.addLegend()

            if self.flag_check_R == "0" and self.flag_check_P == "0" and self.flag_check_Y == "0" and self.flag_check_LC1 == "1" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x4, self.y4, pen=pg.mkPen('#28A70E', width=2), name="Valor 4")
                # self.plt.addLegend()

            if self.flag_check_R == "0" and self.flag_check_P == "0" and self.flag_check_Y == "0" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "1" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x5, self.y5, pen=pg.mkPen('#109772', width=2), name="Valor 5")
                # self.plt.addLegend()

            if self.flag_check_R == "0" and self.flag_check_P == "0" and self.flag_check_Y == "0" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "1" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x6, self.y6, pen=pg.mkPen('#106697', width=2), name="Valor 6")
                # self.plt.addLegend()

            if self.flag_check_R == "0" and self.flag_check_P == "0" and self.flag_check_Y == "0" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "1":
                self.plt.clear()
                self.plt.plot(self.x7, self.y7, pen=pg.mkPen('#106697', width=2), name="Valor 7")
                # self.plt.addLegend()

            if self.flag_check_R == "1" and self.flag_check_P == "1" and self.flag_check_Y == "0" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")

            if self.flag_check_R == "1" and self.flag_check_P == "0" and self.flag_check_Y == "1" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")

            if self.flag_check_R == "1" and self.flag_check_P == "1" and self.flag_check_Y == "1" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")

            if self.flag_check_R == "0" and self.flag_check_P == "1" and self.flag_check_Y == "1" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")

            if self.flag_check_R == "1" and self.flag_check_P == "1" and self.flag_check_Y == "1" and self.flag_check_LC1 == "1" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")
                self.plt.plot(self.x4, self.y4, pen=pg.mkPen('#28A70E', width=2), name="Valor 4")

            if self.flag_check_R == "1" and self.flag_check_P == "1" and self.flag_check_Y == "1" and self.flag_check_LC1 == "1" and self.flag_check_LC2 == "1" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")
                self.plt.plot(self.x4, self.y4, pen=pg.mkPen('#28A70E', width=2), name="Valor 4")
                self.plt.plot(self.x5, self.y5, pen=pg.mkPen('#109772', width=2), name="Valor 5")

            if self.flag_check_R == "1" and self.flag_check_P == "1" and self.flag_check_Y == "1" and self.flag_check_LC1 == "1" and self.flag_check_LC2 == "1" and self.flag_check_LC3 == "1" and self.flag_check_LC4 == "0":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")
                self.plt.plot(self.x4, self.y4, pen=pg.mkPen('#28A70E', width=2), name="Valor 4")
                self.plt.plot(self.x5, self.y5, pen=pg.mkPen('#109772', width=2), name="Valor 5")
                self.plt.plot(self.x6, self.y6, pen=pg.mkPen('#106697', width=2), name="Valor 6")

            if self.flag_check_R == "1" and self.flag_check_P == "1" and self.flag_check_Y == "1" and self.flag_check_LC1 == "1" and self.flag_check_LC2 == "1" and self.flag_check_LC3 == "1" and self.flag_check_LC4 == "1":
                self.plt.clear()
                self.plt.plot(self.x1, self.y1, pen=pg.mkPen('#da0037', width=2), name="Valor 1")
                self.plt.plot(self.x2, self.y2, pen=pg.mkPen('#FF5733', width=2), name="Valor 2")
                self.plt.plot(self.x3, self.y3, pen=pg.mkPen('#BC970B', width=2), name="Valor 3")
                self.plt.plot(self.x4, self.y4, pen=pg.mkPen('#28A70E', width=2), name="Valor 4")
                self.plt.plot(self.x5, self.y5, pen=pg.mkPen('#109772', width=2), name="Valor 5")
                self.plt.plot(self.x6, self.y6, pen=pg.mkPen('#106697', width=2), name="Valor 6")
                self.plt.plot(self.x7, self.y7, pen=pg.mkPen('#106697', width=2), name="Valor 7")

            if self.flag_check_R == "0" and self.flag_check_P == "0" and self.flag_check_Y == "0" and self.flag_check_LC1 == "0" and self.flag_check_LC2 == "0" and self.flag_check_LC3 == "0" and self.flag_check_LC4 == "0":
                self.plt.clear()


        self.PWM_motores()
        self.actualizar_valores()

    def iniciar_grafica(self):
        self.flag_graf = "1"

    def parar_grafica(self):
        self.flag_graf = "0"

    def send_data(self, data):
        data = data + "\n"
        print(data)
        if self.serial.isOpen():
            self.serial.write(data.encode())

    def accelerator_pwm(self, event):
        self.Acelerador.setValue(event)
        self.val_Acc.setText(str(event))
        acc = '1,' + str(event)
        self.send_data(acc)

    def PWM_motores(self):
        Val_PWM_M1 = 1100
        Val_PWM_M2 = 1200
        Val_PWM_M3 = 1400
        Val_PWM_M4 = 1500
        self.M1.setValue(Val_PWM_M1)
        self.M2.setValue(Val_PWM_M2)
        self.M3.setValue(Val_PWM_M3)
        self.M4.setValue(Val_PWM_M4)

    def actualizar_valores(self):
        self.Roll_Medido = 0.3
        self.Pitch_Medido = 0.5
        self.Yaw_Medido = 5

        Roll_Medido = str(self.Roll_Medido)
        Pitch_Medido = str(self.Pitch_Medido)
        Yaw_Medido = str(self.Yaw_Medido)

        self.Med_Roll.setText(Roll_Medido)
        self.Med_Pitch.setText(Pitch_Medido)
        self.Med_Yaw.setText(Yaw_Medido)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    my_app = MyApp()
    my_app.show()
    sys.exit(app.exec_())

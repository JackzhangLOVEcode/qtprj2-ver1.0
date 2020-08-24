import sys
from mainWinUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QIcon
from socket import socket, AF_INET, SOCK_DGRAM

TXConfig = []
RXConfig = []
class configPage(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(configPage, self).__init__()
        self.setupUi(self)
        self.setWindowTitle('FMC205基带射频配置')
        self.setWindowIcon(QIcon('rss1.ico'))
        self.connectTXConfigData()
        self.connectRXConfigData()
        self.sendConfig.clicked.connect(self.sendConfigtoBaseBand)
        self.setDefault.clicked.connect(self.setDefaultConfig)

    def connectTXConfigData(self):
        flag = 1
        TXConfig.append((flag).to_bytes(2,byteorder='little'))
        TXConfig.append((self.papr_en_obj.checkState()).to_bytes(2, byteorder='little'))

    def connectRXConfigData(self):
        flag = 0
        RXConfig.append((flag).to_bytes(2,byteorder='little'))

    def sendConfigtoBaseBand(self):
        ip = ''
        port = 8000
        addr = (ip, port)
        configSocket = socket(AF_INET, SOCK_DGRAM)
        configSocket.sendto(TXConfig, addr)
        configSocket.sendto(RXConfig, addr)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())
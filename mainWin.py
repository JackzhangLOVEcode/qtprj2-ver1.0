import sys
from mainWinUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QIcon
from socket import socket, AF_INET, SOCK_DGRAM

class configPage(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(configPage, self).__init__()
        self.setupUi(self)
        self.setWindowTitle('FMC205基带射频配置')
        self.setWindowIcon(QIcon('rss1.ico'))
        self.TXConfig = []
        self.RXConfig = []
        self.TXConfigBytes = bytes()
        self.RXConfigBytes = bytes()
        self.connectTXConfigData()
        self.connectRXConfigData()
        self.sendConfig.clicked.connect(self.sendConfigtoBaseBand)
        self.setDefault.clicked.connect(self.setDefaultConfig)

    def calculateOFDMSymbleNum(self):
        MTypeToBit = [2, 4, 6, 8]
        car = 0
        for i in range((self.car_num_tx_obj.value()).bit_length()):
            if(self.car_num_tx_obj.value() & (1 << i)):
                car += 1
        m_len = car * 64 * MTypeToBit[self.MType_obj.currentIndex()] / 8 - 4
        return int(m_len)

    def connectTXConfigData(self):
        self.TXConfig.clear()
        flag = 1
        self.TXConfig.extend((flag).to_bytes(1,byteorder='big'))
        self.TXConfig.extend((self.papr_en_obj.checkState()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.depapr_thr_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.depapr_gain_obj.value()).to_bytes(2, byteorder='big'))
        self.TXConfig.extend((self.MType_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.car_num_tx_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend(int((self.Alpha_tx_obj.value() * 32768)).to_bytes(3, byteorder='big'))
        self.TXConfig.extend(int(self.pilot_factor_obj.value()).to_bytes(2,byteorder='big'))
        self.TXConfig.extend(int(self.pss_factor_obj.value()).to_bytes(2, byteorder='big'))
        self.TXConfig.extend((self.ModeBSorMS_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.Sys_reset_obj.checkState()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.Sys_enble_obj.checkState()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.ModeUDPorPN_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.Base_loop_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.Rx_enb_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.Rx_channel_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.Rx_delay_obj.value()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.Time_trig2tx_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.Time_tx_hold_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.ms_T_sync2trig_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.bs_tdd_time_gap_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.bs_tx_time_gap_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.Trig_gap_cnt_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.alway_tx_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.calculateOFDMSymbleNum()).to_bytes(2, byteorder='big'))
        self.TXConfig.extend((self.calculateOFDMSymbleNum() * 12 - 12).to_bytes(2, byteorder='big'))
        self.TXConfig.extend((self.tx1_en_obj.currentIndex().to_bytes(1, byteorder='big')))
        self.TXConfig.extend((self.tx2_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.rx1_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.rx2_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.TDD_EN_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.UDP_loop_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfigBytes = bytes(self.TXConfig)

    def connectRXConfigData(self):
        self.RXConfig.clear()
        flag = 0
        self.RXConfig.extend((flag).to_bytes(1, byteorder='big'))

    def sendConfigtoBaseBand(self):
        ip = ''
        port = 8000
        addr = (ip, port)
        configSocket = socket(AF_INET, SOCK_DGRAM)
        configSocket.sendto(self.TXConfigBytes, addr)
        configSocket.sendto(self.RXConfigBytes, addr)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())
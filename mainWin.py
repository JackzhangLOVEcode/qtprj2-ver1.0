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
        self.RFConfig = []
        self.TXConfigBytes = bytes()
        self.RXConfigBytes = bytes()
        self.ipBB = '127.0.0.1'
        self.portBB = 8000
        self.ipRF = '127.0.0.1'
        self.portRF = 6005
        self.sendConfig.clicked.connect(self.sendConfigtoBaseBand)
        self.setDefault.clicked.connect(self.setDefaultConfig)

    def connectRFConfigData(self):
        self.RFConfig.clear()
        config = [0x81, 0x02, 0x10, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]
        if (self.Amplifier_obj.currentIndex() == 1):
            self.RFConfig.append(bytes(config))
        else:
            config[19] = 0x00
            self.RFConfig.append(bytes(config))
        if(self.filter3_3p5G_obj.currentIndex() == 1):
            config = [0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01]
            self.RFConfig.append(bytes(config))
        if(self.filter3p5_4G_obj.currentIndex() == 1):
            config = [0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            self.RFConfig.append(bytes(config))
        if(self.filter4p5_5G_obj.currentIndex() == 1):
            config = [0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00]
            self.RFConfig.append(bytes(config))
        if(self.filter5_5p5G_obj.currentIndex() == 1):
            config = [0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]
            self.RFConfig.append(bytes(config))

    def calculateValidBitNum(self, car_num):
        car = 0
        for i in range((car_num).bit_length()):
            if (car_num & (1 << i)):
                car += 1
        return int(car)

    def calculateOFDMSymbleNum(self, car_num, mtype):
        MTypeToBit = [2, 4, 6, 8]
        car = self.calculateValidBitNum(car_num)
        m_len = car * 64 * MTypeToBit[mtype] / 8 - 4
        return int(m_len)

    def connectTXConfigData(self):
        self.TXConfig.clear()
        flag = 1
        self.TXConfig.extend((flag).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.papr_en_obj.checkState()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.depapr_thr_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.depapr_gain_obj.value()).to_bytes(2, byteorder='big'))
        self.TXConfig.extend((self.MType_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.car_num_tx_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend(int((self.Alpha_tx_obj.value() * 32768)).to_bytes(3, byteorder='big'))
        self.TXConfig.extend(int(self.pilot_factor_obj.value()).to_bytes(2, byteorder='big'))
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
        mLen = self.calculateOFDMSymbleNum(self.car_num_tx_obj.value(), self.MType_obj.currentIndex())
        self.TXConfig.extend((mLen).to_bytes(2, byteorder='big'))
        maxLen = self.calculateOFDMSymbleNum(self.car_num_tx_obj.value(), self.MType_obj.currentIndex()) * 12 - 12
        self.TXConfig.extend((maxLen).to_bytes(2, byteorder='big'))
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
        self.RXConfig.extend((self.MType_rx_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.agc_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.frft_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.car_num_rx_obj.value()).to_bytes(3, byteorder='big'))
        car_N = self.calculateValidBitNum(self.car_num_rx_obj.value()) * 64 - 1
        self.RXConfig.extend((car_N).to_bytes(2, byteorder='big'))
        data_len = self.calculateOFDMSymbleNum(self.car_num_rx_obj.value(), self.MType_rx_obj.currentIndex())
        self.RXConfig.extend((data_len).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.i_freq_est_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.sync_factor_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.equa_factor_obj.value()).to_bytes(2, byteorder='big'))
        Alpha_rx = int(self.Alpha_rx_obj.value()*32768)
        self.RXConfig.extend((Alpha_rx).to_bytes(3, byteorder='big'))
        self.RXConfig.extend((self.equa_amp_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.equa_amp_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.car_thr1_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfig.extend((self.car_thr2_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfig.extend((self.car_thr3_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfig.extend((self.mmse_thr_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfigBytes = bytes(self.RXConfig)


    def sendConfigtoBaseBand(self):
        addrBB = (self.ipBB, self.portBB)
        addRF = (self.ipRF, self.portRF)
        configSocket = socket(AF_INET, SOCK_DGRAM)
        self.connectRFConfigData()
        print(self.RFConfig)
        for configRF in self.RFConfig:
            configSocket.sendto(configRF, addRF)
            print(configRF)
        self.connectTXConfigData()
        print(self.TXConfigBytes)
        configSocket.sendto((self.TXConfigBytes), addrBB)
        self.connectRXConfigData()
        print(self.RXConfig)
        configSocket.sendto(self.RXConfigBytes, addrBB)
        configSocket.close()

    def setDefaultConfig(self):
        addr = (self.ipBB, self.ipBB)
        defaultConfigSocket = socket(AF_INET, SOCK_DGRAM)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())
# -*- coding:utf-8 -*-
import sys, math, socket, time, queue, struct, encodings.idna
from mainWinUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtCore import QThread, QTimer, QPoint, Qt
from PyQt5.QtGui import QIcon, QPainter, QPixmap, QPen, QColor

def bytesToFloat(h1, h2, h3, h4):
    ba = bytearray()
    ba.append(h1)
    ba.append(h2)
    ba.append(h3)
    ba.append(h4)
    return struct.unpack("!f", ba)[0]

class WorkThread(QThread):
    def __int__(self):
        super(WorkThread, self).__init__()

    def run(self):
        global dataStatistical
        dataStatistical = bytes([0])
        global portStatistical
        portStatistical = 7000
        while True:
            try:
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                hostname = socket.gethostname()
                addr = (socket.gethostbyname(hostname), portStatistical)
                buffsize = 33000
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                # print('receive data ......')
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                # print(data)
                dataStatistical = data
                # print(dataStatistical)
            except socket.timeout:
                statisticalSocket.close()

class DataThread(QThread):
    def __int__(self):
        super(DataThread, self).__init__()

    def run(self):
        global startFlag
        startFlag = True
        global dataQueue
        dataQueue = queue.Queue(0)
        hostName = socket.gethostname()
        ipLocal = socket.gethostbyname(hostName)
        ipData = 63000
        buffSize = 1500
        addr = (ipLocal, ipData)
        socketSource = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socketSource.bind(addr)
        socketSource.settimeout(1)
        while True:
            if startFlag:
                try:
                    data, ipSource = socketSource.recvfrom(buffSize)
                    dataQueue.put(data)
                    print('Receive source data')
                except socket.timeout:
                    print('dataSocket timeout')
                    # pass
        socketSource.close()

class showConstellation():
    def __init__(self, length, height, label):
        self.draw = QPainter()
        self.picture = QPixmap(length, height)
        self.y_height = height
        self.label = label

    def count_dot(self, data, multiplier=1, addend=0):

        self.data = data
        self.picture.fill(Qt.white)
        for i in range(int(len(self.data)/8)):
            imData = bytesToFloat(self.data[0], self.data[1], self.data[2], self.data[3])
            reData = bytesToFloat(self.data[4], self.data[5], self.data[6], self.data[7])
            self.data = self.data[8:]
            x = imData * multiplier + addend
            y = reData * multiplier + addend
            self.end_x = x
            self.end_y = self.y_height - y
            self.uptate_show()
        self.label.setPixmap(self.picture)

    def uptate_show(self):
        self.draw.begin(self.picture)
        self.draw.setPen(QPen(QColor("red"), 2))
        self.draw.drawPoint(QPoint(self.end_x, self.end_y))
        self.draw.end()

class configPage(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(configPage, self).__init__()
        self.setupUi(self)
        self.setWindowTitle('FMC205基带射频配置')
        self.setWindowIcon(QIcon('rss1.ico'))
        self.init()
        self.timer = QTimer()
        self.dataTimer = QTimer()
        self.bindSingalandSlot()

    def init(self):
        self.TXConfig = []
        self.RXConfig = []
        self.RFConfig = []
        self.TXConfigBytes = bytes()
        self.RXConfigBytes = bytes()
        self.ipBB = '192.168.1.84'
        self.portBB = 8000
        self.ipRF = '192.168.1.38'
        self.portRF = 6005
        self.ipData = '192.168.71.174'
        self.portData = 8002
        self.draw = QPainter()
        self.picture = QPixmap(480, 100)
        self.end_dot_list = []
        self.x_num = 100
        self.x_step = 480 / self.x_num
        self.showConstellation = showConstellation(200, 200, self.constellation_show)

    def bindSingalandSlot(self):
        self.sendConfig.clicked.connect(self.sendConfigtoBaseBand)
        self.setDefault.clicked.connect(self.setDefaultConfig)
        self.start.clicked.connect(self.statisticalTimer)
        self.stop.clicked.connect(self.killStatisticalTimer)
        self.sendData.clicked.connect(self.startDataTimer)
        self.stopData.clicked.connect(self.stopDataTimerandDataSource)
        self.statisticalPort_obj.editingFinished.connect(self.setStatisticalPort)

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

    def calculateOFDMSymbleNumRX(self, car_num, mtype):
        MTypeToBit = [2, 4, 6, 8]
        car = self.calculateValidBitNum(car_num)
        m_len = car * 64 * MTypeToBit[mtype] / 8
        return int(m_len)

    def connectTXConfigData(self):
        self.TXConfig.clear()
        flag = 1
        self.TXConfig.extend((flag).to_bytes(1, byteorder='big'))
        papr_en = 1 if self.papr_en_obj.isChecked() else 0
        self.TXConfig.extend((papr_en).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.depapr_thr_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend((self.depapr_gain_obj.value()).to_bytes(2, byteorder='big'))
        self.TXConfig.extend((self.MType_tx_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.TXConfig.extend((self.car_num_tx_obj.value()).to_bytes(3, byteorder='big'))
        self.TXConfig.extend(int((self.Alpha_tx_obj.value() * 32768)).to_bytes(3, byteorder='big'))
        self.TXConfig.extend(int(self.pilot_factor_obj.value()).to_bytes(2, byteorder='big'))
        self.TXConfig.extend(int(self.pss_factor_obj.value()).to_bytes(2, byteorder='big'))
        self.TXConfig.extend((self.ModeBSorMS_obj.currentIndex()).to_bytes(1, byteorder='big'))
        Sys_reset = 1 if self.Sys_reset_obj.isChecked() else 0
        self.TXConfig.extend((Sys_reset).to_bytes(1, byteorder='big'))
        Sys_enble = 1 if self.Sys_enble_obj.isChecked() else 0
        self.TXConfig.extend((Sys_enble).to_bytes(1, byteorder='big'))
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
        mLen = self.calculateOFDMSymbleNum(self.car_num_tx_obj.value(), self.MType_tx_obj.currentIndex())
        self.TXConfig.extend((mLen).to_bytes(2, byteorder='big'))
        maxLen = self.calculateOFDMSymbleNum(self.car_num_tx_obj.value(), self.MType_tx_obj.currentIndex()) * 12 - 12
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
        data_len = self.calculateOFDMSymbleNumRX(self.car_num_rx_obj.value(), self.MType_rx_obj.currentIndex())
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

    def connectDefaultConfig(self):
        # 设置发端默认参数
        self.papr_en_obj.setChecked(True)
        self.depapr_thr_obj.setValue(38050)
        self.depapr_gain_obj.setValue(16384)
        self.MType_tx_obj.setCurrentIndex(0)
        self.car_num_tx_obj.setValue(0xfffff)
        self.Alpha_tx_obj.setValue(0.5)
        self.pilot_factor_obj.setValue(32767)
        self.pss_factor_obj.setValue(32767)
        self.ModeBSorMS_obj.setCurrentIndex(1)
        self.Sys_reset_obj.setChecked(False)
        self.Sys_enble_obj.setChecked(True)
        self.ModeUDPorPN_obj.setCurrentIndex(0)
        self.Base_loop_en_obj.setCurrentIndex(0)
        self.Rx_enb_obj.setCurrentIndex(0)
        self.Rx_channel_obj.setCurrentIndex(0)
        self.Rx_delay_obj.setValue(20)
        self.Time_trig2tx_obj.setValue(34000)
        self.Time_tx_hold_obj.setValue(34000)
        self.ms_T_sync2trig_obj.setValue(21880)
        self.bs_tdd_time_gap_obj.setValue(100000)
        self.bs_tx_time_gap_obj.setValue(50000)
        self.Trig_gap_cnt_obj.setValue(100000)
        self.alway_tx_obj.setCurrentIndex(1)
        self.tx1_en_obj.setCurrentIndex(0)
        self.tx2_en_obj.setCurrentIndex(0)
        self.rx1_en_obj.setCurrentIndex(0)
        self.rx2_en_obj.setCurrentIndex(0)
        self.TDD_EN_obj.setCurrentIndex(0)
        self.UDP_loop_obj.setCurrentIndex(0)

        # 设置收端默认参数
        self.MType_rx_obj.setCurrentIndex(0)
        self.agc_en_obj.setCurrentIndex(1)
        self.frft_en_obj.setCurrentIndex(1)
        self.car_num_rx_obj.setValue(0xfffff)
        self.i_freq_est_obj.setValue(0)
        self.sync_factor_obj.setValue(20384)
        self.equa_factor_obj.setValue(4022)
        self.Alpha_rx_obj.setValue(0.5)
        self.equa_amp_obj.setValue(16384)
        self.equa_amp_en_obj.setCurrentIndex(1)
        self.car_thr1_obj.setValue(102656)
        self.car_thr2_obj.setValue(6250000)
        self.car_thr3_obj.setValue(625000000)
        self.mmse_thr_obj.setValue(1000)

        # 设置射频参数
        self.Amplifier_obj.setCurrentIndex(0)
        self.filter3_3p5G_obj.setCurrentIndex(0)
        self.filter3p5_4G_obj.setCurrentIndex(0)
        self.filter4p5_5G_obj.setCurrentIndex(0)
        self.filter5_5p5G_obj.setCurrentIndex(0)

    def sendConfigtoBaseBand(self):
        self.ipBB = self.BBIP_obj.text()
        self.portBB = int(self.BBPort_obj.text())
        self.ipRF = self.RFIP_obj.text()
        self.portRF = int(self.RFPort_obj.text())
        addrBB = (self.ipBB, self.portBB)
        addRF = (self.ipRF, self.portRF)
        configSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.connectRFConfigData()
        print(self.RFConfig)
        for configRF in self.RFConfig:
            configSocket.sendto(configRF, addRF)
            print(configRF)
        self.connectTXConfigData()
        print(self.TXConfig)
        print(self.TXConfigBytes)
        configSocket.sendto((self.TXConfigBytes), addrBB)
        time.sleep(1)
        self.connectRXConfigData()
        print(self.RXConfig)
        print(self.RXConfigBytes)
        configSocket.sendto(self.RXConfigBytes, addrBB)
        configSocket.close()

    def setDefaultConfig(self):
        self.connectDefaultConfig()
        self.sendConfigtoBaseBand()

    def count_dot(self, value):
        self.beg_x = 0
        self.beg_y = 100
        if len(self.end_dot_list) >= (self.x_num+1):
            self.end_dot_list = self.end_dot_list[-self.x_num: ]
            for i in self.end_dot_list:
                i[0] -= self.x_step
        if len(self.end_dot_list) == 0:
            x = 0
        else:
            x = self.end_dot_list[-1][0] + self.x_step
        y = value
        self.end_dot_list.append([x, y])

        self.picture.fill(Qt.white)
        self.read_dot()

    def read_dot(self):
        for end_dot_list in self.end_dot_list:
            self.end_x = end_dot_list[0]
            self.end_y = 100 - end_dot_list[1]
            self.uptate_show()
        self.SNR_show.setPixmap(self.picture)

    #绘制函数
    def uptate_show(self):
        self.draw.begin(self.picture)
        self.draw.setPen(QPen(QColor("red"), 1))
        self.draw.drawLine(QPoint(self.beg_x, self.beg_y), QPoint(self.end_x, self.end_y))
        self.draw.end()
        self.beg_x = self.end_x
        self.beg_y = self.end_y

    def showStatistical(self, data):
        errbit = int.from_bytes(data[0:4], byteorder='big')
        self.errbit_show.setText(str(errbit))
        tolfrm = int.from_bytes(data[4:8], byteorder='big')
        self.tolfrm_show.setText(str(tolfrm))
        noise_pwr_sum = int.from_bytes(data[8:10], byteorder='big')
        self.noise_pwr_sum_show.setText(str(noise_pwr_sum))
        singal_pwr_sum = int.from_bytes(data[10:12], byteorder='big')
        self.singal_pwr_sum_show.setText(str(singal_pwr_sum))
        o_freq_est_r = int.from_bytes(data[12:14], byteorder='big')
        self.o_freq_est_r_show.setText(str(o_freq_est_r))
        o_freq_est_t = int.from_bytes(data[14:16], byteorder='big')
        self.o_freq_est_t_show.setText(str(o_freq_est_t))
        crc_error = int.from_bytes(data[16:17], byteorder='big')
        self.crc_error_show.setText(str(crc_error))
        error_dft = int.from_bytes(data[17:18], byteorder='big')
        self.error_dft_show.setText(str(error_dft))
        hard_decision_err = int.from_bytes(data[18:19], byteorder='big')
        self.hard_decision_err_show.setText(str(hard_decision_err))
        SNR = 0 if (noise_pwr_sum == 0) else (10 * math.log10(singal_pwr_sum * math.pow(2, 9) / noise_pwr_sum))
        self.SNR_current_show.setText(str(SNR))
        self.count_dot(SNR)
        consData = data[20:]
        self.showConstellation.count_dot(consData, 100, 100)

    def updateStatistical(self):
        # print('timer run')
        self.showStatistical(dataStatistical)

    def statisticalTimer(self):
        self.timer.start(10)
        self.timer.timeout.connect(self.updateStatistical)

    def killStatisticalTimer(self):
        print('statistical timer stop')
        self.timer.stop()

    def startDataTimer(self):
        self.dataTimer.start(1)
        self.dataTimer.timeout.connect(self.sendDataFromSource)

    def sendDataFromSource(self):
        global startFlag
        startFlag = True
        if not dataQueue.empty():
            data = dataQueue.get()
            addr = (self.ipData, self.portData)
            dataSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #print(data)
            dataSocket.sendto(data, addr)
            dataSocket.close()
            print('data send')

    def stopDataTimerandDataSource(self):
        global startFlag
        startFlag = False
        self.dataTimer.stop()
        print('data stop')

    def setStatisticalPort(self):
        global portStatistical
        port = int(self.statisticalPort_obj.text())
        if 1024 <= port < 65535:
            portStatistical = port
        else:
            if(self.statisticalPort_obj.hasFocus()):
                QMessageBox.information(self, "Tips", "1024 <= statisticalPort < 65535")


if __name__ == "__main__":
    workThread = WorkThread()
    workThread.start()
    dataThread = DataThread()
    dataThread.start()
    app = QApplication(sys.argv)
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())
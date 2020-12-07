# -*- coding:utf-8 -*-
import sys, math, socket, queue, time, datetime, encodings.idna
from array import array
from mainWinUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QGridLayout, QStyleFactory
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from othres import getChosenIP, ConnectRFConfig, print_bytes_hex, getStatisticalPort

'''def bytesToFloat(h1, h2, h3, h4):
    ba = bytearray()
    ba.append(h1)
    ba.append(h2)
    ba.append(h3)
    ba.append(h4)
    return struct.unpack("!f", ba)[0]'''

class StatisticThread(QThread):
    def __int__(self):
        super(StatisticThread, self).__init__()

    def run(self):
        # global portStatistical
        # portStatistical = 7000
        global statisticalQueue
        statisticalQueue = queue.Queue(0)
        while True:
            try:
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, portStatistical)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                statisticalQueue.put(data)
                # print("收到数据：", time.strftime('%H-%M-%S',time.localtime(time.time())))
            except socket.timeout:
                # print("调制解调数据超时：", time.strftime('%H-%M-%S',time.localtime(time.time())))
                pass
            statisticalSocket.close()

class LDPCStatisticThread(QThread):
    def __int__(self):
        super(LDPCStatisticThread, self).__init__()

    def run(self):
        #global LDPCportStatistical
        #LDPCportStatistical = 7010
        global LDPCstatisticalQueue
        LDPCstatisticalQueue = queue.Queue(0)
        while True:
            try:
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, LDPCportStatistical)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                LDPCstatisticalQueue.put(data)
                # print("收到数据：", time.strftime('%H-%M-%S',time.localtime(time.time())))
            except socket.timeout:
                # print("LDPC数据超时：", time.strftime('%H-%M-%S',time.localtime(time.time())))
                pass
            statisticalSocket.close()

class SpectrumStatisticThread(QThread):
    def __int__(self):
        super(SpectrumStatisticThread, self).__init__()

    '''def buildUdpSocket(self, port, buffsize):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(True) #设置阻塞模式
        self.socket.settimeout(1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buffsize * 80)
        addr = (ipaddr, port)
        self.socket.bind(addr)'''

    def run(self):
        # global SpectrumPortStatistical
        # SpectrumPortStatistical = 7020
        global SpectrumStatisticalQueue
        SpectrumStatisticalQueue = queue.Queue(0)
        while True:
            try:
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, SpectrumPortStatistical)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                SpectrumStatisticalQueue.put(data)
            except socket.timeout:
                # print("频谱数据超时：", time.strftime('%H-%M-%S',time.localtime(time.time())))
                pass
            statisticalSocket.close()

class IQThread(QThread):
    def __int__(self):
        super(IQThread, self).__init__()

    def run(self):
        # global portIQ
        # portIQ = 7001
        global IQQueue
        IQQueue = queue.Queue(0)
        while True:
            try:
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, portIQ)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                IQQueue.put(data)
                # print("收到数据：", time.strftime('%H-%M-%S',time.localtime(time.time())))
            except socket.timeout:
                # print("调制解调数据超时：", time.strftime('%H-%M-%S',time.localtime(time.time())))
                pass
            statisticalSocket.close()
'''class DataThread(QThread):
    def __int__(self):
        super(DataThread, self).__init__()

    def run(self):
        global startFlag
        startFlag = True
        global dataQueue
        dataQueue = queue.Queue(0)
        hostName = socket.gethostname()
        ipLocal = socket.gethostbyname(hostName)
        portData = 63000
        buffSize = 1500
        addr = (ipLocal, portData)
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
                    # print('dataSocket timeout')
                    pass
        # socketSource.close()'''

class Figure_Canvas(FigureCanvas):
    def __init__(self,parent=None,width=3.9,height=2.7,dpi=100):
        self.fig=Figure(figsize=(width,height),dpi=100)
        super(Figure_Canvas,self).__init__(self.fig)
        self.ax=self.fig.add_subplot(111)

class configPage(QMainWindow, Ui_MainWindow):
    _signal = pyqtSignal(int)
    def __init__(self):
        super(configPage, self).__init__()
        self.setupUi(self)
        self.setWindowTitle('FMC205基带射频配置')
        self.setWindowIcon(QIcon('rss1.ico'))
        self.init()
        self.bindSingalandSlot()

    def init(self):
        self.TXConfig = []
        self.RXConfig = []
        self.RFConfig = ConnectRFConfig()
        self.LDPCConfig = []
        self.IdataIn = []
        self.QdataIn = []
        self.IdataOut = []
        self.QdataOut = []
        self.QdataPilot = []
        self.QdataSpectrum = []
        self.IdataSpectrum = []
        self.SNRDataArray = []
        self.TXConfigBytes = bytes()
        self.RXConfigBytes = bytes()
        self.LDPCConfigBytes = bytes()
        self.ipBB = '192.168.1.84'
        self.portBB = 8000
        self.ipRF = '192.168.1.38'
        self.portRF = 6005
        self.ipData = '192.168.1.84'
        self.portData = 8002
        self.ipLDPC = '192.168.1.85'
        self.portLDPC = 8000
        self.timer = QTimer()
        # self.dataTimer = QTimer()
        self.LDPCtimer = QTimer()
        self.spectrumTimer = QTimer()
        self.IQtimer = QTimer()
        self.RFFrequencyReceived = 1.6
        self.PrepareIQinCanvas()
        self.PrepareIQOutCanvas()
        self.PreparePilotLineCanvas()
        self.PrepareSpectrumLineCanvas()
        self.PrepareSpectrumIdata()
        self.PrepareSNRLineCanvas()
        self.statisticalPort_obj.setText(str(portStatistical))
        self.LDPCstatisticalPort_obj.setText(str(LDPCportStatistical))
        self.spectrumPort_obj.setText(str(SpectrumPortStatistical))
        self.IQPort_obj.setText(str(portIQ))
        self.Reserv_5_obj.setVisible(False)
        self.Reserv_5_label.setVisible(False)
        self.Reserv_6_obj.setVisible(False)
        self.Reserv_6_label.setVisible(False)
        self.Reserv_I0_obj.setVisible(False)
        self.Reserv_I0_label.setVisible(False)
        self.Reserv_I1_obj.setVisible(False)
        self.Reserv_I1_label.setVisible(False)
        self.Reserv_I2_obj.setVisible(False)
        self.Reserv_I2_label.setVisible(False)
        self.Reserv_U0_obj.setVisible(False)
        self.Reserv_U0_label.setVisible(False)
        self.Reserv_U1_obj.setVisible(False)
        self.Reserv_U1_label.setVisible(False)
        self.Rx_channel_obj.setVisible(False)
        self.Rx_channel_label.setVisible(False)
        self.FIFOEmpty_show_2.setVisible(False)
        self.FIFOEmpty_label_2.setVisible(False)
        self.phase_est_show_2.setVisible(False)
        self.phase_est_label_2.setVisible(False)
        self.o_freq_est_t_show_2.setVisible(False)
        self.o_freq_est_t_label_2.setVisible(False)
        self.singal_pwr_sum_show_2.setVisible(False)
        self.singal_pwr_sum_label_2.setVisible(False)
        self.noise_pwr_sum_show_2.setVisible(False)
        self.noise_pwr_sum_label_2.setVisible(False)
        self.SNR_current_show_2.setVisible(False)
        self.SNR_current_label_2.setVisible(False)

    def bindSingalandSlot(self):
        self.sendBBConfig.clicked.connect(self.sendBBandLDPCConfig)
        self.setBBDefault.clicked.connect(self.setBBandLDPCConfig)
        self.setRFDefault.clicked.connect(self.setRFConfig)
        self.start.clicked.connect(self.statisticalTimer)
        self.start.clicked.connect(self.startIQTimer)
        self.stop.clicked.connect(self.killStatisticalTimer)
        self.stop.clicked.connect(self.killIQTimer)
        self.start_LDPC.clicked.connect(self.LDPCStatisticalTimer)
        self.stop_LDPC.clicked.connect(self.killLDPCStatisticalTimer)
        self.start_spectrum.clicked.connect(self.startSpectrumTimer)
        self.stop_spectrum.clicked.connect(self.killSpectrumTimer)
        self.statisticalPort_obj.editingFinished.connect(self.setStatisticalPort)
        self.LDPCstatisticalPort_obj.editingFinished.connect(self.setLDPCStatisticalPort)
        self.spectrumPort_obj.editingFinished.connect(self.setSpectrumStatisticalPort)
        self.IQPort_obj.editingFinished.connect(self.setIQPort)
        self.RF_send_freq_obj.returnPressed.connect(self.RFSendFreqChanged)
        self.RF_receive_freq_obj.returnPressed.connect(self.RFReceiveFreqChanged)
        self.RF_receive_band_obj.returnPressed.connect(self.RFReceiveBandChanged)
        self.RF_TX1_channel_obj.valueChanged.connect(self.RFTXChannelChanged)
        self.RF_TX2_channel_obj.valueChanged.connect(self.RFTXChannelChanged)
        self.RF_RX_AGC.stateChanged.connect(self.RFRXGainChanged)
        self.RF_RX1_gain_obj.valueChanged.connect(self.RFRXGainChanged)
        self.RF_RX2_gain_obj.valueChanged.connect(self.RFRXGainChanged)
        self.RF_observe_AGC.stateChanged.connect(self.RFORXGainChanged)
        self.RF_ORX1_gain_obj.valueChanged.connect(self.RFORXGainChanged)
        self.RF_ORX2_gain_obj.valueChanged.connect(self.RFORXGainChanged)
        self.RF_DRXA_gain_obj.valueChanged.connect(self.RFDRXGainChanged)
        self.RF_TX1_enable.stateChanged.connect(self.RFTRChannelChanged)
        self.RF_TX2_enable.stateChanged.connect(self.RFTRChannelChanged)
        self.RF_RX1_enable.stateChanged.connect(self.RFTRChannelChanged)
        self.RF_RX2_enable.stateChanged.connect(self.RFTRChannelChanged)
        self.BB_sample_rate_obj.activated.connect(self.BBSampleRateChanged)
        self.Ref_Clock_obj.activated.connect(self.RFClockChanged)
        self.source_Clock_obj.activated.connect(self.RFClockChanged)
        self.observation_channel_obj.activated.connect(self.RFObservationChannelChanged)
        self.Amplifier_obj.activated.connect(self.RFAmplifierChanged)
        self.filterOption.activated.connect(self.RFFilterChanged)
        self.RFCalibration.clicked.connect(self.RFCalibrationStatusChanged)

    def closeEvent(self,event):
        reply = QMessageBox.information(self, '警告',"系统将退出，是否确认?", QMessageBox.Yes |QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()

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
        depar_gain = int(1/self.depapr_gain_obj.value()*math.pow(2, 14))
        self.TXConfig.extend((depar_gain).to_bytes(2, byteorder='big'))
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
        self.TXConfig.extend((self.udpfifo_reset_obj.currentIndex()).to_bytes(1, byteorder='big'))
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
        equa_amp = int(self.equa_amp_obj.value() * math.pow(2, 14))
        self.RXConfig.extend((equa_amp).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.equa_amp_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.car_thr1_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfig.extend((self.car_thr2_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfig.extend((self.car_thr3_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfig.extend((self.mmse_thr_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfig.extend((self.scale_equa_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.scale_fft_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.phase_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.phase_factor_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.freq_offset_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.LDPC_en_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.MMSEorLS_obj.currentIndex().to_bytes(1, byteorder='big')))
        self.RXConfig.extend((self.as_time_trig2tx_obj.value()).to_bytes(3, byteorder='big'))
        self.RXConfig.extend((self.as_trig2tx_cnt_obj.value()).to_bytes(3, byteorder='big'))
        Reserv_B0 = 1 if self.Reserv_B0_obj.isChecked() else 0
        self.RXConfig.extend((Reserv_B0).to_bytes(1, byteorder='big'))
        Reserv_B1 = 1 if self.Reserv_B1_obj.isChecked() else 0
        self.RXConfig.extend((Reserv_B1).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.Reserv_U0_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.Reserv_U1_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.Reserv_I0_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.Reserv_I1_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.Reserv_I2_obj.value()).to_bytes(2, byteorder='big'))
        localIPArray = list(map(int, ipaddr.split(".")))
        self.RXConfig = self.RXConfig + localIPArray
        self.RXConfigBytes = bytes(self.RXConfig)

    def connectLDPCConfig(self):
        modeTable = [0, 13, 26, 39]
        self.LDPCConfig.clear()
        flag = 0
        self.LDPCConfig.extend((flag).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.LDPC_loop_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.Pause_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.MType_tx_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.MType_rx_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.car_num_tx_obj.value()).to_bytes(3, byteorder='big'))
        self.LDPCConfig.extend((self.car_num_rx_obj.value()).to_bytes(3, byteorder='big'))
        if self.calculateValidBitNum(self.car_num_tx_obj.value()) > 7:
            modeTx = self.calculateValidBitNum(self.car_num_tx_obj.value()) - 8 + modeTable[self.MType_tx_obj.currentIndex()]
        else:
            modeTx = 255
        if self.calculateValidBitNum(self.car_num_rx_obj.value()) > 7:
            modeRx = self.calculateValidBitNum(self.car_num_rx_obj.value()) - 8 + modeTable[self.MType_rx_obj.currentIndex()]
        else:
            modeRx = 255
        self.LDPCConfig.extend((modeTx).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((modeRx).to_bytes(1, byteorder='big'))
        LDPCreset = 1 if self.LDPC_reset_obj.isChecked() else 0
        self.LDPCConfig.extend((LDPCreset).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.LDPC_UDPorPN_obj.currentIndex()).to_bytes(2, byteorder='big'))
        self.LDPCConfig.extend(int(self.FFTManual_obj.currentIndex()).to_bytes(2, byteorder='big'))
        self.LDPCConfig.extend(int(self.FFTTrigger_obj.currentIndex()).to_bytes(2, byteorder='big'))
        localIPArray = list(map(int, ipaddr.split(".")))
        self.LDPCConfig = self.LDPCConfig + localIPArray
        self.LDPCConfig.extend((self.Reserv_5_obj.value()).to_bytes(4, byteorder='big'))
        self.LDPCConfig.extend((self.Reserv_6_obj.value()).to_bytes(4, byteorder='big'))
        self.LDPCConfigBytes = bytes(self.LDPCConfig)

    def setBBDefaultConfig(self):
        # 设置发端默认参数
        self.papr_en_obj.setChecked(True)
        self.depapr_thr_obj.setValue(38050)
        self.depapr_gain_obj.setValue(1.0)
        self.MType_tx_obj.setCurrentIndex(0)
        self.car_num_tx_obj.setValue(0xfffff)
        self.Alpha_tx_obj.setValue(0.5)
        self.pilot_factor_obj.setValue(16384)
        self.pss_factor_obj.setValue(16384)
        self.ModeBSorMS_obj.setCurrentIndex(1)
        self.Sys_reset_obj.setChecked(False)
        self.Sys_enble_obj.setChecked(True)
        self.ModeUDPorPN_obj.setCurrentIndex(1)
        self.Base_loop_en_obj.setCurrentIndex(1)
        self.Rx_enb_obj.setCurrentIndex(0)
        self.Rx_channel_obj.setCurrentIndex(0)
        self.Rx_delay_obj.setValue(20)
        self.Time_trig2tx_obj.setValue(13000)
        self.Time_tx_hold_obj.setValue(40000)
        self.ms_T_sync2trig_obj.setValue(23880)
        self.bs_tdd_time_gap_obj.setValue(100000)
        self.bs_tx_time_gap_obj.setValue(50000)
        self.Trig_gap_cnt_obj.setValue(100000)
        self.alway_tx_obj.setCurrentIndex(0)
        self.tx1_en_obj.setCurrentIndex(0)
        self.udpfifo_reset_obj.setCurrentIndex(0)
        self.rx1_en_obj.setCurrentIndex(1)
        self.rx2_en_obj.setCurrentIndex(0)
        self.TDD_EN_obj.setCurrentIndex(0)
        self.UDP_loop_obj.setCurrentIndex(0)
        self.Reserv_B0_obj.setChecked(False)
        self.Reserv_B1_obj.setChecked(False)

        # 设置收端默认参数
        self.MType_rx_obj.setCurrentIndex(0)
        self.agc_en_obj.setCurrentIndex(1)
        self.frft_en_obj.setCurrentIndex(1)
        self.car_num_rx_obj.setValue(0xfffff)
        self.i_freq_est_obj.setValue(0)
        self.sync_factor_obj.setValue(20384)
        self.equa_factor_obj.setValue(4122)
        self.Alpha_rx_obj.setValue(0.5)
        self.equa_amp_obj.setValue(1.0)
        self.equa_amp_en_obj.setCurrentIndex(1)
        self.car_thr1_obj.setValue(102656)
        self.car_thr2_obj.setValue(6250000)
        self.car_thr3_obj.setValue(625000000)
        self.mmse_thr_obj.setValue(1000)
        self.scale_equa_obj.setValue(0)
        self.scale_fft_obj.setValue(0)
        self.phase_en_obj.setCurrentIndex(1)
        self.phase_factor_obj.setValue(2048)
        self.freq_offset_en_obj.setCurrentIndex(1)
        self.LDPC_en_obj.setCurrentIndex(1)
        self.MMSEorLS_obj.setCurrentIndex(0)
        self.as_time_trig2tx_obj.setValue(17339)
        self.as_trig2tx_cnt_obj.setValue(58735)

    def setRFDefaultConfig(self):
        # 设置射频参数
        self.Amplifier_obj.setCurrentIndex(0)
        self.filterOption.setCurrentIndex(0)
        self.FPGA_option_obj.setCurrentIndex(0)
        self.FMC_option_obj.setCurrentIndex(0)
        self.RF_send_freq_obj.setText('2.5')
        self.RF_receive_freq_obj.setText('1.6')
        self.RF_receive_band_obj.setText('2.0')
        self.RF_TX1_channel_obj.setValue(10.0)
        self.RF_TX2_channel_obj.setValue(10.0)
        self.RF_RX_AGC.setChecked(False)
        self.RF_RX1_gain_obj.setValue(30.0)
        self.RF_RX2_gain_obj.setValue(30.0)
        self.RF_observe_AGC.setChecked(False)
        self.RF_ORX1_gain_obj.setValue(18)
        self.RF_ORX2_gain_obj.setValue(18)
        self.RF_DRXA_gain_obj.setValue(52)
        self.RF_RX1_enable.setChecked(True)
        self.RF_RX2_enable.setChecked(True)
        self.RF_TX1_enable.setChecked(True)
        self.RF_TX2_enable.setChecked(True)
        self.BB_sample_rate_obj.setCurrentIndex(4)
        self.Ref_Clock_obj.setCurrentIndex(2)
        self.source_Clock_obj.setCurrentIndex(2)
        self.observation_channel_obj.setCurrentIndex(1)


    def setLDPCDefaultConfig(self):
        # 设置LDPC配置参数
        self.LDPC_loop_obj.setCurrentIndex(0)
        self.Pause_obj.setCurrentIndex(0)
        self.LDPC_reset_obj.setChecked(False)
        self.LDPC_UDPorPN_obj.setCurrentIndex(0)
        self.FFTManual_obj.setCurrentIndex(0)
        self.FFTTrigger_obj.setCurrentIndex(0)

    def sendConfigtoBaseBand(self):
        self.ipBB = self.BBIP_obj.text()
        try:
            self.portBB = int(self.BBPort_obj.text())
        except ValueError:
            print("基带端口配置无效")
            return
        addrBB = (self.ipBB, self.portBB)
        configSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        '''try:
            configSocket.bind((ipaddr, 9000))
        except OSError:
            print('发送基带配置：IP端口绑定失败')
            return'''
        self.connectTXConfigData()
        print("表1配置(基带发端配置)：", self.TXConfig)
        configSocket.sendto((self.TXConfigBytes), addrBB)
        time.sleep(1)
        self.connectRXConfigData()
        print("表2配置(基带收端配置)：", self.RXConfig)
        configSocket.sendto(self.RXConfigBytes, addrBB)
        configSocket.close()
        return

    def sendConfigtoRF(self, data, feedbacktime=0.1):
        self.ipRF = self.RFIP_obj.text()
        try:
            self.portRF = int(self.RFPort_obj.text())
        except ValueError:
            self.textBrowser_2.append("<font color='red'>" + "射频端口配置无效")
            return
        addrRF = (self.ipRF, self.portRF)
        configSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        '''try:
            configSocket.bind((ipaddr, 9001))
        except OSError:
            self.textBrowser_2.append("<font color='red'>"+"发送射频配置：IP端口绑定失败")
            return'''
        self.textBrowser_2.append("发送配置数据：")
        self.textBrowser_2.append(print_bytes_hex(data))
        configSocket.sendto(data, addrRF)
        configSocket.settimeout(feedbacktime)
        try:
            configData, addr = configSocket.recvfrom(1500)
        except socket.timeout:
            self.textBrowser_2.append("<font color='red'>"+"硬件未响应，配置失败！！")
            configSocket.close()
            return
        if configData == data:
            self.textBrowser_2.append("<font color='green'>"+"配置成功！")
        else:
            self.textBrowser_2.append("<font color='red'>"+"上位机与硬件配置不一致，配置失败！！硬件返回配置为：")
            self.textBrowser_2.append(print_bytes_hex(configData))
        configSocket.close()
        return

    def sendConfigtoLDPC(self):
        self.ipLDPC = self.LDPC_IP_obj.text()
        try:
            self.portLDPC = int(self.LDPC_Port_obj.text())
        except ValueError:
            print("LDPC端口配置无效")
            return
        addrLDPC = (self.ipLDPC, self.portLDPC)
        configSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        '''try:
            configSocket.bind((ipaddr, 9002))
        except OSError:
            print('发送LDPC设置：IP端口绑定失败')
            return'''
        self.connectLDPCConfig()
        print("表8配置(LDPC配置)：", self.LDPCConfig)
        configSocket.sendto(self.LDPCConfigBytes, addrLDPC)
        configSocket.close()
        return

    def setBBConfig(self):
        self.setBBDefaultConfig()
        # self.sendConfigtoBaseBand()

    def setRFConfig(self):
        self.setRFDefaultConfig()
        '''self.RFSendFreqChanged()
        self.RFReceiveFreqChanged()
        self.RFReceiveBandChanged()
        self.RFTXChannelChanged()
        self.RFRXGainChanged()
        self.RFORXGainChanged()
        self.RFDRXGainChanged()
        self.RFTRChannelChanged()
        self.BBSampleRateChanged()
        self.RFClockChanged()
        self.RFObservationChannelChanged()
        self.RFAmplifierChanged()
        self.RFFilterChanged()'''

    def setLDPCConfig(self):
        self.setLDPCDefaultConfig()
        # self.sendConfigtoLDPC()

    def setBBandLDPCConfig(self):
        self.setBBConfig()
        self.setLDPCConfig()

    def sendBBandLDPCConfig(self):
        self.sendConfigtoBaseBand()
        self.sendConfigtoLDPC()

    def RFSendFreqChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[5]
        try:
            RFSendFreq = float(self.RF_send_freq_obj.text())
            data = self.RFConfig.connectFreqInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, RFSendFreq)
            self.textBrowser_2.append("<font color='blue'>"+"配置发送频率")
            self.sendConfigtoRF(data, feedbacktime=0.5)
        except ValueError:
            self.textBrowser_2.append("<font color='red'>" + "发送频率配置无效")

    def RFReceiveFreqChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[6]
        try:
            self.RFFrequencyReceived = float(self.RF_receive_freq_obj.text())
            data = self.RFConfig.connectFreqInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, self.RFFrequencyReceived)
            self.textBrowser_2.append("<font color='blue'>"+"设置接收频率")
            self.sendConfigtoRF(data, feedbacktime=0.5)
        except ValueError:
            self.textBrowser_2.append("<font color='red'>" + "接收频率配置无效")

    def RFReceiveBandChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[7]
        try:
            RFReceiveband = float(self.RF_receive_band_obj.text())
            data = self.RFConfig.connectFreqInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, RFReceiveband)
            self.textBrowser_2.append("<font color='blue'>"+"设置侦听频率")
            self.sendConfigtoRF(data)
        except ValueError:
            self.textBrowser_2.append("<font color='red'>" + "侦听频率配置无效")

    def RFTXChannelChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[8]
        RFTXChannel1Attenuation = float(self.RF_TX1_channel_obj.value())
        RFTXChannel2Attenuation = float(self.RF_TX2_channel_obj.value())
        data = self.RFConfig.connectTXAttenuationInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, RFTXChannel1Attenuation, RFTXChannel2Attenuation)
        self.textBrowser_2.append("<font color='blue'>"+"设置发射衰减")
        self.sendConfigtoRF(data)

    def RFRXGainChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[9]
        AGC = 1 if self.RF_RX_AGC.isChecked() else 0
        RX1Gain = float(self.RF_RX1_gain_obj.value())
        RX2Gain = float(self.RF_RX2_gain_obj.value())
        step = 0.5
        data = self.RFConfig.connectRXGainInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, AGC, RX1Gain, RX2Gain, step)
        self.textBrowser_2.append("<font color='blue'>"+"设置接收增益")
        self.sendConfigtoRF(data)

    def RFORXGainChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[10]
        AGC = 1 if self.RF_observe_AGC.isChecked() else 0
        ORX1Gain = float(self.RF_ORX1_gain_obj.value())
        ORX2Gain = float(self.RF_ORX2_gain_obj.value())
        step = 1
        data = self.RFConfig.connectRXGainInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, AGC, ORX1Gain, ORX2Gain, step)
        self.textBrowser_2.append("<font color='blue'>"+"设置观察接收增益")
        self.sendConfigtoRF(data)

    def RFDRXGainChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[11]
        DRXgain = float(self.RF_DRXA_gain_obj.value())
        data = self.RFConfig.connectDetectionRXGainInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, DRXgain)
        self.textBrowser_2.append("<font color='blue'>"+"设置侦测接收增益")
        self.sendConfigtoRF(data)

    def RFTRChannelChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[12]
        RX1Channel = 1 if self.RF_RX1_enable.isChecked() else 0
        RX2Channel = 1 if self.RF_RX2_enable.isChecked() else 0
        TX1Channel = 1 if self.RF_TX1_enable.isChecked() else 0
        TX2Channel = 1 if self.RF_TX2_enable.isChecked() else 0
        data = self.RFConfig.connectTXRXEnableInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, RX1Channel, RX2Channel, TX1Channel, TX2Channel)
        self.textBrowser_2.append("<font color='blue'>"+"设置收发通道使能")
        self.sendConfigtoRF(data)

    def BBSampleRateChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[13]
        sampleRate = int(self.BB_sample_rate_obj.currentIndex())
        data = self.RFConfig.connectBaseBandSampleRateInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, sampleRate)
        self.textBrowser_2.append("<font color='blue'>"+"设置基带采样率")
        self.sendConfigtoRF(data)

    def RFClockChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[14]
        RefClock = int(self.Ref_Clock_obj.currentIndex())
        sourceClock = int(self.source_Clock_obj.currentIndex())
        data = self.RFConfig.connectClockInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, RefClock, sourceClock)
        self.textBrowser_2.append("<font color='blue'>"+"设置时钟源选择")
        self.sendConfigtoRF(data)

    def RFObservationChannelChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[15]
        observationChannel = int(self.observation_channel_obj.currentIndex())
        data = self.RFConfig.connectObservationChannelInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, observationChannel)
        self.textBrowser_2.append("<font color='blue'>"+"设置观察通道选择")
        self.sendConfigtoRF(data)

    def RFCalibrationStatusChanged(self):
        WR_bit = 1
        FPGAOption = int(self.FPGA_option_obj.currentIndex())
        FMCOption = 1
        RegisterAddr = self.RFConfig.addrhead[16]
        calibrationValue = 1
        self.RF_Config.hide()
        QMessageBox.information(self, "提示：", "射频硬件校准中，请等待......")
        self.textBrowser_2.append("<font color='blue'>" + "硬件校准:")
        data = self.RFConfig.connectCalibrationInfo(FPGAOption, WR_bit, FMCOption, RegisterAddr, calibrationValue)
        self.sendConfigtoRF(data, feedbacktime=10)
        self.RF_Config.show()

    def RFAmplifierChanged(self):
        amplifierArray = [[0x81, 0x02, 0x10, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                          [0x81, 0x02, 0x10, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]]
        data = bytes(amplifierArray[self.Amplifier_obj.currentIndex()])
        self.textBrowser_2.append("<font color='blue'>"+"配置放大器")
        self.sendConfigtoRF(data)

    def RFFilterChanged(self):
        filterArray = [[0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01],
                       [0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                       [0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00],
                       [0x81, 0x02, 0x10, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]]
        data = bytes(filterArray[self.filterOption.currentIndex()])
        self.textBrowser_2.append("<font color='blue'>"+"配置滤波器")
        self.sendConfigtoRF(data)

    def showBasebandStatistical(self, data):
        errbit = int.from_bytes(data[0:4], byteorder='big')
        self.errbit_show.setText(str(errbit))
        tolfrm = int.from_bytes(data[4:8], byteorder='big')
        self.tolfrm_show.setText(str(tolfrm))
        noise_pwr_sum = int.from_bytes(data[8:10], byteorder='big')
        self.noise_pwr_sum_show.setText(str(noise_pwr_sum))
        singal_pwr_sum = int.from_bytes(data[10:12], byteorder='big')
        # print("singal_pwr_sum",data[10:12], singal_pwr_sum)
        self.singal_pwr_sum_show.setText(str(singal_pwr_sum))
        phase_est = int.from_bytes(data[12:14], byteorder='big', signed=True)
        self.phase_est_show.setText('{:.1f}'.format(phase_est/8096*45))
        o_freq_est_t = int.from_bytes(data[14:16], byteorder='big', signed=True)
        # print("o_freq_est_t:",data[14:16], o_freq_est_t)
        self.o_freq_est_t_show.setText(str(o_freq_est_t))
        crc_error = int.from_bytes(data[16:17], byteorder='big')
        # print("crc_error:",data[16:17], crc_error)
        self.crc_error_show.setText(str(crc_error))
        over_flag_rx = int.from_bytes(data[17:18], byteorder='big')
        self.over_flag_rx_show.setText(str(over_flag_rx))
        FIFOEmpty = int.from_bytes(data[18:19], byteorder='big')
        # print("FIFOEmpty:",data[18:19], FIFOEmpty)
        self.FIFOEmpty_show.setText(str(FIFOEmpty))
        if (noise_pwr_sum == 0):
            SNR = 50
        elif self.MType_rx_obj.currentIndex() == 3:
            SNR = (10 * math.log10(singal_pwr_sum * math.pow(2, 10) / noise_pwr_sum))
        else:
            SNR = (10 * math.log10(singal_pwr_sum * math.pow(2, 6) / noise_pwr_sum))
        self.SNR_current_show.setText('{:.1f}'.format(SNR))
        # self.count_dot(SNR)
        self.PrepareSNRSamples(SNR)
        car1 = int.from_bytes(data[19:22], byteorder='big')
        # print("car1_hex:", data[19:22], car1)
        self.car1_hex_show.setText(format(car1, 'x').zfill(5))
        car2 = int.from_bytes(data[22:25], byteorder='big')
        # print("car2_hex:", data[22:25], car2)
        self.car2_hex_show.setText(format(car2, 'x').zfill(5))
        car3 = int.from_bytes(data[25:28], byteorder='big')
        self.car3_hex_show.setText(format(car3, 'x').zfill(5))
        MtypeTable = ['QPSK', '16QAM', '64QAM', '256QAM']
        Rb_mtype_tx = int.from_bytes(data[28:29], byteorder='big')
        self.Rb_mtype_tx_show.setText(MtypeTable[Rb_mtype_tx])
        Rb_mtype_rx = int.from_bytes(data[29:30], byteorder='big')
        self.Rb_mtype_rx_show.setText(MtypeTable[Rb_mtype_rx])
        Rb_car_tx = int.from_bytes(data[30:33], byteorder='big')
        self.Rb_car_tx_show.setText(format(Rb_car_tx, 'x').zfill(5))
        Rb_car_rx = int.from_bytes(data[33:36], byteorder='big')
        # print("Rb_car_rx:", data[33:36], Rb_car_rx)
        self.Rb_car_rx_show.setText(format(Rb_car_rx, 'x').zfill(5))
        rx_frame_cnt = int.from_bytes(data[36:39], byteorder='big')
        self.cnt_rx_frame_show.setText(str(rx_frame_cnt))
        tx_frame_cnt = int.from_bytes(data[39:42], byteorder='big')
        self.cnt_frame_tx_show.setText(str(tx_frame_cnt))
        if tolfrm == 0:
            BER = 'Invalid'
            self.BER_show.setText(BER)
        else:
            M2bit = [2, 4, 6, 8]
            M = M2bit[self.MType_rx_obj.currentIndex()]
            BER = errbit/(((self.calculateValidBitNum(self.car_num_rx_obj.value())*64*M-32)*12)*tolfrm)
            self.BER_show.setText('{:.2e}'.format(BER))

    def showLDPCStatistical(self, data):
        errbit = int.from_bytes(data[0:4], byteorder='big')
        self.errbit_show_2.setText(str(errbit))
        tolfrm = int.from_bytes(data[4:8], byteorder='big')
        self.tolfrm_show_2.setText(str(tolfrm))
        noise_pwr_sum = int.from_bytes(data[8:10], byteorder='big')
        self.noise_pwr_sum_show_2.setText(str(noise_pwr_sum))
        singal_pwr_sum = int.from_bytes(data[10:12], byteorder='big')
        self.singal_pwr_sum_show_2.setText(str(singal_pwr_sum))
        phase_est = int.from_bytes(data[12:14], byteorder='big', signed=True)
        self.phase_est_show_2.setText('{:.1f}'.format(phase_est / 8096 * 45))
        o_freq_est_t = int.from_bytes(data[14:16], byteorder='big', signed=True)
        self.o_freq_est_t_show_2.setText(str(o_freq_est_t))
        crc_error = int.from_bytes(data[16:17], byteorder='big')
        self.crc_error_show_2.setText(str(crc_error))
        over_flag_rx = int.from_bytes(data[17:18], byteorder='big')
        self.over_flag_rx_show_2.setText(str(over_flag_rx))
        FIFOEmpty = int.from_bytes(data[18:19], byteorder='big')
        self.FIFOEmpty_show_2.setText(str(FIFOEmpty))
        SNR = 0 if (noise_pwr_sum == 0) else (10 * math.log10(singal_pwr_sum * math.pow(2, 9) / noise_pwr_sum))
        self.SNR_current_show_2.setText(str(SNR))
        # self.PrepareSNRSamples(SNR)
        car1 = int.from_bytes(data[19:22], byteorder='big')
        self.car1_hex_show_2.setText(format(car1, 'x').zfill(5))
        car2 = int.from_bytes(data[22:25], byteorder='big')
        self.car2_hex_show_2.setText(format(car2, 'x').zfill(5))
        car3 = int.from_bytes(data[25:28], byteorder='big')
        self.car3_hex_show_2.setText(format(car3, 'x').zfill(5))
        MtypeTable = ['QPSK', '16QAM', '64QAM', '256QAM']
        Rb_mtype_tx = int.from_bytes(data[28:29], byteorder='big')
        self.Rb_mtype_tx_show_2.setText(MtypeTable[Rb_mtype_tx])
        Rb_mtype_rx = int.from_bytes(data[29:30], byteorder='big')
        self.Rb_mtype_rx_show_2.setText(MtypeTable[Rb_mtype_rx])
        Rb_car_tx = int.from_bytes(data[30:33], byteorder='big')
        self.Rb_car_tx_show_2.setText(format(Rb_car_tx, 'x').zfill(5))
        Rb_car_rx = int.from_bytes(data[33:36], byteorder='big')
        self.Rb_car_rx_show_2.setText(format(Rb_car_rx, 'x').zfill(5))
        tab = [1148, 1340, 1532, 1596, 1788, 1980, 2044, 2236, 2428, 2492,  2684,  2876,  3068,
               2428, 2684, 3068, 3324, 3580, 3964, 4220, 4604, 4860, 5116,  5500,  5756,  6140,
               3580, 4028, 4604, 5052, 5500, 5948, 6396, 6844, 7292, 7740,  8188,  8636,  9212,
               4860, 5500, 6140, 6652, 7292, 7932, 8572, 9212, 9724, 10364, 11004, 11644, 12284]
        modeTable = [0, 13, 26, 39]
        if (self.calculateValidBitNum(self.car_num_rx_obj.value()) > 7) and (tolfrm != 0):
            modeRx = self.calculateValidBitNum(self.car_num_rx_obj.value()) - 8 + modeTable[
                self.MType_rx_obj.currentIndex()]
            BER = errbit/(tab[modeRx]*8*tolfrm)
            self.BER_show_2.setText('{:.2e}'.format(BER))
        else:
            BER = 'Invalid'
            self.BER_show_2.setText(BER)

    def PrepareIQinCanvas(self):
        self.IQinFigure = Figure_Canvas()
        # self.IQinFigure.ax.spines['top'].set_color('none')
        # self.IQinFigure.ax.spines['right'].set_color('none')
        # self.IQinFigure.ax.xaxis.set_ticks_position('bottom')
        # self.IQinFigure.ax.spines['bottom'].set_position(('data', 0))
        # self.IQinFigure.ax.yaxis.set_ticks_position('left')
        # self.IQinFigure.ax.spines['left'].set_position(('data', 0))
        self.IQinFigureLayout = QGridLayout(self.IQshowFRFTin)
        self.IQinFigureLayout.addWidget(self.IQinFigure)

    def PrepareIQOutCanvas(self):
        self.IQOutFigure = Figure_Canvas()
        self.IQOutFigureLayout = QGridLayout(self.IQShowFRFTout)
        self.IQOutFigureLayout.addWidget(self.IQOutFigure)

    def PreparePilotLineCanvas(self):
        self.LineFigure = Figure_Canvas()
        self.LineFigureLayout = QGridLayout(self.IQshowPilot)
        self.LineFigureLayout.addWidget(self.LineFigure)

    def PrepareSpectrumLineCanvas(self):
        self.SpectrumLineFigure = Figure_Canvas()
        self.SpectrumLineLayout = QGridLayout(self.SpectrumDetection)
        self.SpectrumLineLayout.addWidget(self.SpectrumLineFigure)

    def PrepareSpectrumIdata(self):
        self.IdataSpectrum.clear()
        data = list(range(2048))
        for element in data:
            if element <= 1022:
                newElement = 1022 - element
            else:
                newElement = 3070 - element
            self.IdataSpectrum.append(newElement)
        for i in range(2048):
            newElement02 = 2047 - self.IdataSpectrum[i]
            self.IdataSpectrum[i] = newElement02*125/2048 - 125/2


    def PrepareSNRLineCanvas(self):
        self.SNRLineFigure = Figure_Canvas()
        self.SNRLineFigureLayout = QGridLayout(self.SNRshow)
        self.SNRLineFigureLayout.addWidget(self.SNRLineFigure)

    def PrepareSNRSamples(self, value):
        self.SNRLineFigure.ax.cla()
        self.SNRLineFigure.ax.set_xlim(0, 100)
        #self.SNRLineFigure.ax.set_ylim(0, 50)
        if len(self.SNRDataArray) >= 100:
            self.SNRDataArray = self.SNRDataArray[-100:]
        self.SNRDataArray.append(value)
        # SNRLine = Line2D(list(range(len(self.SNRDataArray))), self.SNRDataArray)
        # self.SNRLineFigure.ax.add_line(SNRLine)
        self.SNRLineFigure.ax.plot(self.SNRDataArray)
        self.SNRLineFigure.draw()
        self.SNRLineFigure.flush_events()

    def getIQdata(self, sourceData, Qdata, Idata=[], targetLen=1280, dataType='pilot'):
        for i in range(int(len(sourceData)/4)):
            ivalue = int.from_bytes(sourceData[0:2], byteorder='big', signed=True)
            qvalue = int.from_bytes(sourceData[2:4], byteorder='big', signed=True)
            if dataType == 'pilot':
                Qdata.append((math.pow(ivalue, 2) + math.pow(qvalue, 2))/(math.pow(2, 26)))
            elif dataType == 'spectrum':
                if ivalue != 0 or qvalue != 0:
                    Qdata.append(20 * math.log10(math.sqrt(ivalue*ivalue + qvalue*qvalue)) - 96)
                else:
                    Qdata.append(-126)
            else:
                Idata.append(ivalue/65535)
                Qdata.append(qvalue/65535)
            sourceData = sourceData[4:]
        if len(Qdata) >= targetLen:
            return True
        else:
            return False

    def showStatistical(self, data):
        flag = int.from_bytes(data[0:1], byteorder='big')
        Statistical = data[1:]
        if flag == 1:
            # print(Statistical)
            self.showBasebandStatistical(Statistical)
        elif flag == 2:
            if(self.getIQdata(Statistical, self.QdataIn, self.IdataIn, 2048, 'IQin') == True):
                self.IQinFigure.ax.cla()
                self.IQinFigure.ax.set_autoscale_on(True)
                self.IQinFigure.ax.scatter(self.IdataIn, self.QdataIn, 3)
                self.IQinFigure.draw()
                self.IQinFigure.flush_events()
                self.IdataIn.clear()
                self.QdataIn.clear()
        elif flag == 3:
            if (self.getIQdata(Statistical, self.QdataOut, self.IdataOut, 2048, 'IQout') == True):
                self.IQOutFigure.ax.cla()
                self.IQOutFigure.ax.set_autoscale_on(True)
                self.IQOutFigure.ax.scatter(self.IdataOut, self.QdataOut, 3)
                self.IQOutFigure.draw()
                self.IQOutFigure.flush_events()
                self.IdataOut.clear()
                self.QdataOut.clear()
        elif flag == 4:
            # print(len(self.QdataPilot))
            if len(self.QdataPilot) == 1280:
                self.LineFigure.ax.cla()
                self.LineFigure.ax.set_autoscale_on(True)
                self.LineFigure.ax.plot(self.QdataPilot)
                self.LineFigure.draw()
                # print("draw Pilot")
                self.LineFigure.flush_events()
                self.QdataPilot.clear()
            else:
                self.QdataPilot.clear()
            self.getIQdata(Statistical, self.QdataPilot, [], 1280, 'pilot')

        elif flag == 5:
            # print(Statistical)
            self.showLDPCStatistical(Statistical)
        elif flag == 6:
            if len(self.QdataSpectrum) == 2048:
                self.SpectrumLineFigure.ax.cla()
                self.SpectrumLineFigure.ax.set_xlabel("Frequcy(MHz)")
                self.SpectrumLineFigure.ax.set_autoscale_on(True)
                IdataSpectrum = list(map(lambda x: x + self.RFFrequencyReceived*1000, self.IdataSpectrum))
                self.SpectrumLineFigure.ax.plot(IdataSpectrum, self.QdataSpectrum)
                # print("draw Spectrum!!")
                self.SpectrumLineFigure.draw()
                self.SpectrumLineFigure.flush_events()
                self.QdataSpectrum.clear()
            else:
                self.QdataSpectrum.clear()
            self.getIQdata(Statistical, self.QdataSpectrum, [], 2048, 'spectrum')
        elif flag == 7:
            self.getIQdata(Statistical, self.QdataSpectrum, [], 2048, 'spectrum')
            self.SpectrumLineFigure.ax.cla()
            self.SpectrumLineFigure.flush_events()
        elif flag == 8:
            self.getIQdata(Statistical, self.QdataPilot, [], 1280, 'pilot')
            # print("receive data")
            self.LineFigure.ax.cla()
            self.LineFigure.flush_events()
        else:
            print("无效的统计数据")

    def updateStatistical(self):
        if not statisticalQueue.empty():
            self.showStatistical(statisticalQueue.get())

    def updateLDPCStatistical(self):
        if not LDPCstatisticalQueue.empty():
            self.showStatistical(LDPCstatisticalQueue.get())

    def updateSpectrumStatistical(self):
        if not SpectrumStatisticalQueue.empty():
            self.showStatistical(SpectrumStatisticalQueue.get())

    def updateIQStatistical(self):
        if not IQQueue.empty():
            self.showStatistical(IQQueue.get())

    def statisticalTimer(self):
        self.timer.start(1)
        print('调制解调统计数据开始更新')
        self.timer.timeout.connect(self.updateStatistical)

    def killStatisticalTimer(self):
        print('停止显示调制解调统计数据')
        self.timer.stop()

    def LDPCStatisticalTimer(self):
        self.LDPCtimer.start(1)
        print('LDPC统计数据开始更新')
        self.LDPCtimer.timeout.connect(self.updateLDPCStatistical)

    def killLDPCStatisticalTimer(self):
        print('停止显示LDPC统计数据')
        self.LDPCtimer.stop()

    def startSpectrumTimer(self):
        self.spectrumTimer.start(1)
        print("频谱检测开始更新")
        self.spectrumTimer.timeout.connect(self.updateSpectrumStatistical)

    def killSpectrumTimer(self):
        print("停止显示频谱检测")
        self.spectrumTimer.stop()

    def startIQTimer(self):
        self.IQtimer.start(1)
        print("IQ显示开始更新")
        self.IQtimer.timeout.connect(self.updateIQStatistical)

    def killIQTimer(self):
        print("停止IQ显示更新")
        self.IQtimer.stop()

    '''def startDataTimer(self):
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
        print('data stop')'''

    def setStatisticalPort(self):
        global portStatistical
        try:
            port = int(self.statisticalPort_obj.text())
            if 1024 <= port < 65535:
                if port != int(self.IQPort_obj.text()):
                    portStatistical = port
                else:
                    if (self.statisticalPort_obj.hasFocus()):
                        QMessageBox.information(self, "Warnning", "该端口已被占用！")
            else:
                if(self.statisticalPort_obj.hasFocus()):
                    QMessageBox.information(self, "Tips", "1024 <= statisticalPort < 65535")
        except ValueError:
            print("调制解调统计端口配置无效")

    def setLDPCStatisticalPort(self):
        global LDPCportStatistical
        try:
            port = int(self.LDPCstatisticalPort_obj.text())
            if 1024 <= port < 65535:
                LDPCportStatistical = port
            else:
                if(self.LDPCstatisticalPort_obj.hasFocus()):
                    QMessageBox.information(self, "Tips", "1024 <= LDPCportStatistical < 65535")
        except ValueError:
            print("LDPC统计端口配置无效")

    def setSpectrumStatisticalPort(self):
        global SpectrumPortStatistical
        try:
            port = int(self.spectrumPort_obj.text())
            if 1024 <= port < 65535:
                SpectrumPortStatistical = port
            else:
                if(self.spectrumPort_obj.hasFocus()):
                    QMessageBox.information(self, "Tips", "1024 <= SpectrumPortStatistical < 65535")
        except ValueError:
            print("频谱检测端口配置无效")

    def setIQPort(self):
        global portIQ
        try:
            port = int(self.IQPort_obj.text())
            if 1024 <= port < 65535:
                if port != int(self.statisticalPort_obj.text()):
                    portIQ = port
                else:
                    if (self.IQPort_obj.hasFocus()):
                        QMessageBox.information(self, "Warnning", "该端口已被占用！")
            else:
                if(self.spectrumPort_obj.hasFocus()):
                    QMessageBox.information(self, "Tips", "1024 <= SpectrumPortStatistical < 65535")
        except ValueError:
            print("IQ端口配置无效")

if __name__ == "__main__":
    portStatistical, LDPCportStatistical, SpectrumPortStatistical, portIQ = getStatisticalPort()
    ipaddr = getChosenIP('1')
    statisticThread = StatisticThread()
    statisticThread.start()
    statisticThreadLDPC = LDPCStatisticThread()
    statisticThreadLDPC.start()
    spectrumThread = SpectrumStatisticThread()
    spectrumThread.start()
    IQdataThread = IQThread()
    IQdataThread.start()
    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create('Windows'))
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())
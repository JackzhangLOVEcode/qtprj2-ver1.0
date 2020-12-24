# -*- coding:utf-8 -*-
import sys, math, socket, queue, time, datetime, encodings.idna
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
        global BBstop
        BBstop = True
        global statisticalQueue
        statisticalQueue = queue.Queue(0)
        while True:
            try:
                if BBstop == True:
                    statisticalQueue.queue.clear()
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, portStatistical)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                if BBstop == False:
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
        global LDPCstop
        LDPCstop = True
        global LDPCstatisticalQueue
        LDPCstatisticalQueue = queue.Queue(0)
        while True:
            try:
                if LDPCstop == True:
                    LDPCstatisticalQueue.queue.clear()
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, LDPCportStatistical)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                if LDPCstop == False:
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
        global SpectrumStop
        SpectrumStop = True
        global SpectrumStatisticalQueue
        SpectrumStatisticalQueue = queue.Queue(0)
        while True:
            try:
                if SpectrumStop == True:
                    SpectrumStatisticalQueue.queue.clear()
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, SpectrumPortStatistical)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                if SpectrumStop == False:
                    SpectrumStatisticalQueue.put(data)
            except socket.timeout:
                # print("频谱数据超时：", time.strftime('%H-%M-%S',time.localtime(time.time())))
                pass
            statisticalSocket.close()


class IQThread(QThread):
    def __int__(self):
        super(IQThread, self).__init__()

    def run(self):
        global IQstop
        IQstop = True
        global IQQueue
        IQQueue = queue.Queue(0)
        while True:
            try:
                if IQstop == True:
                    IQQueue.queue.clear()
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, portIQ)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                if IQstop == False:
                    IQQueue.put(data)
                # print("收到数据：", time.strftime('%H-%M-%S',time.localtime(time.time())))
            except socket.timeout:
                # print("调制解调数据超时：", time.strftime('%H-%M-%S',time.localtime(time.time())))
                pass
            statisticalSocket.close()


class SSSysThread(QThread):
    def __int__(self):
        super(SSSysThread, self).__init__()

    def run(self):
        global SSSysStop
        SSSysStop = True
        global SSSysQueue
        SSSysQueue = queue.Queue(0)
        while True:
            try:
                if SSSysStop == True:
                    SSSysQueue.queue.clear()
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, portSSSys)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                if SSSysStop == False:
                    SSSysQueue.put(data)
            except socket.timeout:
                pass
            statisticalSocket.close()


class SSCorrValueThread(QThread):
    def __int__(self):
        super(SSCorrValueThread, self).__init__()

    def run(self):
        global SSCorrStop
        SSCorrStop = True
        global SSCorrValueQueue
        SSCorrValueQueue = queue.Queue(0)
        while True:
            try:
                if SSCorrStop == True:
                    SSCorrValueQueue.queue.clear()
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ipaddr, portCorrValue)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                if SSCorrStop == False:
                    SSCorrValueQueue.put(data)
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
        self.SSConfig = []
        self.IdataIn = []
        self.QdataIn = []
        self.IdataOut = []
        self.QdataOut = []
        self.QdataPilot = []
        self.QdataSpectrum = []
        self.IdataSpectrum = []
        self.SNRDataArray = []
        self.CorrValue01 = []
        self.CorrValue02 = []
        self.pathValue = []
        self.TXConfigBytes = bytes()
        self.RXConfigBytes = bytes()
        self.LDPCConfigBytes = bytes()
        self.SSConfigBytes = bytes()
        self.ipBB = '192.168.1.84'
        self.portBB = 8000
        self.ipRF = '192.168.1.38'
        self.portRF = 6005
        # self.ipData = '192.168.1.84'
        # self.portData = 8002
        self.ipLDPC = '192.168.1.90'
        self.portLDPC = 8000
        self.ipSS = '192.168.1.93'
        self.portSS = 8003
        self.timer = QTimer()
        # self.dataTimer = QTimer()
        self.LDPCtimer = QTimer()
        self.spectrumTimer = QTimer()
        self.IQtimer = QTimer()
        self.SStimer01 = QTimer()
        self.SStimer02 = QTimer()
        self.RFFrequencyReceived = 1.6
        self.PrepareIQinCanvas()
        self.PrepareIQOutCanvas()
        self.PreparePilotLineCanvas()
        self.PrepareSpectrumLineCanvas()
        self.PrepareCorrValue01Canvas()
        self.PrepareCorrValue02Canvas()
        self.PrepareSpectrumIdata()
        self.PrepareSNRLineCanvas()
        self.statisticalPort_obj.setText(str(portStatistical))
        self.LDPCstatisticalPort_obj.setText(str(LDPCportStatistical))
        self.spectrumPort_obj.setText(str(SpectrumPortStatistical))
        self.IQPort_obj.setText(str(portIQ))
        self.Reserv_6_obj.setVisible(False)
        self.Reserv_6_label.setVisible(False)
        self.Reserv_I0_obj.setVisible(False)
        self.Reserv_I0_label.setVisible(False)
        self.Reserv_I1_obj.setVisible(False)
        self.Reserv_I1_label.setVisible(False)
        self.Reserv_I2_obj.setVisible(False)
        self.Reserv_I2_label.setVisible(False)
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
        self.checkFalg = False
        self.errbitLDPCArray = []
        self.tolfrmLDPCArray = []

    def bindSingalandSlot(self):
        self.sendBBConfig.clicked.connect(self.sendBBandLDPCConfig)
        self.setBBDefault.clicked.connect(self.setBBandLDPCConfig)
        self.setRFDefault.clicked.connect(self.setRFDefaultConfig)
        self.sendSpreadConfig.clicked.connect(self.sendSSConfig)
        self.setSpreadDefault.clicked.connect(self.setSSDefaultConfig)
        self.start.clicked.connect(self.statisticalTimer)
        self.start.clicked.connect(self.startIQTimer)
        self.stop.clicked.connect(self.killStatisticalTimer)
        self.stop.clicked.connect(self.killIQTimer)
        self.start_LDPC.clicked.connect(self.LDPCStatisticalTimer)
        self.stop_LDPC.clicked.connect(self.killLDPCStatisticalTimer)
        self.start_spectrum.clicked.connect(self.startSpectrumTimer)
        self.stop_spectrum.clicked.connect(self.killSpectrumTimer)
        self.startSysData.clicked.connect(self.startSSTimer)
        self.stopSysData.clicked.connect(self.killSSTimer)
        self.statisticalPort_obj.editingFinished.connect(self.setStatisticalPort)
        self.LDPCstatisticalPort_obj.editingFinished.connect(self.setLDPCStatisticalPort)
        self.spectrumPort_obj.editingFinished.connect(self.setSpectrumStatisticalPort)
        self.IQPort_obj.editingFinished.connect(self.setIQPort)
        self.SysPort_obj.editingFinished.connect(self.setSSSysPort)
        self.CorrValuePort_obj.editingFinished.connect(self.setSSCorrValuePort)
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
        self.selfCheck.clicked.connect(self.startCheckConnect)
        self.stopCheck.clicked.connect(self.stopCheckConnect)
        self.setTDDDefault.clicked.connect(self.setTDDDefaultConfig)
        self.setFDDDefault.clicked.connect(self.setFDDDefaultConfig)
        self.setUDPDefault.clicked.connect(self.setUDPDefaultConfig)
        self.setAUTODefault.clicked.connect(self.setAUTODefaultConfig)

    def closeEvent(self, event):
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

    def calculateOFDMSymbleNum(self, car_num, mtype, adjust = 4):
        MTypeToBit = [2, 4, 6, 8]
        car = self.calculateValidBitNum(car_num)
        m_len = car * 64 * MTypeToBit[mtype] / 8 - adjust
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
        data_len = self.calculateOFDMSymbleNum(self.car_num_rx_obj.value(), self.MType_rx_obj.currentIndex(), 0)
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
        Reserv_U0 = 1 if self.Reserv_U0_obj.isChecked() else 0
        self.RXConfig.extend((Reserv_U0).to_bytes(1, byteorder='big'))
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
        Reserv_5 = 1 if self.Reserv_5_obj.isChecked() else 0
        self.LDPCConfig.extend((Reserv_5).to_bytes(4, byteorder='big'))
        self.LDPCConfig.extend((self.ModeBSorMS_obj.currentIndex()).to_bytes(4, byteorder='big'))
        self.LDPCConfigBytes = bytes(self.LDPCConfig)

    def connectSSConfig(self):
        self.SSConfig.clear()
        flag = 0
        self.SSConfig.extend(int(flag).to_bytes(1, byteorder='big'))
        self.SSConfig.extend(int(self.Source_select_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.SSConfig.extend(int(self.ZC_select_obj.currentIndex()).to_bytes(1, byteorder='big'))
        Rst = 1 if self.RST_obj.isChecked() else 0
        self.SSConfig.extend(int(Rst).to_bytes(1, byteorder='big'))
        self.SSConfig.extend(int(self.ZC32_T_obj.value()).to_bytes(3, byteorder='big'))
        self.SSConfig.extend(int(0).to_bytes(4, byteorder='big'))
        self.SSConfig.extend(int(self.Statis_num_obj.value()).to_bytes(2, byteorder='big'))
        self.SSConfig.extend(int(0).to_bytes(4, byteorder='big'))
        self.SSConfig.extend(int(self.ZC128_T_obj.value()).to_bytes(3, byteorder='big'))
        self.SSConfig.extend(int(0).to_bytes(3, byteorder='big'))
        self.SSConfig.extend(int(self.ZC32_Value_obj.value()).to_bytes(4, byteorder='big'))
        self.SSConfig.extend(int(self.ZC128_Value_obj.value()).to_bytes(4, byteorder='big'))
        SSIp = list(map(int, self.BBIP_obj.text().split(".")))
        self.SSConfig = self.SSConfig + SSIp
        localIp = list(map(int, ipaddr.split(".")))
        self.SSConfig = self.SSConfig + localIp
        self.SSConfig.extend(int(0).to_bytes(28, byteorder='big'))
        self.SSConfigBytes = bytes(self.SSConfig)

    def setSSDefaultConfig(self):
        self.Source_select_obj.setCurrentIndex(0)
        self.ZC_select_obj.setCurrentIndex(0)
        self.RST_obj.setChecked(False)
        self.ZC32_T_obj.setValue(83333)
        self.Statis_num_obj.setValue(1000)
        self.ZC128_T_obj.setValue(625000)
        self.ZC32_Value_obj.setValue(24575)
        self.ZC128_Value_obj.setValue(24575)

        # self.BBIP_obj.setText('192.168.1.93')
        # self.BBPort_obj.setText('8003')

    def setBBDefaultConfig(self):
        # 设置发端默认参数
        self.papr_en_obj.setChecked(True)
        self.depapr_thr_obj.setValue(38050)
        self.depapr_gain_obj.setValue(1.0)
        self.MType_tx_obj.setCurrentIndex(0)
        self.car_num_tx_obj.setValue(0xfffff)
        self.Alpha_tx_obj.setValue(0.5)
        self.pilot_factor_obj.setValue(11467)
        self.pss_factor_obj.setValue(11467)
        self.ModeBSorMS_obj.setCurrentIndex(1)
        self.Sys_reset_obj.setChecked(False)
        self.Sys_enble_obj.setChecked(True)
        self.ModeUDPorPN_obj.setCurrentIndex(1)
        self.Base_loop_en_obj.setCurrentIndex(0)
        self.Rx_enb_obj.setCurrentIndex(0)
        self.Rx_channel_obj.setCurrentIndex(0)
        self.Rx_delay_obj.setValue(20)
        self.Time_trig2tx_obj.setValue(33300)
        self.Time_tx_hold_obj.setValue(40000)
        self.ms_T_sync2trig_obj.setValue(3880)
        self.bs_tdd_time_gap_obj.setValue(80000)
        self.bs_tx_time_gap_obj.setValue(40000)
        self.Trig_gap_cnt_obj.setValue(80000)
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
        self.scale_equa_obj.setValue(1)
        self.scale_fft_obj.setValue(0)
        self.phase_en_obj.setCurrentIndex(1)
        self.phase_factor_obj.setValue(2867)
        self.freq_offset_en_obj.setCurrentIndex(1)
        self.LDPC_en_obj.setCurrentIndex(1)
        self.MMSEorLS_obj.setCurrentIndex(0)
        self.as_time_trig2tx_obj.setValue(17339)
        self.as_trig2tx_cnt_obj.setValue(58735)

        # self.BBIP_obj.setText('192.168.1.84')
        # self.BBPort_obj.setText('8000')

    def setRFDefaultConfig(self):
        # 设置射频参数
        self.Amplifier_obj.setCurrentIndex(0)
        self.filterOption.setCurrentIndex(0)
        self.FPGA_option_obj.setCurrentIndex(0)
        self.FMC_option_obj.setCurrentIndex(0)
        self.RF_send_freq_obj.setText('1')
        self.RF_receive_freq_obj.setText('1')
        self.RF_receive_band_obj.setText('2.0')
        self.RF_TX1_channel_obj.setValue(10.0)
        self.RF_TX2_channel_obj.setValue(10.0)
        self.RF_RX_AGC.setChecked(False)
        self.RF_RX1_gain_obj.setValue(0.0)
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
        self.LDPC_loop_obj.setCurrentIndex(1)
        self.Pause_obj.setCurrentIndex(0)
        self.LDPC_reset_obj.setChecked(False)
        self.LDPC_UDPorPN_obj.setCurrentIndex(1)
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
        # time.sleep(1)
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

    def sendSSConfig(self):
        self.ipSS = self.BBIP_obj.text()
        try:
            self.portSS = int(self.BBPort_obj.text())
        except ValueError:
            print("扩频端口配置无效")
            return
        addrSS= (self.ipSS, self.portSS)
        configSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.connectSSConfig()
        print("扩频配置：", self.SSConfig)
        configSocket.sendto(self.SSConfigBytes, addrSS)
        configSocket.close()
        return

    '''def setRFConfig(self):
        self.setRFDefaultConfig()
        self.RFSendFreqChanged()
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

    def setBBandLDPCConfig(self):
        self.setBBDefaultConfig()
        self.setLDPCDefaultConfig()

    def setTDDDefaultConfig(self):
        self.setBBDefaultConfig()
        self.setLDPCDefaultConfig()
        self.TDD_EN_obj.setCurrentIndex(1)
        self.tx1_en_obj.setCurrentIndex(1)

    def setFDDDefaultConfig(self):
        self.setBBDefaultConfig()
        self.setLDPCDefaultConfig()
        self.TDD_EN_obj.setCurrentIndex(0)
        self.tx1_en_obj.setCurrentIndex(1)
        self.bs_tdd_time_gap_obj.setValue(40000)
        self.bs_tx_time_gap_obj.setValue(40000)
        self.Trig_gap_cnt_obj.setValue(40000)

    def setUDPDefaultConfig(self):
        self.setBBDefaultConfig()
        self.setLDPCDefaultConfig()
        self.tx1_en_obj.setCurrentIndex(1)
        self.bs_tdd_time_gap_obj.setValue(40000)
        self.bs_tx_time_gap_obj.setValue(40000)
        self.Trig_gap_cnt_obj.setValue(40000)
        self.LDPC_UDPorPN_obj.setCurrentIndex(0)
        self.FFTManual_obj.setCurrentIndex(1)
        self.Reserv_5_obj.setChecked(True)

    def setAUTODefaultConfig(self):
        self.setBBDefaultConfig()
        self.setLDPCDefaultConfig()
        self.TDD_EN_obj.setCurrentIndex(1)
        self.tx1_en_obj.setCurrentIndex(1)
        self.bs_tdd_time_gap_obj.setValue(600000)
        self.bs_tx_time_gap_obj.setValue(300000)
        self.Trig_gap_cnt_obj.setValue(600000)
        self.ms_T_sync2trig_obj.setValue(100000)
        self.Reserv_B1_obj.setChecked(True)

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
            self.RF_receive_freq_obj.setText('1.6')

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

    def showSuitableCarrier(self, Mtype, SNRarray):
        threshold = [8, 13, 22, 30][Mtype]
        base = 0
        pointer = len(SNRarray)
        for snr in SNRarray:
            pointer -= 1
            if snr >= threshold:
                base += 2**(pointer)
        return format(base, 'x').zfill(5)

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
        if (singal_pwr_sum == 0):
            SNR = 0
        else:
            if (noise_pwr_sum == 0):
                SNR = 50
            else:
                if self.MType_rx_obj.currentIndex() == 3:
                    SNR = (10 * math.log10(singal_pwr_sum * math.pow(2, 10) / noise_pwr_sum))
                else:
                    SNR = (10 * math.log10(singal_pwr_sum * math.pow(2, 6) / noise_pwr_sum))
        self.SNR_current_show.setText('{:.1f}'.format(SNR))
        # self.count_dot(SNR)
        # self.PrepareSNRSamples(SNR)
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
        sync_pwr = int.from_bytes(data[138:140], byteorder='big')
        self.sync_pwr_show.setText(str(sync_pwr))
        if self.TDD_EN_obj.currentIndex() == 1:
            R2_16b = int.from_bytes(data[142:144], byteorder='big')
            Path = (R2_16b - [42535.8, 42822.8, 42535.8][self.ModeBSorMS_obj.currentIndex()]) * 6
            self.pathValue.append(Path)
            if len(self.pathValue) >= 10:
                pathValueAverage = sum(self.pathValue)/len(self.pathValue)
                self.distance_show.setText(str(pathValueAverage)+'m')
                if pathValueAverage <= 0 or float(self.RF_receive_freq_obj.text()) <= 0:
                    PathLoss = 32.5
                    # self.statusbar.showMessage('实际路径小于0，或者射频接收频率设置错误！！')
                else:
                    PathLoss = 32.5+20*math.log10(Path*10**-3)+20*math.log10(float(self.RF_receive_freq_obj.text())*10**-3)
                self.pathLoss_show.setText(str(PathLoss) + 'dB')
                self.pathValue.clear()
        else:
            self.distance_show.setText('无效值')
            self.pathLoss_show.setText('无效值')
        if tolfrm == 0:
            BER = 'Invalid'
            self.BER_show.setText(BER)
        else:
            M2bit = [2, 4, 6, 8]
            M = M2bit[self.MType_rx_obj.currentIndex()]
            BER = errbit/(((self.calculateValidBitNum(self.car_num_rx_obj.value())*64*M-32)*12)*tolfrm)
            self.BER_show.setText('{:.2e}'.format(BER))
        SNRSub = []
        for i in range(20):
            sigPwr = int.from_bytes(data[(42+i*2):(44+i*2)], byteorder='big')
            nosPwr = int.from_bytes(data[(90+i*2):(92+i*2)], byteorder='big')
            if (sigPwr == 0):
                SNRSub.append(0)
            else:
                if (nosPwr == 0):
                    SNRSub.append(50)
                else:
                    if self.MType_rx_obj.currentIndex() == 3:
                        SNRSub.append(10 * math.log10(sigPwr * math.pow(2, 10) / nosPwr))
                    else:
                        SNRSub.append(10 * math.log10(sigPwr * math.pow(2, 6) / nosPwr))
        self.SuitableCarrier_show.setText(self.showSuitableCarrier(self.MType_rx_obj.currentIndex(), SNRSub))
        self.PrepareSNRSamples(SNRSub)

    def showLDPCStatistical(self, data):
        errbit = int.from_bytes(data[0:4], byteorder='big')
        self.errbit_show_2.setText(str(errbit))
        tolfrm = int.from_bytes(data[4:8], byteorder='big')
        self.tolfrm_show_2.setText(str(tolfrm))
        if self.checkFalg == True:
            self.statusbar.showMessage('自检开始')
            self.errbitLDPCArray.append(errbit)
            self.tolfrmLDPCArray.append(tolfrm)
            if len(self.errbitLDPCArray) == 5 and len(self.tolfrmLDPCArray) == 5:
                if len(set(self.tolfrmLDPCArray)) > 1 and len(set(self.errbitLDPCArray)) == 1 and self.errbitLDPCArray[0] == 0:
                    self.selfCheck.setStyleSheet("background-color: rgb(85, 255, 0)")
                    self.selfCheck.setText('自检成功')
                    self.checkFalg = False
                else:
                    self.selfCheck.setStyleSheet("background-color: rgb(255, 0, 0)")
                    self.selfCheck.setText('自检失败')
                self.tolfrmLDPCArray.clear()
                self.errbitLDPCArray.clear()
        else:
            self.tolfrmLDPCArray.clear()
            self.errbitLDPCArray.clear()
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

    def showSSSysdata(self, data):
        Frame_Num32 = int.from_bytes(data[8:10], byteorder='big')
        self.Frame_num32_obj.setText(str(Frame_Num32))
        Frame_Num128 = int.from_bytes(data[10:12], byteorder='big')
        self.Frame_num128_obj.setText(str(Frame_Num128))
        Biterr_num32 = int.from_bytes(data[19:22], byteorder='big')
        self.Biterr_num32_obj.setText(str(Biterr_num32))
        Biterr_num128 = int.from_bytes(data[22:25], byteorder='big')
        self.Biterr_num128_obj.setText(str(Biterr_num128))
        if Frame_Num32 != 0:
            Biterr_rate32 = Biterr_num32/(Frame_Num32 * 239 * 8)
            self.Biterr_rate32_obj.setText('{:.2e}'.format(Biterr_rate32))
        else:
            self.Biterr_rate32_obj.setText("无效")

        if Frame_Num128 != 0:
            Biterr_rate128 = Biterr_num128/(Frame_Num128 * 239 * 8)
            self.Biterr_rate128_obj.setText('{:.2e}'.format(Biterr_rate128))
        else:
            self.Biterr_rate128_obj.setText("无效")

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

    def PrepareCorrValue01Canvas(self):
        self.CorrValue01LineFigure = Figure_Canvas()
        self.CorrValue01LineLayout = QGridLayout(self.Corr_Value_ZC1)
        self.CorrValue01LineLayout.addWidget(self.CorrValue01LineFigure)

    def PrepareCorrValue02Canvas(self):
        self.CorrValue02LineFigure = Figure_Canvas()
        self.CorrValue02LineLayout = QGridLayout(self.Corr_Value_ZC2)
        self.CorrValue02LineLayout.addWidget(self.CorrValue02LineFigure)

    def PrepareSNRLineCanvas(self):
        self.SNRLineFigure = Figure_Canvas()
        self.SNRLineFigureLayout = QGridLayout(self.subCarrierSNR)
        self.SNRLineFigureLayout.addWidget(self.SNRLineFigure)

    def PrepareSNRSamples(self, value):
        self.SNRLineFigure.ax.cla()
        self.SNRLineFigure.ax.set_autoscale_on(True)
        #self.SNRLineFigure.ax.set_xlim(0, 100)
        #self.SNRLineFigure.ax.set_ylim(0, 50)
        #if len(self.SNRDataArray) >= 100:
        #    self.SNRDataArray = self.SNRDataArray[-100:]
        #self.SNRDataArray.append(value)
        # SNRLine = Line2D(list(range(len(self.SNRDataArray))), self.SNRDataArray)
        # self.SNRLineFigure.ax.add_line(SNRLine)
        self.SNRLineFigure.ax.plot(value)
        self.SNRLineFigure.draw()
        self.SNRLineFigure.flush_events()

    def getIQdata(self, sourceData, Qdata, Idata=[], targetLen=1280, dataType='pilot'):
        for i in range(int(len(sourceData)/4)):
            if dataType == 'corrValue01' or dataType == 'corrValue02':
                Qdata.append(int.from_bytes(sourceData[0:4], byteorder='big'))
            else:
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
        elif flag == 10:
            self.showSSSysdata(Statistical)
        elif flag == 14:
            if len(self.CorrValue01) == 1280:
                self.CorrValue01LineFigure.ax.cla()
                self.CorrValue01LineFigure.ax.set_autoscale_on(True)
                self.CorrValue01LineFigure.ax.plot(self.CorrValue01)
                threshold = [self.ZC32_Value_obj.value(), self.ZC128_Value_obj.value()]
                self.CorrValue01LineFigure.ax.plot([threshold[self.ZC_select_obj.currentIndex()]]*1280)
                self.CorrValue01LineFigure.draw()
                self.CorrValue01LineFigure.flush_events()
                self.CorrValue01.clear()
            else:
                self.CorrValue01.clear()
            self.getIQdata(Statistical, self.CorrValue01, [], 1280, 'corrValue01')
        elif flag == 18:
            self.getIQdata(Statistical, self.CorrValue01, [], 1280, 'corrValue01')
            self.CorrValue01LineFigure.ax.cla()
            self.CorrValue01LineFigure.flush_events()
        elif flag == 15:
            if len(self.CorrValue02) == 1280:
                self.CorrValue02LineFigure.ax.cla()
                self.CorrValue02LineFigure.ax.set_autoscale_on(True)
                self.CorrValue02LineFigure.ax.plot(self.CorrValue02)
                threshold = [self.ZC32_Value_obj.value(), self.ZC128_Value_obj.value()]
                self.CorrValue02LineFigure.ax.plot([threshold[self.ZC_select_obj.currentIndex()]]*1280)
                self.CorrValue02LineFigure.draw()
                self.CorrValue02LineFigure.flush_events()
                self.CorrValue02.clear()
            else:
                self.CorrValue02.clear()
            self.getIQdata(Statistical, self.CorrValue02, [], 1280, 'corrValue02')
        elif flag == 17:
            self.getIQdata(Statistical, self.CorrValue02, [], 1280, 'corrValue02')
            self.CorrValue02LineFigure.ax.cla()
            self.CorrValue02LineFigure.flush_events()
        else:
            print("无效的统计数据")
            print(flag, Statistical)

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

    def updateSSSysStatistical(self):
        if not SSSysQueue.empty():
            # print("更新扩频系统数据")
            self.showStatistical(SSSysQueue.get())

    def updateSSCorrValueStatistical(self):
        if not SSCorrValueQueue.empty():
            # print("更新扩频相关值")
            self.showStatistical(SSCorrValueQueue.get())

    def statisticalTimer(self):
        global BBstop
        BBstop = False
        self.timer.start(0)
        print('调制解调统计数据开始更新')
        self.timer.timeout.connect(self.updateStatistical)

    def killStatisticalTimer(self):
        global BBstop
        BBstop = True
        print('停止显示调制解调统计数据')
        self.timer.stop()

    def LDPCStatisticalTimer(self):
        global LDPCstop
        LDPCstop = False
        self.LDPCtimer.start(1)
        print('LDPC统计数据开始更新')
        self.LDPCtimer.timeout.connect(self.updateLDPCStatistical)

    def killLDPCStatisticalTimer(self):
        global LDPCstop
        LDPCstop = True
        print('停止显示LDPC统计数据')
        self.LDPCtimer.stop()

    def startSpectrumTimer(self):
        global SpectrumStop
        SpectrumStop = False
        self.spectrumTimer.start(1)
        print("频谱检测开始更新")
        self.spectrumTimer.timeout.connect(self.updateSpectrumStatistical)

    def killSpectrumTimer(self):
        global SpectrumStop
        SpectrumStop = True
        print("停止显示频谱检测")
        self.spectrumTimer.stop()

    def startIQTimer(self):
        global IQstop
        IQstop = False
        self.IQtimer.start(1)
        print("IQ显示开始更新")
        self.IQtimer.timeout.connect(self.updateIQStatistical)

    def killIQTimer(self):
        global IQstop
        IQstop = True
        print("停止IQ显示更新")
        self.IQtimer.stop()

    def startSSTimer(self):
        global SSSysStop, SSCorrStop
        SSSysStop, SSCorrStop = False, False
        print("扩频数据显示开始更新")
        self.SStimer01.start(1)
        self.SStimer02.start(1)
        self.SStimer01.timeout.connect(self.updateSSSysStatistical)
        self.SStimer02.timeout.connect(self.updateSSCorrValueStatistical)

    def killSSTimer(self):
        global SSSysStop, SSCorrStop
        SSSysStop, SSCorrStop = True, True
        print("停止显示扩频数据")
        self.SStimer01.stop()
        self.SStimer02.stop()

    def startCheckConnect(self):
        configCheck = [['1', '2', 10, 0, 1, 0, 1, 1, 0], ['2', '1', 10, 0, 1, 1, 1, 1, 0]]
        configSend = configCheck[self.ModeBSorMS_obj.currentIndex()]
        self.setBBandLDPCConfig()
        self.setRFDefaultConfig()
        self.RF_send_freq_obj.setText(configSend[0])
        self.RF_receive_freq_obj.setText(configSend[1])
        self.RF_TX1_channel_obj.setValue(configSend[2])
        self.RF_RX1_gain_obj.setValue(configSend[3])
        self.source_Clock_obj.setCurrentIndex(configSend[4])
        self.ModeBSorMS_obj.setCurrentIndex(configSend[5])
        self.TDD_EN_obj.setCurrentIndex(configSend[6])
        self.tx1_en_obj.setCurrentIndex(configSend[7])
        self.Base_loop_en_obj.setCurrentIndex(configSend[8])
        self.RFSendFreqChanged()
        self.RFReceiveFreqChanged()
        self.RFTXChannelChanged()
        self.RFRXGainChanged()
        self.RFClockChanged()
        self.RFCalibrationStatusChanged()
        self.sendBBandLDPCConfig()
        self.checkFalg = True

    def stopCheckConnect(self):
        self.checkFalg = False
        self.selfCheck.setStyleSheet("background-color: rgb(220, 220, 220)")
        self.selfCheck.setText('开始自检')
        self.statusbar.showMessage('停止自检')

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

    def setSSSysPort(self):
        global portSSSys
        try:
            port = int(self.SysPort_obj.text())
            if 1024 <= port < 65535:
                portSSSys = port
            else:
                if (self.SysPort_obj.hasFocus()):
                    QMessageBox.information(self, "Tips", "1024 <= SysPort < 65535")
        except ValueError:
            print("扩频系统端口配置无效")

    def setSSCorrValuePort(self):
        global portCorrValue
        try:
            port = int(self.CorrValuePort_obj.text())
            if 1024 <= port < 65535:
                portCorrValue = port
            else:
                if (self.CorrValuePort_obj.hasFocus()):
                    QMessageBox.information(self, "Tips", "1024 <= CorrValuePort < 65535")
        except ValueError:
            print("扩频相关数端口配置无效")

if __name__ == "__main__":
    portStatistical, LDPCportStatistical, SpectrumPortStatistical, portIQ, portSSSys, portCorrValue = getStatisticalPort()
    ipaddr = getChosenIP('1')
    statisticThread = StatisticThread()
    statisticThread.start()
    statisticThreadLDPC = LDPCStatisticThread()
    statisticThreadLDPC.start()
    spectrumThread = SpectrumStatisticThread()
    spectrumThread.start()
    IQdataThread = IQThread()
    IQdataThread.start()
    SSSystemThread = SSSysThread()
    SSSystemThread.start()
    CorrValueThread = SSCorrValueThread()
    CorrValueThread.start()
    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create('Windows'))
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())
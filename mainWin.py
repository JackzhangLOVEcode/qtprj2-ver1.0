# -*- coding:utf-8 -*-
import sys, math, socket, time, queue, struct, encodings.idna, stopThreading
from threading import Thread
from array import array
from re import match
from os.path import exists
from mainWinUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QGridLayout
from PyQt5.QtCore import QThread, QTimer, QPoint, Qt, pyqtSignal
from PyQt5.QtGui import QIcon, QPainter, QPixmap, QPen, QColor, QIntValidator, QDoubleValidator
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

fpgalist = ['FPGA1','FPGA2','FPGA3','FPGA4']
specfile = 'specline.txt'
specbin = 'specbin.bin'
specip = 'specip.txt'
txdata = bytearray(20)
bufaddr = bytearray(16)
bufdata = bytearray(16*4)
rxdata = bytearray(20)
lindex = 0
txindex = 0
addrhead = array('b',[0x00,0x01,0x02,0x03,0x04,0x10,0x11,0x12,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E])
ip1 = 192
ip2 = 168
ip3 = 0
ip4 = 104
PORT = 6005
Uport = 8888

Dport = 9999
Iport = 6005
framedatalen = 235
zerodata = bytearray([1 for i in range(256)])

def bytesToFloat(h1, h2, h3, h4):
    ba = bytearray()
    ba.append(h1)
    ba.append(h2)
    ba.append(h3)
    ba.append(h4)
    return struct.unpack("!f", ba)[0]

class StatisticThread(QThread):
    def __int__(self):
        super(StatisticThread, self).__init__()

    def run(self):
        global dataStatistical
        dataStatistical = bytes([0])
        global portStatistical
        portStatistical = 7000
        ipfile = open(specip, 'r', encoding='utf-8')
        str1 = ipfile.readline().strip()
        str2 = ipfile.readline().strip()
        pattern = '[1-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'
        if match(pattern, str1) and match(pattern, str2) :
            ulocalip = str2
        else:
            hostname = socket.gethostname()
            ulocalip = socket.gethostbyname(hostname)
        while True:
            try:
                statisticalSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                addr = (ulocalip, portStatistical)
                buffsize = 1500
                statisticalSocket.bind(addr)
                statisticalSocket.settimeout(1)
                #print('receive data ......')
                data, addrsource = statisticalSocket.recvfrom(buffsize)
                #print(data)
                dataStatistical = data
                # print(dataStatistical)
            except socket.timeout:
                statisticalSocket.close()
                break
            '''except OSError:
                print("本地IP配置与IP配置文件中的IP不一致！！！")
                break'''

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
        socketSource.close()

'''class showConstellation():
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
        self.draw.end()'''

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
        self.timer = QTimer()
        self.dataTimer = QTimer()
        self.bindSingalandSlot()
        # 以下程序同步于哈工大王老师的代码
        self.up_udp_socket = None
        self.down_udp_socket = None
        self.down_sever_th = None
        self.up_sever_th = None
        self.info_sever_th = None  ## 用于应答，暂时没用
        self.num = 0
        self.tnum = 0
        self.delayworking = False
        self.serialnum = 0
        self.rxbuffer = bytearray([0 for i in range(framedatalen)]) * 256
        self.started = False  ##通信机数据开始接收
        self.confirm = [False] * 256  # 用于收数据段的确认
        self.lastlen = 0  # 用于适配物理层网络口的数据大小，补零操作
        self.frameno = 0  # 用于记录数据段的序号
        self.framenum = 0  # 用于记录UDP数据包切成数据段的总数
        self.frameserialno = 0  # 用于记录UDP包的序号

        if self.ConfigFileExist(specfile):
            self.ReadConfigFile()
        else:
            self.setDefaultPara()

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        for litem in fpgalist:
            self.comboBox.addItem(litem)
        self.comboBox_2.addItem('config模式')
        self.comboBox_2.addItem('FPGA工作模式')
        self.comboBox_2.addItem('侦测频点')

        self.comboBox_3.addItem('外供时钟')
        self.comboBox_3.addItem('TCOX')
        self.comboBox_3.addItem('OCOX')

        self.comboBox_4.addItem('本振源ORX1')
        self.comboBox_4.addItem('本振源ORX2')
        self.comboBox_4.addItem('本振源ORX3')

        self.DisplayConfigPara()
        self.sendtobuffer()
        self._signal.connect(self.txsignalslot)

        self.pushButton.clicked.connect(self.exitproc)                  # app exit

        self.lineEdit.editingFinished.connect(self.ipaddrchanged)       # ip changed
        self.lineEdit_14.editingFinished.connect(self.uipaddrchanged)       # ip changed

        self.lineEdit_2.editingFinished.connect(self.rxFreqChanged)     #接收频率改变
        self.lineEdit_3.editingFinished.connect(self.txFreqChanged)     #发射频率改变

        self.checkBox_5.stateChanged.connect(self.TRstateChanged)       #收发使能状态改变
        self.checkBox_6.stateChanged.connect(self.TRstateChanged)       #同上
        self.checkBox_7.stateChanged.connect(self.TRstateChanged)       #同上
        self.checkBox_8.stateChanged.connect(self.TRstateChanged)       #同上
        self.comboBox.currentIndexChanged.connect(self.FpgaSelectionChanged)    #Fpag选择改变 new function
        self.comboBox_2.currentIndexChanged.connect(self.SpecModeChanged)       #配置模式改变
        self.comboBox_4.currentIndexChanged.connect(self.DetChannelChanged)     #观察通道改变
        self.comboBox_3.currentIndexChanged.connect(self.ClockChanged)          #时钟改变

        self.checkBox_10.stateChanged.connect(self.RxsuppstateChanged)
        self.checkBox_9.stateChanged.connect(self.TxsuppstateChanged)
        self.checkBox_11.stateChanged.connect(self.firstCRCchanged)

        self.checkBox_2.stateChanged.connect(self.AGC_GhangedProc)          #AGC使能关联函数
        self.lineEdit_6.editingFinished.connect(self.AGC_GhangedProc)       #同上
        self.lineEdit_8.editingFinished.connect(self.AGC_GhangedProc)       #同上
        self.checkBox_4.stateChanged.connect(self.ORXAGC_GhangedProc)       #ORXAGC使能关联函数
        self.lineEdit_10.editingFinished.connect(self.ORXAGC_GhangedProc)  # 同上
        self.lineEdit_12.editingFinished.connect(self.ORXAGC_GhangedProc)  # 同上
        self.pushButton_4.clicked.connect(self.servLink)
        self.pushButton_5.clicked.connect(self.clearData)
        self.lineEdit_15.editingFinished.connect(self.cportchanged)
        pFrqIncValidator = QIntValidator()
        pFrqIncValidator.setRange(0, 150)

        pdetFreqValidator = QIntValidator()
        pdetFreqValidator.setRange(0, 50)

        self.lineEdit_4.setValidator(pdetFreqValidator)

        self.lineEdit_6.setValidator(pFrqIncValidator)
        self.lineEdit_7.setValidator(pFrqIncValidator)
        self.lineEdit_8.setValidator(pFrqIncValidator)
        self.lineEdit_9.setValidator(pFrqIncValidator)
        self.lineEdit_10.setValidator(pFrqIncValidator)
        self.lineEdit_11.setValidator(pFrqIncValidator)
        self.lineEdit_12.setValidator(pFrqIncValidator)

        pDoubleValidator = QDoubleValidator()
        pDoubleValidator.setRange(-100, 100)
        pDoubleValidator.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator.setDecimals(4)

        self.lineEdit_2.setValidator(pDoubleValidator)
        self.lineEdit_3.setValidator(pDoubleValidator)

        ipstr = socket.gethostbyname(socket.gethostname())
        print(ipstr)

        if self.ConfigFileExist(specip):
            if self.ReadipFile():
                #pass
                print('读取网卡地址成功:'+self.ulocalip + '& ' +self.dlocalip)
            else:
                print('按缺省地址启动！')
                self.ulocalip = ipstr
                self.dlocalip = ipstr
        else:
            print('按缺省地址启动！')
            self.ulocalip = ipstr
            self.dlocalip = ipstr
        self.pushButton.setEnabled(False)
        self.pushButton.setVisible(False)

        self.statusbar = self.statusBar()
        self.statusbar.showMessage('Ready!')

        self.ptimer = QTimer()
        self.ptimer.timeout.connect(self.update_connection)
        self.firsttimer = QTimer()
        self.firsttimer.timeout.connect(self.channelconnect)    # 通信机联络,确定通道建立
        self.channelstate = False                               # 记录当前状态

        self.info_udp_server_start()        #信息服务端 server 自动启动接收反馈信息
        self.downsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.upsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.checkBox_11.setVisible(False)
        self.groupBox_3.setVisible(False)
        self.textBrowser_recv.setVisible(False)
        self.pushButton_5.setVisible(False)
        # self.setFixedSize(600,788)
        # 同步完毕第一部分

    def init(self):
        self.TXConfig = []
        self.RXConfig = []
        self.RFConfig = []
        self.LDPCConfig = []
        self.IdataIn = []
        self.QdataIn = []
        self.IdataOut = []
        self.QdataOut = []
        self.IdataPilot = []
        self.QdataPilot = []
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
        # self.draw = QPainter()
        # self.picture = QPixmap(480, 100)
        # self.end_dot_list = []
        # self.x_num = 100
        # self.x_step = 480 / self.x_num
        self.PrepareIQinCanvas()
        self.PrepareIQOutCanvas()
        self.PreparePilotLineCanvas()
        self.PrepareSNRLineCanvas()
        self.Reserv_1_obj.setVisible(False)
        self.Reserv_1_label.setVisible(False)
        self.Reserv_2_obj.setVisible(False)
        self.Reserv_2_label.setVisible(False)
        self.Reserv_3_obj.setVisible(False)
        self.Reserv_3_label.setVisible(False)
        self.Reserv_4_obj.setVisible(False)
        self.Reserv_4_label.setVisible(False)
        self.Reserv_5_obj.setVisible(False)
        self.Reserv_5_label.setVisible(False)
        self.Reserv_6_obj.setVisible(False)
        self.Reserv_6_label.setVisible(False)
        self.Reserv_B0_obj.setVisible(False)
        self.Reserv_B0_label.setVisible(False)
        self.Reserv_B1_obj.setVisible(False)
        self.Reserv_B1_label.setVisible(False)
        self.Reserv_I0_obj.setVisible(False)
        self.Reserv_I0_label.setVisible(False)
        self.Reserv_I1_obj.setVisible(False)
        self.Reserv_I1_label.setVisible(False)
        self.Reserv_I2_obj.setVisible(False)
        self.Reserv_I2_label.setVisible(False)
        self.Reserv_I3_obj.setVisible(False)
        self.Reserv_I3_label.setVisible(False)
        self.Reserv_U0_obj.setVisible(False)
        self.Reserv_U0_label.setVisible(False)
        self.Reserv_U1_obj.setVisible(False)
        self.Reserv_U1_label.setVisible(False)

    def bindSingalandSlot(self):
        self.sendConfig.clicked.connect(self.sendConfigtoBaseBand)
        self.setDefault.clicked.connect(self.setDefaultConfig)
        self.start.clicked.connect(self.statisticalTimer)
        self.stop.clicked.connect(self.killStatisticalTimer)
        self.sendData.clicked.connect(self.startDataTimer)
        self.stopData.clicked.connect(self.stopDataTimerandDataSource)
        self.statisticalPort_obj.editingFinished.connect(self.setStatisticalPort)

    # 以下代码同步于哈工大王老师的代码
    def ConfigFileExist(self, Filename):
        if exists(Filename):
            return True
        else:
            return False

    def ReadipFile(self):
        ipfile = open(specip,'r',encoding='utf-8')
        str1 = ipfile.readline().strip()
        #ipaddr = str.split('.')
        str2 = ipfile.readline().strip()
        pattern = '[1-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'
        if match(pattern,str1) and match(pattern,str2) :
            self.dlocalip = str1
            self.ulocalip = str2
            return  True
        else:
            return False

    def ReadConfigFile(self):
        txfile = open(specfile,'r',encoding='utf-8')
        str = txfile.readline().strip()
        ipaddr = str.split('.')
        self.ip1 = int(ipaddr[0])
        self.ip2 = int(ipaddr[1])
        self.ip3 = int(ipaddr[2])
        self.ip4 = int(ipaddr[3].rstrip())
        self.ipaddr = str
        ip1 = self.ip1
        ip2 = self.ip2
        ip3 = self.ip3
        ip4 = self.ip4

        str = txfile.readline()
        self.rxfreq = float(str.rstrip())
        str = txfile.readline()
        self.txfreq = float(str.rstrip())
        str = txfile.readline()
        self.detfrq = float(str.rstrip())
        str = txfile.readline()
        self.rx1_inc = int(str.rstrip())
        str = txfile.readline()
        self.rx2_inc = int(str.rstrip())
        str = txfile.readline()
        self.tx1_dec = int(str.rstrip())
        str = txfile.readline()
        self.tx2_dec = int(str.rstrip())
        str = txfile.readline()
        self.orx1_inc = int(str.rstrip())
        str = txfile.readline()
        self.orx2_inc = int(str.rstrip())
        str = txfile.readline()
        self.snrx_inc =  int(str.rstrip())
        str = txfile.readline()
        indextemp = str.split(',')
        self.cb_index1 = int(indextemp[0])
        self.fpgasel = self.cb_index1
        self.cb_index2 = int(indextemp[1])
        self.specmode = self.cb_index2
        self.cb_index3 = int(indextemp[2])
        self.detchl = self.cb_index3
        self.cb_index4 = int(indextemp[3].strip())
        self.clocksel = self.cb_index4
        str = txfile.readline()
        booltemp = str.split(',')
        self.agc_bool = bool(int(booltemp[0]))
        self.orxagc_bool = bool(int(booltemp[1]))
        self.rx1_enable = bool(int(booltemp[2]))
        self.rx2_enable = bool(int(booltemp[3]))
        self.tx1_enable = bool(int(booltemp[4]))
        self.tx2_enable = bool(int(booltemp[5]))
        self.rxsupp = bool(int(booltemp[6]))
        self.txsupp = bool(int(booltemp[7]))
        self.firstcrc = bool(int(booltemp[8].rstrip()))

        str = txfile.readline().strip()
        ipaddr = str.split('.')
        self.uip1 = int(ipaddr[0])
        self.uip2 = int(ipaddr[1])
        self.uip3 = int(ipaddr[2])
        self.uip4 = int(ipaddr[3].rstrip())
        self.uipaddr = str.rstrip()

        str = txfile.readline().strip()
        self.cport = int(str)
        txfile.close()
        return

    def setDefaultPara(self):
        self.ipaddr = '127.0.0.1'
        self.uipaddr = '127.0.0.1'
        self.ip1 = 192
        self.ip2 = 168
        self.ip3 = 0
        self.ip4 = 1
        self.uip1 = 192
        self.uip2 = 168
        self.uip3 = 0
        self.uip4 = 2

        ip1 = self.ip1
        ip2 = self.ip2
        ip3 = self.ip3
        ip4 = self.ip4

        self.fpgasel = 3
        self.cb_index1 = self.fpgasel
        self.specmode = 1
        self.cb_index2 = self.specmode
        self.rxfreq = 1.2
        self.txfreq = 1.3
        self.detfrq = 1.4
        self.detchl = 2
        self.cb_index3 = self.detchl

        self.agc_bool = True
        self.rx1_inc = 40
        self.tx1_dec = 100
        self.rx2_inc = 40
        self.tx2_dec = 100

        self.orxagc_bool = True
        self.orx1_inc = 40
        self.orx2_inc = 40
        self.snrx_inc = 40

        self.rx1_enable = True
        self.tx1_enable = True
        self.rx2_enable = False
        self.tx2_enable = False
        self.rxsupp = False
        self.txsupp = False
        self.clocksel = 2
        self.cb_index4 = self.clocksel
        self.crct_enable = False
        self.cport = 5000
        self.firstcrc = False

    def DisplayConfigPara(self):
        self.lineEdit.setText('%s.%s.%s.%s'%(self.ip1,self.ip2,self.ip3,self.ip4))
        self.lineEdit_14.setText('%s.%s.%s.%s'%(self.uip1,self.uip2,self.uip3,self.uip4))

        self.lineEdit_2.setText(str(self.rxfreq))
        self.lineEdit_3.setText(str(self.txfreq))
        self.lineEdit_4.setText(str(self.detfrq))
        self.lineEdit_6.setText(str(self.rx1_inc))
        self.lineEdit_7.setText(str(self.tx1_dec))
        self.lineEdit_8.setText(str(self.rx2_inc))
        self.lineEdit_9.setText(str(self.tx2_dec))
        self.lineEdit_10.setText(str(self.orx1_inc))
        self.lineEdit_11.setText(str(self.snrx_inc))
        self.lineEdit_12.setText(str(self.orx2_inc))


        if self.agc_bool==True:
            self.checkBox_2.setChecked(True)
        else:
            self.checkBox_2.setChecked(False)

        if self.orxagc_bool == True:
            self.checkBox_4.setChecked(True)
        else:
            self.checkBox_4.setChecked(False)

        if self.rx1_enable == True:
            self.checkBox_5.setChecked(True)
        else:
            self.checkBox_5.setChecked(False)

        if self.rx2_enable == True:
            self.checkBox_6.setChecked(True)
        else:
            self.checkBox_6.setChecked(False)

        if self.tx1_enable == True:
            self.checkBox_7.setChecked(True)
        else:
            self.checkBox_7.setChecked(False)

        if self.tx2_enable == True:
            self.checkBox_8.setChecked(True)
        else:
            self.checkBox_8.setChecked(False)

        if self.rxsupp == True:
            self.checkBox_10.setChecked(True)
        else:
            self.checkBox_10.setChecked(False)

        if self.txsupp == True:
            self.checkBox_9.setChecked(True)
        else:
            self.checkBox_9.setChecked(False)
        if self.firstcrc == True:
            self.checkBox_11.setChecked(True)
        else:
            self.checkBox_11.setChecked(False)

        self.lineEdit_4.setEnabled(False)
        self.comboBox.setCurrentIndex(self.cb_index1)
        self.comboBox_2.setEnabled(False)
        self.comboBox_2.setCurrentIndex(self.cb_index2)
        self.comboBox_4.setEnabled(False)
        self.comboBox_4.setCurrentIndex(self.cb_index3)
        self.comboBox_3.setEnabled(False)
        self.comboBox_3.setCurrentIndex(self.cb_index4)
        self.checkBox.setEnabled(False)
        self.lineEdit_15.setText(str(self.cport))

    def sendtobuffer(self):
        for i in range(16):
            bufaddr[i] = addrhead[i]
        for i in range(20):
            txdata[i] = 0x00

    def txsignalslot(self, parameter):
        print(str(parameter))
        print('slot called')
        WR_bit = 1  # maybe changed sometimes
        txdata[0] = WR_bit * (2 ** 7) + 1
        txdata[1] = self.comboBox.currentIndex() + 0x01
        txdata[2] = 0x10
        self.s.sendto(txdata, (self.ipaddr, self.cport))
        print(self.ipaddr + ":8001:")
        for i in range(20):
            print('[%02x]' % txdata[i])
        return

    def exitproc(self):
        sender = self.sender()
        self.s.close()
        self.savetofile()
        self.delayworking = False
        self.up_udp_close()
        self.down_udp_close()
        qApp = QApplication.instance()
        qApp.quit()

    def ipaddrchanged(self):
        print('ip changed')
        ipstr = self.lineEdit.text()
        pattern = '[1-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'
        if match(pattern, ipstr):
            self.ipaddr = ipstr
            partip = ipstr.split('.')
            self.ip1 = int(partip[0])
            self.ip2 = int(partip[1])
            self.ip3 = int(partip[2])
            self.ip4 = int(partip[3])
            ip1 = self.ip1
            ip2 = self.ip2
            ip3 = self.ip3
            ip4 = self.ip4

        else:
            self.lineEdit.setText(self.ipaddr)
            QMessageBox.information(self, ('地址错误!'), ("""地址错误提示"""), \
                                    QMessageBox.StandardButton(QMessageBox.Yes))

    def uipaddrchanged(self):
        print('uip changed')
        ipstr = self.lineEdit_14.text()
        pattern = '[1-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'
        if match(pattern,ipstr) :
            self.uipaddr = ipstr
            partip = ipstr.split('.')
            self.uip1 = int(partip[0])
            self.uip2 = int(partip[1])
            self.uip3 = int(partip[2])
            self.uip4 = int(partip[3])
        else:
            self.lineEdit_14.setText(self.uipaddr)
            QMessageBox.information(self,('地址错误!'),("""地址错误提示"""),\
                                    QMessageBox.StandardButton(QMessageBox.Yes))

    def LongvTochar(self,  freq):
        lgv = int(freq * 10 ** 7)
        midh = int(lgv / (256 * 256))
        midl = int(lgv % (256 * 256))
        bh1 = int(midh / 256)
        bh2 = int(midh % 256)
        bh3 = int(midl / 256)
        bh4 = int(midl % 256)

        txdata[16] = bh1
        txdata[17] = bh2
        txdata[18] = bh3
        txdata[19] = bh4

    def rxFreqChanged(self):
        self.rxfreq = float(str(self.lineEdit_2.text()).rstrip())
        self.LongvTochar(self.rxfreq)
        txdata[3] = 0x11
        self._signal.emit(2)  # 2:接收频率改变信号 proc

    def txFreqChanged(self):
        self.txfreq = float(str(self.lineEdit_3.text().rstrip()))
        self.LongvTochar(self.txfreq)
        txdata[3] = 0x10
        self._signal.emit(3)  # 3:发射频率改变信号 proc

    def TRstateChanged(self):
        self.rx1_enable = self.checkBox_5.isChecked()
        self.rx2_enable = self.checkBox_6.isChecked()
        self.tx1_enable = self.checkBox_7.isChecked()
        self.tx2_enable = self.checkBox_8.isChecked()
        txdata[3] = 0x1b
        txdata[16] = 0x00
        txdata[17] = 0x00
        txdata[18] = 0x00
        txdata[19] = int(self.rx1_enable)*8 + int(self.rx2_enable)*4 + int(self.tx1_enable)*2 + int(self.tx2_enable)

        self._signal.emit(4)    #4:收发使能状态改变信号

    def FpgaSelectionChanged(self):#Fpag选择改变处理函数
        self.cb_index1 = self.comboBox.currentIndex()
        self.fpgasel = self.cb_index1
        return

    def SpecModeChanged(self):       #配置模式改变处理函数
        self.cb_index2 = self.comboBox_2.currentIndex()
        self.specmode = self.cb_index2
        return

    def DetChannelChanged(self):     #观察通道改变处理函数
        self.cb_index3 = self.comboBox_4.currentIndex()
        self.detchl = self.cb_index3
        return

    def ClockChanged(self):          #时钟改变处理函数
        self.cb_index4 = self.comboBox_3.currentIndex()
        self.clocksel = self.cb_index4
        return

    def RxsuppstateChanged(self):
        self.rxsupp = self.checkBox_10.isChecked()
        return

    def TxsuppstateChanged(self):
        self.txsupp = self.checkBox_9.isChecked()
        return

    def firstCRCchanged(self):
        self.firstcrc = self.checkBox_11.isChecked()
        return

    def AGC_GhangedProc(self):
        self.agc_bool = self.checkBox_2.isChecked()
        self.rx1_inc = int(self.lineEdit_6.text().strip())
        self.rx2_inc = int(self.lineEdit_8.text().strip())
        txdata[3] = 0x18
        txdata[16] = 0x00
        txdata[17] = int(self.agc_bool)
        txdata[18] = int(self.rx1_inc * 2)
        txdata[19] = int(self.rx2_inc * 2)

        self._signal.emit(5)  # 5:AGC使能改变发送接收通道增益
        return

    def ORXAGC_GhangedProc(self):
        self.orxagc_bool = self.checkBox_4.isChecked()
        self.orx1_inc = int(self.lineEdit_10.text().strip())
        self.orx2_inc = int(self.lineEdit_12.text().strip())
        txdata[3] = 0x19
        txdata[16] = 0x00
        txdata[17] = int(self.orxagc_bool)
        txdata[18] = int(self.orx1_inc)
        txdata[19] = int(self.orx2_inc)

        self._signal.emit(6)  # 6:ORXAGC使能改变发送接收通道增益
        return

    def up_udp_server_start(self):
        """
        开启上行 UDP服务端方法
        :return:
        """
        self.up_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            port = 8001
            address = (self.ulocalip, port)
            self.up_udp_socket.bind(address)
            msg = '上行 UDP 打开。。。'
            #print(msg)
        except Exception as ret:
            msg = '上行 UDP 没有打开，请检查网络'

        else:
            self.up_sever_th = Thread(target=self.up_udp_server_concurrency)
            self.up_sever_th.start()
            msg = '上行UDP服务端正在监听端口:{}\n'.format(port)
        print(msg)

    def up_udp_server_concurrency(self):
        """
        上行UDP通信线程，转发板卡数据，加上行数据头
        :return:
        """
        buf = bytes.fromhex('7f 7f 7f 7f')
        while True:
            data = self.up_udp_socket.recv(1500)
            framelen = len(data)
            print("通信机 长度："+ str(framelen))
            if framelen < 4:
                continue
            if framelen > 0 and framelen <= 20:
                if data[3] == 0x10:
                    displaymsg = '发射频率设置成功！'

                elif data[3] == 0x11:
                    displaymsg = '接收频率设置成功！'
                elif data[3] == 0x18:
                    displaymsg = '接收增益设置成功！'
                elif data[3] == 0x19:
                    displaymsg = '观察接收增益设置成功！'
                elif data[3] == 0x1a:
                    displaymsg = '侦测接收增益设置成功！'
                elif data[3] == 0x1b:
                    displaymsg = '收发通道使能设置成功！'
                else:
                    displaymsg = '其他数据'
                self.statusbar.showMessage(displaymsg)
                continue
            elif framelen == 239:
                serialnum = data[0]
                fno = data[1]
                fnum = data[2]
                lastlen = data[3]

                if fno == 0:
                    self.frameserialno = serialnum
                    self.frameno = fno
                    self.framenum = fnum
                    self.lastlen = lastlen
                    self.started = True
                if self.started == False:
                    print('not catch frame head!')
                    continue

                if fno != self.frameno or serialnum != self.frameserialno:
                    self.started = False
                    continue
                self.frameno += 1

                self.rxbuffer[framedatalen*fno:framedatalen*fno+239] = bytearray(data[4:])
                self.confirm[fno] = True

                if fno == fnum and serialnum == self.frameserialno:
                    if False not in self.confirm[:fnum+1]:
                        if self.rxbuffer[2] == 0x04:
                            self.rxbuffer[2] = 0x05
                        buftemp = buf + self.rxbuffer[:fnum*framedatalen+self.lastlen]
                        crcdata = self.rxbuffer[3:fnum*framedatalen+self.lastlen-2]
                        wcrcbytes = self.crc16(str(crcdata))
                        self.upsock.sendto(buftemp, (self.uipaddr, Uport))
                        '''if wcrcbytes == int(self.rxbuffer[fnum*framedatalen+self.lastlen-1]+self.rxbuffer[fnum*framedatalen+self.lastlen-2]*256):
                            self.upsock.sendto(buftemp, (self.uipaddr, Uport))
                        else:
                            print('crc error 1')'''
                    else:
                        print('Drop 1 fr!')
                    self.frameno = 0
                    self.framenum = 0
                    self.started = False
                    self.Clearconfirm()

            else:
                print('其它长度数据：'+ str(framelen) )
                self.num = self.num + 1

    def up_udp_close(self):
        """
        功能函数，关闭上行网络连接
        :return:
        """
        try:
            stopThreading.stop_thread(self.up_sever_th)
        except Exception:
            print('关闭upd错')
            pass
        try:
            self.up_udp_socket.close()
            if self.delayworking is False:
                msg = '已断开网络\n'
                print(msg)
                #self.linked = False
        except Exception as ret:
            pass

    def Clearconfirm(self):
        for i in range(256):
            self.confirm[i] =  False

    def crc16(self, buffer):
        c, treat, bcrc, wcrc = 0, 0, 0, 0
        for i in range(len(buffer)):
            c = ord(buffer[i])
            for j in range(8):
                treat = c & 0x80
                c <<= 1
                bcrc = (wcrc >> 8) & 0x80
                wcrc <<= 1
                wcrc %= (0xffff + 1)
                if treat != bcrc:
                    wcrc ^= 0x1021
        return wcrc

    def servLink(self):
        """
        启动上下行数据通信UDP通路，暂时关闭上行ip地址编辑功能，切换按钮名称
        """
        if not self.delayworking:
            self.up_udp_server_start()
            self.down_udp_server_start()
            #self.pushButton_link.setWindowTitle('断开')
            print('Udp opened ')
            self.delayworking = True
            self.pushButton_4.setText('停止')
            self.lineEdit_14.setEnabled(False)
            self.ptimer.start(3000)  ##周期发送给控制机联络信息
        else:
            self.delayworking = False
            self.up_udp_close()
            self.down_udp_close()
            print('Udp closed')
            self.pushButton_4.setText('转发')
            self.lineEdit_14.setEnabled(True)

            self.ptimer.stop()         ##周期发送给控制机联络信息

    def down_udp_server_start(self):
        """
        开启下行UDP服务端方法
        :return:
        """
        self.down_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            port = Dport
            address = (self.dlocalip, port)
            self.down_udp_socket.bind(address)
            msg = '下行UDP打开...'
            print(msg)
        except Exception as ret:
            msg = '下行UDP没打开，请检查网络'
            #print(msg)
        else:
            self.down_sever_th = Thread(target=self.down_udp_server_concurrency)
            self.down_sever_th.start()
            msg = 'UDP服务端正在监听端口:{}\n'.format(port)
        print(msg)

    def down_udp_server_concurrency(self):
        """
        下行UDP通信线程，用于转发控制服务器数据给板卡
        :return:
        """
        buf = bytes.fromhex('7f 7f 7f 7f')
        while True:
            answerbuf = bytearray([0x0, 0x0, 0x0, 0x0, 0x0, 0x0])
            data = self.down_udp_socket.recv(1500)
            framelen = len(data)
            print(framelen)
            if framelen > 9:
                pass
            else:
                continue
            if framelen == 10 and data[6] == 0:     ##联机信号，直接回复给控制机
                sendbuf = buf + bytes(data)[4:]
                try:
                    self.upsock.sendto(sendbuf, (self.uipaddr, Uport))
                except:
                    print('error 05')
                self.tnum = self.tnum + 1
                continue
            elif framelen == 11 and data[6] == 0x02:  ##资源申请指令 order = 0x02
                answerbuf[0] = 0x0a
                answerbuf[2] = 0x02
                res = int(data[8]*256 +data[7])     ##控制机数据低位在前，高位在后
                #print(res)
                if res <=10 :
                    answerbuf[3] = 1
                    self.statusbar.showMessage('资源申请应答成功')
                else:
                    answerbuf[3] = 2
                    self.statusbar.showMessage('资源申请应答失败')
                sendbuf = buf + bytes(answerbuf)
                try:
                    self.upsock.sendto(sendbuf, (self.uipaddr, Uport))
                except:
                    print('error 06')

                self.tnum = self.tnum + 1
                continue
            else:
                receivedata = bytearray(data)
                crcdata = bytearray(data)[7:framelen - 2]
                wcrcbytes = self.crc16(str(crcdata))
                if wcrcbytes != int(data[-1] * 256 + data[-2]):
                    receivedata[framelen - 1] = int(wcrcbytes % 256)
                    receivedata[framelen - 2] = int(wcrcbytes / 256)
                    print(
                        'data {} crc:'.format(wcrcbytes) + str(int(wcrcbytes % 256)) + '  ' + str(int(wcrcbytes / 256)))

                oneframedata = bytes(data)[4:]
                tlength = len(oneframedata)
                partnum = int(tlength / framedatalen)
                lastlen = int(tlength % framedatalen)
                sframedata = bytearray(b'')
                sframedata.append((self.serialnum))
                if lastlen == 0:
                    totalnum = partnum - 1
                else:
                    totalnum = partnum
                print('totalnum = {} serilno = {} lastlen ={}'.format(totalnum,self.serialnum,lastlen))
                for i in range(totalnum + 1):
                    if i == 0:
                        sframedata.append(i)
                        sframedata.append(totalnum)
                        sframedata.append(lastlen)
                    else:
                        sframedata[1] = i
                    if i == totalnum:
                        sendframe = bytes(sframedata) + bytes(oneframedata[i * framedatalen:]) + bytes(zerodata[:(framedatalen - lastlen)])
                        self.serialnum = (self.serialnum + 1) % 256
                    else:
                        sendframe = sframedata + oneframedata[i * framedatalen:(i + 1) * framedatalen]
                    print(len(sendframe))

                    try:                                    ##其他指令转发给收发信机--通信机
                        self.downsock.sendto(sendframe, (self.ipaddr, self.cport))
                    except:
                        print('error 07')
                    time.sleep(0.001)

            self.tnum = self.tnum + 1

    def down_udp_close(self):
        """
        功能函数，关闭下行UDP通信
        :return:
        """
        try:
            stopThreading.stop_thread(self.down_sever_th)
        except Exception:
            print('关闭upd错')
            pass
        try:
            self.down_udp_socket.close()
            if self.delayworking is False:
                msg = '已断开网络\n'
                print(msg)
        except Exception as ret:
            pass

    def clearData(self):
        self.textBrowser_recv.clearHistory()
        self.textBrowser_recv.clear()
        self.num  = 0
        self.tnum = 0

    def cportchanged(self):
        self.cport = int(self.lineEdit_15.text())

    def update_connection(self):
        condata =bytearray([0x7f,0x7f,0x7f,0x7f,0x0b, 0x0, 0x01, 0x0a,0x0, 0xcb, 0xef])
        sendbuf = bytes(condata)
        self.upsock.sendto(sendbuf, (self.uipaddr, Uport))
        return

    def channelconnect(self):
        ##发送接受频率信息来确定联络状态
        self.rxFreqChanged()
    def info_udp_server_start(self):
        """
        开启下行UDP服务端方法
        :return:
        """
        self.info_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            port = Iport
            address = (self.dlocalip, port)
            self.info_udp_socket.bind(address)
            msg = '信息UDP打开'
            print(msg)
        except Exception as ret:
            msg = '信息UDP没打开，请检查网络'
            print(msg)
        else:
            self.info_sever_th = Thread(target=self.info_udp_server_concurrency)
            self.info_sever_th.start()
            msg = '信息UDP服务端正在监听端口:{}\n'.format(port)
            print(msg)

    def info_udp_server_concurrency(self):
        """
        下行UDP通信线程，用于转发控制服务器数据给板卡
        :return:
        """
        infosock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while True:
            data = self.info_udp_socket.recv(1500)
            framelen = len(data)
            print(framelen)
            if framelen > 0 and framelen <= 20:
                if data[3] == 0x10:
                    displaymsg = '发射频率设置成功！'
                if data[3] == 0x11:
                    displaymsg = '接收频率设置成功！'
                if data[3] == 0x18:
                    displaymsg = '接收增益设置成功！'
                if data[3] == 0x19:
                    displaymsg = '观察接收增益设置成功！'
                if data[3] == 0x1a:
                    displaymsg = '侦测接收增益设置成功！'
                if data[3] == 0x1b:
                    displaymsg = '收发通道使能设置成功！'
                self.statusbar.showMessage(displaymsg)
            else:
                continue

    def info_udp_close(self):
        """
        功能函数，关闭下行UDP通信
        :return:
        """
        try:
            stopThreading.stop_thread(self.info_sever_th)
        except Exception:
            print('关闭信息server upd错')
            pass
        try:
            self.info_udp_socket.close()
            if self.delayworking is False:
                msg = '已断开信息udp\n'
                print(msg)
        except Exception as ret:
            pass


    def closeEvent(self,event):
        reply = QMessageBox.information(self, '警告',"系统将退出，是否确认?", QMessageBox.Yes |QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.up_udp_close()
            self.down_udp_close()
            self.info_udp_close()
            self.delayworking = False

            self.savetofile()

            event.accept()
        else:
            event.ignore()

    def savetofile(self):
        txfile = open(specfile,'w',encoding='utf-8')
        str = self.lineEdit.text()
        txfile.write(str+'\n')
        str = self.lineEdit_2.text()
        txfile.write(str + '\n')
        str = self.lineEdit_3.text()
        txfile.write(str + '\n')
        str = self.lineEdit_4.text()
        txfile.write(str + '\n')
        str = self.lineEdit_6.text()
        txfile.write(str + '\n')
        str = self.lineEdit_8.text()
        txfile.write(str + '\n')
        str = self.lineEdit_7.text()
        txfile.write(str + '\n')
        str = self.lineEdit_9.text()
        txfile.write(str + '\n')
        str = self.lineEdit_10.text()
        txfile.write(str + '\n')
        str = self.lineEdit_12.text()
        txfile.write(str + '\n')
        str = self.lineEdit_11.text()
        txfile.write(str + '\n')
        #txfile.close()
        txfile.write('%s,%s,%s,%s\n'%(self.cb_index1,self.cb_index2,self.cb_index3,self.cb_index4))
        txfile.write('%s,%s,%s,%s,%s,%s,%s,%s,%s\n'%(int(self.agc_bool),int(self.orxagc_bool),\
                            int(self.rx1_enable),int(self.rx2_enable),int(self.tx1_enable), \
                            int(self.tx2_enable),int(self.rxsupp),int(self.txsupp),int(self.firstcrc)))
        str = self.lineEdit_14.text()
        txfile.write(str+'\n')

        str = self.lineEdit_15.text()
        txfile.write(str+'\n')
        txfile.close()
        return
    # 同步完毕第二部分

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
        self.RXConfig.extend((self.equa_amp_obj.value()).to_bytes(2, byteorder='big'))
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
        self.RXConfig.extend((self.Reserv_B0_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.Reserv_B1_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.Reserv_U0_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.Reserv_U1_obj.value()).to_bytes(1, byteorder='big'))
        self.RXConfig.extend((self.Reserv_I0_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.Reserv_I1_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.Reserv_I2_obj.value()).to_bytes(2, byteorder='big'))
        self.RXConfig.extend((self.Reserv_I3_obj.value()).to_bytes(4, byteorder='big'))
        self.RXConfigBytes = bytes(self.RXConfig)

    def connectLDPCConfig(self):
        modeTable = [0, 13, 26, 39]
        self.LDPCConfig.clear()
        flag = 0
        self.LDPCConfig.extend((flag).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.LDPC_loop_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.Pause_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.Mtype_txldpc_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.Mtype_rxldpc_obj.currentIndex()).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.car_num_txldpc_obj.value()).to_bytes(3, byteorder='big'))
        self.LDPCConfig.extend((self.car_num_rxldpc_obj.value()).to_bytes(3, byteorder='big'))
        if self.calculateValidBitNum(self.car_num_txldpc_obj.value()) > 7:
            modeTx = self.calculateValidBitNum(self.car_num_txldpc_obj.value()) - 8 + modeTable[self.Mtype_txldpc_obj.currentIndex()]
        else:
            modeTx = 255
        if self.calculateValidBitNum(self.car_num_rxldpc_obj.value()) > 7:
            modeRx = self.calculateValidBitNum(self.car_num_rxldpc_obj.value()) - 8 + modeTable[self.Mtype_rxldpc_obj.currentIndex()]
        else:
            modeRx = 255
        self.LDPCConfig.extend((modeTx).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((modeRx).to_bytes(1, byteorder='big'))
        LDPCreset = 1 if self.LDPC_reset_obj.isChecked() else 0
        self.LDPCConfig.extend((LDPCreset).to_bytes(1, byteorder='big'))
        self.LDPCConfig.extend((self.Reserv_1_obj.value()).to_bytes(2, byteorder='big'))
        self.LDPCConfig.extend((self.Reserv_2_obj.value()).to_bytes(2, byteorder='big'))
        self.LDPCConfig.extend((self.Reserv_3_obj.value()).to_bytes(2, byteorder='big'))
        self.LDPCConfig.extend((self.Reserv_4_obj.value()).to_bytes(4, byteorder='big'))
        self.LDPCConfig.extend((self.Reserv_5_obj.value()).to_bytes(4, byteorder='big'))
        self.LDPCConfig.extend((self.Reserv_6_obj.value()).to_bytes(4, byteorder='big'))
        self.LDPCConfigBytes = bytes(self.LDPCConfig)

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
        self.Time_trig2tx_obj.setValue(12800)
        self.Time_tx_hold_obj.setValue(34000)
        self.ms_T_sync2trig_obj.setValue(21880)
        self.bs_tdd_time_gap_obj.setValue(100000)
        self.bs_tx_time_gap_obj.setValue(50000)
        self.Trig_gap_cnt_obj.setValue(100000)
        self.alway_tx_obj.setCurrentIndex(0)
        self.tx1_en_obj.setCurrentIndex(1)
        self.udpfifo_reset_obj.setCurrentIndex(0)
        self.rx1_en_obj.setCurrentIndex(1)
        self.rx2_en_obj.setCurrentIndex(0)
        self.TDD_EN_obj.setCurrentIndex(1)
        self.UDP_loop_obj.setCurrentIndex(0)

        # 设置收端默认参数
        self.MType_rx_obj.setCurrentIndex(0)
        self.agc_en_obj.setCurrentIndex(1)
        self.frft_en_obj.setCurrentIndex(1)
        self.car_num_rx_obj.setValue(0xfffff)
        self.i_freq_est_obj.setValue(0)
        self.sync_factor_obj.setValue(20384)
        self.equa_factor_obj.setValue(4122)
        self.Alpha_rx_obj.setValue(0.5)
        self.equa_amp_obj.setValue(16384)
        self.equa_amp_en_obj.setCurrentIndex(1)
        self.car_thr1_obj.setValue(102656)
        self.car_thr2_obj.setValue(6250000)
        self.car_thr3_obj.setValue(625000000)
        self.mmse_thr_obj.setValue(1000)
        self.scale_equa_obj.setValue(0)
        self.scale_fft_obj.setValue(0)
        self.phase_en_obj.setCurrentIndex(0)
        self.phase_factor_obj.setValue(0)
        self.freq_offset_en_obj.setCurrentIndex(0)
        self.LDPC_en_obj.setCurrentIndex(0)
        self.MMSEorLS_obj.setCurrentIndex(0)
        self.as_time_trig2tx_obj.setValue(17339)
        self.as_trig2tx_cnt_obj.setValue(58735)

        # 设置射频参数
        self.Amplifier_obj.setCurrentIndex(0)
        self.filter3_3p5G_obj.setCurrentIndex(0)
        self.filter3p5_4G_obj.setCurrentIndex(0)
        self.filter4p5_5G_obj.setCurrentIndex(0)
        self.filter5_5p5G_obj.setCurrentIndex(0)

        #设置LDPC配置参数
        self.LDPC_loop_obj.setCurrentIndex(0)
        self.Pause_obj.setCurrentIndex(0)
        self.Mtype_txldpc_obj.setCurrentIndex(0)
        self.Mtype_rxldpc_obj.setCurrentIndex(0)
        self.car_num_txldpc_obj.setValue(0xfffff)
        self.car_num_rxldpc_obj.setValue(0xfffff)
        self.LDPC_reset_obj.setChecked(False)

    def sendConfigtoBaseBand(self):
        self.ipBB = self.BBIP_obj.text()
        self.portBB = int(self.BBPort_obj.text())
        self.ipRF = self.RFIP_obj.text()
        self.portRF = int(self.RFPort_obj.text())
        self.ipLDPC = self.LDPC_IP_obj.text()
        self.portLDPC = int(self.LDPC_Port_obj.text())
        addrBB = (self.ipBB, self.portBB)
        addrRF = (self.ipRF, self.portRF)
        addrLDPC = (self.ipLDPC, self.portLDPC)
        configSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            configSocket.bind((self.ulocalip, 8000))
        except OSError:
            print('本地IP绑定失败，请核对本地IP与配置文件specip.txt中的设置是否一致')
        self.connectRFConfigData()
        print(self.RFConfig)
        for configRF in self.RFConfig:
            configSocket.sendto(configRF, addrRF)
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
        self.connectLDPCConfig()
        print(self.LDPCConfig)
        configSocket.sendto(self.LDPCConfigBytes, addrLDPC)
        configSocket.close()

    def setDefaultConfig(self):
        self.connectDefaultConfig()
        self.sendConfigtoBaseBand()

    '''def count_dot(self, value):
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
        self.beg_y = self.end_y'''

    def showBasebandStatistical(self, data):
        errbit = int.from_bytes(data[0:4], byteorder='big')
        self.errbit_show.setText(str(errbit))
        tolfrm = int.from_bytes(data[4:8], byteorder='big')
        self.tolfrm_show.setText(str(tolfrm))
        noise_pwr_sum = int.from_bytes(data[8:10], byteorder='big')
        self.noise_pwr_sum_show.setText(str(noise_pwr_sum))
        singal_pwr_sum = int.from_bytes(data[10:12], byteorder='big')
        self.singal_pwr_sum_show.setText(str(singal_pwr_sum))
        phase_est = int.from_bytes(data[12:14], byteorder='big', signed=True)
        self.phase_est_show.setText(str(phase_est/8096*45))
        o_freq_est_t = int.from_bytes(data[14:16], byteorder='big', signed=True)
        self.o_freq_est_t_show.setText(str(o_freq_est_t))
        crc_error = int.from_bytes(data[16:17], byteorder='big')
        self.crc_error_show.setText(str(crc_error))
        over_flag_rx = int.from_bytes(data[17:18], byteorder='big')
        self.over_flag_rx_show.setText(str(over_flag_rx))
        FIFOEmpty = int.from_bytes(data[18:19], byteorder='big')
        self.FIFOEmpty_show.setText(str(FIFOEmpty))
        SNR = 0 if (noise_pwr_sum == 0) else (10 * math.log10(singal_pwr_sum * math.pow(2, 9) / noise_pwr_sum))
        self.SNR_current_show.setText(str(SNR))
        #self.count_dot(SNR)
        self.PrepareSNRSamples(SNR)
        car1 = int.from_bytes(data[19:22], byteorder='big')
        self.car1_hex_show.setText(hex(car1))
        car2 = int.from_bytes(data[22:25], byteorder='big')
        self.car2_hex_show.setText(hex(car2))
        car3 = int.from_bytes(data[25:28], byteorder='big')
        self.car3_hex_show.setText(hex(car3))
        MtypeTable = ['QPSK', '16QAM', '64QAM', '256QAM']
        Rb_mtype_tx = int.from_bytes(data[28:29], byteorder='big')
        self.Rb_mtype_tx_show.setText(MtypeTable[Rb_mtype_tx])
        Rb_mtype_rx = int.from_bytes(data[29:30], byteorder='big')
        self.Rb_mtype_rx_show.setText(MtypeTable[Rb_mtype_rx])
        Rb_car_tx = int.from_bytes(data[30:33], byteorder='big')
        self.Rb_car_tx_show.setText(hex(Rb_car_tx))
        Rb_car_rx = int.from_bytes(data[33:36], byteorder='big')
        self.Rb_car_rx_show.setText(hex(Rb_car_rx))
        rx_frame_cnt = int.from_bytes(data[36:39], byteorder='big')
        self.cnt_rx_frame_show.setText(str(rx_frame_cnt))
        tx_frame_cnt = int.from_bytes(data[39:42], byteorder='big')
        self.cnt_frame_tx_show.setText(str(tx_frame_cnt))

    def showLDPCStatistical(self, data):
        errbit = int.from_bytes(data[0:4], byteorder='big')
        self.errbit_show.setText(str(errbit))
        tolfrm = int.from_bytes(data[4:8], byteorder='big')
        self.tolfrm_show.setText(str(tolfrm))
        noise_pwr_sum = int.from_bytes(data[8:10], byteorder='big')
        self.noise_pwr_sum_show.setText(str(noise_pwr_sum))
        singal_pwr_sum = int.from_bytes(data[10:12], byteorder='big')
        self.singal_pwr_sum_show.setText(str(singal_pwr_sum))
        phase_est = int.from_bytes(data[12:14], byteorder='big', signed=True)
        self.phase_est_show.setText(str(phase_est / 8096 * 45))
        o_freq_est_t = int.from_bytes(data[14:16], byteorder='big', signed=True)
        self.o_freq_est_t_show.setText(str(o_freq_est_t))
        crc_error = int.from_bytes(data[16:17], byteorder='big')
        self.crc_error_show.setText(str(crc_error))
        over_flag_rx = int.from_bytes(data[17:18], byteorder='big')
        self.over_flag_rx_show.setText(str(over_flag_rx))
        FIFOEmpty = int.from_bytes(data[18:19], byteorder='big')
        self.FIFOEmpty_show.setText(str(FIFOEmpty))
        SNR = 0 if (noise_pwr_sum == 0) else (10 * math.log10(singal_pwr_sum * math.pow(2, 9) / noise_pwr_sum))
        self.SNR_current_show.setText(str(SNR))
        #self.count_dot(SNR)
        self.PrepareSNRSamples(SNR)
        car1 = int.from_bytes(data[19:22], byteorder='big')
        self.car1_hex_show.setText(hex(car1))
        car2 = int.from_bytes(data[22:25], byteorder='big')
        self.car2_hex_show.setText(hex(car2))
        car3 = int.from_bytes(data[25:28], byteorder='big')
        self.car3_hex_show.setText(hex(car3))
        MtypeTable = ['QPSK', '16QAM', '64QAM', '256QAM']
        Rb_mtype_tx = int.from_bytes(data[28:29], byteorder='big')
        self.Rb_mtype_tx_show.setText(MtypeTable[Rb_mtype_tx])
        Rb_mtype_rx = int.from_bytes(data[29:30], byteorder='big')
        self.Rb_mtype_rx_show.setText(MtypeTable[Rb_mtype_rx])
        Rb_car_tx = int.from_bytes(data[30:33], byteorder='big')
        self.Rb_car_tx_show.setText(hex(Rb_car_tx))
        Rb_car_rx = int.from_bytes(data[33:36], byteorder='big')
        self.Rb_car_rx_show.setText(hex(Rb_car_rx))

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
        self.IQinFigure.ax.set_xlim(-0.5, 0.5)
        self.IQinFigure.ax.set_ylim(-0.5, 0.5)

    def PrepareIQOutCanvas(self):
        self.IQOutFigure = Figure_Canvas()
        #self.IQOutFigure.ax.spines['top'].set_color('none')
        #self.IQOutFigure.ax.spines['right'].set_color('none')
        #self.IQOutFigure.ax.xaxis.set_ticks_position('bottom')
        #self.IQOutFigure.ax.spines['bottom'].set_position(('data', 0))
        #self.IQOutFigure.ax.yaxis.set_ticks_position('left')
        #self.IQOutFigure.ax.spines['left'].set_position(('data', 0))
        self.IQOutFigureLayout = QGridLayout(self.IQShowFRFTout)
        self.IQOutFigureLayout.addWidget(self.IQOutFigure)
        self.IQOutFigure.ax.set_xlim(-0.5, 0.5)
        self.IQOutFigure.ax.set_ylim(-0.5, 0.5)

    def PreparePilotLineCanvas(self):
        self.LineFigure = Figure_Canvas()
        self.LineFigureLayout = QGridLayout(self.IQshowPilot)
        self.LineFigureLayout.addWidget(self.LineFigure)
        self.LineFigure.ax.set_xlim(-1, 1)
        self.LineFigure.ax.set_ylim(-1, 1)
        #self.line = Line2D([0], [0])

    def PrepareSNRLineCanvas(self):
        self.SNRLineFigure = Figure_Canvas()
        self.SNRLineFigureLayout = QGridLayout(self.SNRshow)
        self.SNRLineFigureLayout.addWidget(self.SNRLineFigure)
        self.SNRLineFigure.ax.set_xlim(0, 100)
        self.SNRLineFigure.ax.set_ylim(0, 50)

    def PrepareSNRSamples(self, value):
        if len(self.SNRDataArray) >= 100:
            self.SNRDataArray = self.SNRDataArray[-100: ]
        self.SNRDataArray.append(value)
        SNRLine = Line2D(list(range(len(self.SNRDataArray))), self.SNRDataArray)
        self.SNRLineFigure.ax.add_line(SNRLine)
        self.SNRLineFigure.draw()

    def getIQdata(self, sourceData, Idata, Qdata, targetLen=1280):
        for i in range(int(len(sourceData)/4)):
            ivalue = int.from_bytes(sourceData[0:2], byteorder='big', signed=True)
            qvalue = int.from_bytes(sourceData[2:4], byteorder='big', signed=True)
            Idata.append(ivalue/65535)
            Qdata.append(qvalue/65535)
            sourceData = sourceData[4:]
        if len(Idata) >= targetLen:
            #print("Idata:", Idata)
            return True
        else:
            return False

    def showStatistical(self, data):
        flag = int.from_bytes(data[0:1], byteorder='big')
        print(flag)
        Statistical = data[1:]
        if flag == 1:
            self.showBasebandStatistical(Statistical)
        elif flag == 2:
            if(self.getIQdata(Statistical, self.IdataIn, self.QdataIn, 1024) == True):
                self.IQinFigure.ax.scatter(self.IdataIn, self.QdataIn, 3)
                self.IQinFigure.draw()
                #print("IQdata in")
                self.IdataIn.clear()
                self.QdataIn.clear()
        elif flag == 3:
            if (self.getIQdata(Statistical, self.IdataOut, self.QdataOut, 1024) == True):
                self.IQOutFigure.ax.scatter(self.IdataOut, self.QdataOut, 3)
                self.IQOutFigure.draw()
                #print("IQdata out")
                self.IdataOut.clear()
                self.QdataOut.clear()
        elif flag == 4:
            if (self.getIQdata(Statistical, self.IdataPilot, self.QdataPilot, 320) == True):
                self.line = Line2D(self.IdataPilot, self.QdataPilot)
                self.LineFigure.ax.add_line(self.line)
                self.LineFigure.draw()
                #print("IQPilot draw")
                self.IdataPilot.clear()
                self.QdataPilot.clear()
        elif flag == 5:
            self.showLDPCStatistical(Statistical)
        else:
            print("无效的统计数据")

    def updateStatistical(self):
        # print('timer run')
        if len(dataStatistical) > 0:
            self.showStatistical(dataStatistical)
        else:
            print("暂未收到统计数据")

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
    statisticThread = StatisticThread()
    statisticThread.start()
    dataThread = DataThread()
    dataThread.start()
    app = QApplication(sys.argv)
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())
import socket, serial, pynmea2

def getChosenIP(segment):
    hostName = socket.gethostname()
    ipAddrInfoList = socket.getaddrinfo(hostName, None, 2)
    ipList = [item[4][0] for item in ipAddrInfoList]
    for ip in ipList:
        if segment == ip.split(".")[2]:
            print('IP匹配成功！！本机IP为：', ip)
            return ip
    print('本机无网段为%s的IP，请检查本机IP设置！！' % segment)
    print('本机IP为：', ipList)
    return ipList[0]

portFile = 'setPort.txt'
def getStatisticalPort(MDport=7000, LDPCport=7010, SpectrumPort=7020, IQport=7001, SSSysport=7002, SSCorrValuePort=7003):
    try:
        with open(portFile, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            MDport = int(lines[0].strip().split(":")[1])
            LDPCport = int(lines[1].strip().split(":")[1])
            SpectrumPort = int(lines[2].strip().split(":")[1])
            IQport = int(lines[3].strip().split(":")[1])
            SSSysport = int(lines[4].strip().split(":")[1])
            SSCorrValuePort = int(lines[5].strip().split(":")[1])
    except FileNotFoundError:
        print("提示：未设置统计端口port，以默认端口启动")
    return MDport, LDPCport, SpectrumPort, IQport, SSSysport, SSCorrValuePort

def getGPSdata(port='COM3', baudrate=38400):
    try:
        ser = serial.Serial(port, baudrate=baudrate)
        line = bytes.decode(ser.readline())
        if line.startswith('$GNRMC'):
            rmc = pynmea2.parse(line)
            print(rmc.longitude, rmc.latitude)
    except serial.serialutil.SerialException as error:
        print("串口连接错误：")
        print(error)

def print_bytes_hex(data):
    lin = ['%02X' % i for i in data]
    # print(" ".join(lin))
    return " ".join(lin)

class ConnectRFConfig():
    def __init__(self):
        self.addrhead = [0x00, 0x01, 0x02, 0x03, 0x04, 0x10, 0x11, 0x12, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F]
        self.txConfig = bytearray(20)
        self.fpgaAddr = [0x101, 0x102, 0x103, 0x104]

    def _buildConfig(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, value: int):
        """通用配置构建方法：设置基础信息、寄存器信息和数据值"""
        # setBaseInfo
        fpgaAddr = self.fpgaAddr[fpgaindex]
        if 0 <= wrinfo <= 1:
            WR_bit = wrinfo
        else:
            WR_bit = 1
            print("读写配置错误")
        bytesInfo = WR_bit * 2**15 + fpgaAddr
        self.txConfig[:2] = bytesInfo.to_bytes(2, byteorder='big')
        # connectRegisterInfo
        if fmcinfo == 1:
            self.txConfig[2:4] = (fmcinfo * 2**12 + registerinfo).to_bytes(2, byteorder='big')
        # set payload
        self.txConfig[16:] = value.to_bytes(4, byteorder='big')
        return self.txConfig

    def connectFreqInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, value):
        """用于组织发射频率、接收频率以及侦测接收频率的配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(value * 10 ** 9 / 100))

    def connectTXAttenuationInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, channel1, channel2):
        """用于组织发射衰减配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(channel1/0.5*2**8 + channel2/0.5))

    def connectRXGainInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, agcenable, channel1, channel2, step):
        """用于组织接收增益、观察接收增益配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(agcenable*2**16 + channel1/step*2**8 + channel2/step))

    def connectDetectionRXGainInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, channelA):
        """用于组织侦测接收增益配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(channelA))

    def connectTXRXEnableInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, rx1, rx2, tx1, tx2):
        """用于组织收发通道使能配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(rx1*2**3 + rx2*2**2 + tx1*2**1 + tx2))

    def connectBaseBandSampleRateInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, samplerate):
        """用于组织基带采样率配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(samplerate))

    def connectClockInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, outerclock, sourceclock):
        """用于组织时钟选择配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(outerclock*2**2 + sourceclock))

    def connectObservationChannelInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, channel):
        """用于组织观察通道选择配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(channel))

    def connectCalibrationInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo, value):
        """用于组织校准配置数据"""
        return self._buildConfig(fpgaindex, wrinfo, fmcinfo, registerinfo, int(value))

    def getHardwareAndVersionInfo(self, fpgaindex: int, wrinfo: int, fmcinfo: int, registerinfo):
        """用于获取硬件版本信息"""
        pass

if __name__ == "__main__":
    config = ConnectRFConfig()
    print("设置发送频率")
    print_bytes_hex(config.connectFreqInfo(0, 1, 1, config.addrhead[5], 3.0))  # 设置发送频率
    print("设置接收频率")
    print_bytes_hex(config.connectFreqInfo(0, 1, 1, config.addrhead[6], 3.0))  # 设置接收频率
    print("设置侦听频率")
    print_bytes_hex(config.connectFreqInfo(0, 1, 1, config.addrhead[7], 2))  # 设置侦听频率
    print("设置发射衰减")
    print_bytes_hex(config.connectTXAttenuationInfo(0, 1, 1, config.addrhead[8], 30.5, 30.5))  # 设置发射衰减
    print("设置接收增益")
    print_bytes_hex(config.connectRXGainInfo(0, 1, 1, config.addrhead[9], 0, 10.5, 10.5, 0.5))  # 设置接收增益
    print("设置观察接收增益")
    print_bytes_hex(config.connectRXGainInfo(0, 1, 1, config.addrhead[10], 0, 18, 18, 1))  # 设置观察接收增益
    print("设置侦测接收增益")
    print_bytes_hex(config.connectDetectionRXGainInfo(0, 1, 1, config.addrhead[11], 52))  # 设置侦测接收增益
    print("设置收发通道使能")
    print_bytes_hex(config.connectTXRXEnableInfo(0, 1, 1, config.addrhead[12], 1, 1, 1, 1))  # 设置收发通道使能
    print("设置基带采样率")
    print_bytes_hex(config.connectBaseBandSampleRateInfo(0, 1, 1, config.addrhead[13], 4))  # 设置基带采样率
    print("设置时钟源选择")
    print_bytes_hex(config.connectClockInfo(0, 1, 1, config.addrhead[14], 2, 1))  # 设置参考时钟选择
    print("设置观察通道选择")
    print_bytes_hex(config.connectObservationChannelInfo(0, 1, 1, config.addrhead[15], 2))  # 设置观察通道选择
    getStatisticalPort()
    getGPSdata()
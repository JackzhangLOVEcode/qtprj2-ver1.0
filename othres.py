import socket


def getChosenIP(segment):
    hostName = socket.gethostname()
    ipAddrInfoList = socket.getaddrinfo(hostName, None, 2)
    ipList = []
    for item in ipAddrInfoList:
        ipList.append(item[4][0])
    for ip in ipList:
        if segment == ip.split(".")[2]:
            return ip
    print('本机所有无网段为%s的IP，请检查本机IP设置！！'%segment)
    print('本机IP为：', ipList)
    return ipList[0]

def print_bytes_hex(data):
    lin = ['%02X' % i for i in data]
    # print(" ".join(lin))
    return " ".join(lin)


class ConnectRFConfig():
    def __init__(self):
        self.addrhead = [0x00, 0x01, 0x02, 0x03, 0x04, 0x10, 0x11, 0x12, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F]
        self.txConfig = bytearray(20)
        self.fpgaAddr = [0x101, 0x102, 0x103, 0x104]

    def setBaseInfo(self, fpgaindex:int, wrinfo:int):
        fpgaAddr = self.fpgaAddr[fpgaindex]
        if wrinfo >= 0 and wrinfo <= 1:
            WR_bit = wrinfo
        else:
            WR_bit = 1
            print("读写配置错误")
        bytesInfo = WR_bit * 2**15 + fpgaAddr
        bytesInfoArray = bytesInfo.to_bytes(2, byteorder='big')
        self.txConfig[:2] = bytesInfoArray

    def connectRegisterInfo(self, fmcinfo:int, registerinfo):
        if fmcinfo == 1:
            byteInfo = (fmcinfo * 2**12 + registerinfo).to_bytes(2, byteorder='big')
            self.txConfig[2:4] = byteInfo

    def connectFreqInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, value):
        """用于组织发射频率、接收频率以及侦测接收频率的配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(value * 10 ** 9 / 100).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectTXAttenuationInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, channel1, channel2):
        """用于组织发射衰减配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(channel1/0.5*2**8 + channel2/0.5).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectRXGainInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, agcenable, channel1, channel2, step):
        """用于组织接收增益、观察接收增益配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(agcenable*2**16 + channel1/step*2**8 + channel2/step).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectDetectionRXGainInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, channelA):
        """用于组织侦测接收增益配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(channelA).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectTXRXEnableInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, rx1, rx2, tx1, tx2):
        """用于组织收发通道使能配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(rx1*2**3 + rx2*2**2 + tx1*2**1 + tx2).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectBaseBandSampleRateInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, samplerate):
        """用于组织基带采样率配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(samplerate).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectClockInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, outerclock, sourceclock):
        """用于组织时钟选择配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(outerclock*2**2 + sourceclock).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectObservationChannelInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, channel):
        """用于组织观察通道选择配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(channel).to_bytes(4, byteorder='big')
        return self.txConfig

    def connectCalibrationInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo, value):
        """用于组织校准配置数据"""
        self.setBaseInfo(fpgaindex, wrinfo)
        self.connectRegisterInfo(fmcinfo, registerinfo)
        self.txConfig[16:] = int(value).to_bytes(4, byteorder='big')
        return self.txConfig

    def getHardwareAndVersionInfo(self, fpgaindex:int, wrinfo:int, fmcinfo:int, registerinfo):
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
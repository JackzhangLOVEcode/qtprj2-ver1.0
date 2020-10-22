import socket


def getChosenIP(segment: "输入目标网段,格式为字符串"):
    hostName = socket.gethostname()
    ipAddrInfoList = socket.getaddrinfo(hostName, None, 2)
    ipList = []
    for item in ipAddrInfoList:
        ipList.append(item[4][0])
    for ip in ipList:
        if segment == ip.split(".")[2]:
            return ip
    print('本机所有网卡无目标网段IP，请检查本机IP设置或者重新设置目标网段！！')
    return '127.0.0.1'
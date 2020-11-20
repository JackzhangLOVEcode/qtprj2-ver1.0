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
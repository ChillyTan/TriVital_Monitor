# *_* coding : UTF-8 *_*
# 开发团队 ：乐育科技
# 开发人员 ：zcq
# 开发时间 ：2022/6/16 16:53
# 文件名称 ：PackUnpack.PY 
# 开发工具 ：PyCharm


class PackUnpack:
    def __init__(self):
        # 数据包的长度默认为0
        self.sPackLen = 0
        # 获取到数据包ID标志默认为False，即尚未获取到ID
        self.sGotPackId = False
        # 剩余的字节数默认为0
        self.sRestByteNum = 0
        # 数据包初始化值为0，长度固定为10，即1个模块ID，1个数据头，1个二级ID，6个数据，1个校验和
        self.mBufList = [0] * 10

    def packData(self, listPack):
        success = False
        if listPack[0] < 0x80:         # 数据包ID必须在0x00-0x7F之间, listPack[0]为数据包ID
            while len(listPack) < 10:  # 统一待打包的数据包为10字节，不足补0，方便后续列表操作
                listPack.append(0)
            self.packWithCheckSum(listPack)
            success = True
        return success

    def packWithCheckSum(self, pack):
        # 规定包的长度为10，不为10即为错误包
        if len(pack) != 10:
            return
        checkSum = pack[0]   # 取出模块ID，加到校验和
        dataHead = 0         # 数据头清零
        for i in range(8, 1, -1):
            dataHead <<= 1   # 数据头左移一位后重新赋给dataHead，后面数据的最高位位于dataHead的低位
            pack[i] = (pack[i - 1]) | 0x80  # 对数据进行最高位置1操作，并将数据位置右移一位（因为数据头要插在模块ID后面）
            checkSum += pack[i]                      # 数据加到校验和，包括二级ID
            dataHead |= ((pack[i - 1]) & 0x80) >> 7  # 取出原始数据的最高位，与dataHead相或
        pack[1] = dataHead | 0x80           # 数据头在打包后数据包的第二个位置，仅次于包头，数据头的最高位也要置为1
        checkSum += pack[1]  # 将数据头加到校验和
        pack[9] = (checkSum & 0xFF) | 0x80  # 取低八位，校验和的最高位也要置为1

    def unpackData(self, data):
        findPack = False
        if self.sGotPackId:  # 已经接收到包ID
            if data >= 0x80:
                # 数据包中的数据从第二个字节开始存储，因为第一个字节是包ID
                self.mBufList[self.sPackLen] = data
                self.sPackLen += 1
                # 剩余字节数自减
                self.sRestByteNum -= 1
                # 已经接收到完整的数据包，包长为10
                if self.sRestByteNum >= 0 and self.sPackLen == 10:
                    findPack = self.unpackWithCheckSum(self.mBufList)
                    self.sGotPackId = False
            else:
                pass
        elif data < 0x80:
            # 当前的数据为包ID
            self.sRestByteNum = 9    # 剩余的包长，即打包好的包长减去1
            self.sPackLen = 1        # 接收到包ID，即表示包长为1
            self.mBufList[0] = data  # 数据包的ID赋值给mBufList
            self.sGotPackId = True   # 表示已经接收到包ID
        return findPack

    def unpackWithCheckSum(self, listPack):
        if len(listPack) != 10:  # 包长度不为10，返回false
            return False
        checkSum = listPack[0]   # 取出模块ID，加到校验和
        dataHead = listPack[1]   # 取出数据包的数据头，赋给dataHead
        checkSum += dataHead     # 将数据头加到校验和
        for i in range(1, 8):
            checkSum += listPack[i + 1]  # 将数据依次加到校验和
            # 还原有效的8位数据
            listPack[i] = (listPack[i + 1] & 0x7f) | ((dataHead & 0x1) << 7)
            dataHead >>= 1
        # 判断校验和
        if (checkSum & 0x7f) != ((listPack[9]) & 0x7f):
            return False
        return True

    def getUnpackRslt(self):
        return self.mBufList

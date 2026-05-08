
# CRC CCITT (XModem)
# @Time: 2020/9/7
# @Site: https://stackoverflow.com/questions/25239423/crc-ccitt-16-bit-python-manual-calculation
# @Test Site: https://www.lammertbies.nl/comm/info/crc-calculation
POLYNOMIAL = 0x1021
PRESET = 0

def _initial(c):
    crc = 0
    c = c << 8
    for j in range(8):
        if (crc ^ c) & 0x8000:
            crc = (crc << 1) ^ POLYNOMIAL
        else:
            crc = crc << 1
        c = c << 1
    return crc

_tab = [ _initial(i) for i in range(256) ]

def _update_crc(crc, c):
    cc = 0xff & c

    tmp = (crc >> 8) ^ cc
    crc = (crc << 8) ^ _tab[tmp & 0xff]
    crc = crc & 0xffff
    return crc

def crc(s):
    crc = PRESET
    for c in s:
        crc = _update_crc(crc, ord(c))
    return crc

def crcb(*i):
    crc = PRESET
    for c in i:
        crc = _update_crc(crc, c)
    return crc
def crcbytes(i):
    crc = PRESET
    for c in i:
        crc = _update_crc(crc, c)
    return crc

if __name__ == "__main__":
    data = b"123456789"
    print(hex(crc(data.decode())))
    print(hex(crcbytes(data)))
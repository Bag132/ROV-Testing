class SMBus():
    def __init__(self, bus_num):
        print('SMBus | init')

    def read_byte_data(self, address, reg):
        # print('SMBus | Read byte data')
        return 0

    def write_byte_data(self, address, reg, data):
        # print(f'SMBus | Write byte data {data}')
        return 0

    def write_byte(self, address, data):
        # print(f'SMBus | Write byte {data}')
        return 0

    def write_i2c_block_data(self, address, reg, data):
        # print(f'SMBus | Write block {data}')
        return 0

    def close(self):
        print('SMBus | Close')
        return 0

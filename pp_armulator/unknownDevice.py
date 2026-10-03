class unknown:
    def __init__(self, mapingBegin, mapingEnd, size=512):
        self.mapingEnd = mapingEnd
        self.mapingBegin = mapingBegin
        self.storage = bytearray(size)
        
    def mountImage(self, imageFile):
        pass
    
    def read(self, address, lenght):
        print(f"UNKNOWN: trying to read from {address:08x}")
        return bytes(lenght)
    
    def write(self, adress, data):
        print(f"UNKNOWN: {data:08x} writen to adress {adress:08x}")


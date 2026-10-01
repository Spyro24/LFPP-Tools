class lcd:
    def __init__(self, mapingBegin, mapingEnd, size=512):
        self.mapingEnd = mapingEnd
        self.mapingBegin = mapingBegin
        self.image = bytearray(size)
        
    def mountImage(self, imageFile):
        pass
    
    def read(self):
        pass
    
    def write(self, adress, data):
        print(f"LCD: {data:08x} writen to adress {adress:08x}")
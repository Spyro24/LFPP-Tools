class bootRom:
    def __init__(self, mapingBegin: int, mapingEnd: int):
        self.mapingEnd = mapingEnd
        self.mapingBegin = mapingBegin
        self.rom = bytearray(mapingEnd - mapingBegin) #empty rom
    
    def mapOut(self): #we are setting the maping to -1 to make sure nothing can access it
        self.mapingBegin = -1
        self.mapingEnd = -1
        
    def mountImage(self, imageFilePath: str):
        image = open(imageFilePath, "br")
        self.rom = bytearray(image.read())
        image.close()
        print("Boot Rom: Boot Rom image mounted")
    
    def read(self, address, lenght):
        address = address - self.mapingBegin
        return(self.rom[address:address + lenght])
    
    def write(self, adress, data):
        print(f"Boot Rom: Write unsuported (Address: 0x{adress:08x}, Value: {data:08x})")
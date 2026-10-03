class SDRAM:
    def __init__(self, mapingBegin: int, mapingEnd: int):
        self.mapingEnd = mapingEnd
        self.mapingBegin = mapingBegin
        self.ram = bytearray((mapingEnd - mapingBegin) + 1) #empty rom
    
    def mapOut(self): #we are setting the maping to -1 to make sure nothing can access it
        self.mapingBegin = -1
        self.mapingEnd = -1
        
    def mountImage(self, imageFilePath: str):
        image = open(imageFilePath, "br")
        data = image.read()
        for n in range(len(data)):
            self.ram[n] = data[n]
        image.close()
        print("Ram: Ram image mounted")
    
    def read(self, address, lenght):
        address = address - self.mapingBegin
        return(self.ram[address:address + lenght])
    
    def write(self, adress, data):
        print(f"Boot Rom: Write unsuported (Address: 0x{adress:08x}, Value: {data:08x})")

class ISRAM:
    def __init__(self, mapingBegin: int, mapingEnd: int):
        self.mapingEnd = mapingEnd
        self.mapingBegin = mapingBegin
        self.ram = bytearray((mapingEnd - mapingBegin) + 1) #empty rom
    
    def read(self, address, lenght):
        address = address - self.mapingBegin
        return(self.ram[address:address + lenght])
    
    def write(self, adress, data):
        self.ram[adress - self.mapingBegin:adress - self.mapingBegin + 4] = int(data).to_bytes(4, "little")
    
    def dump(self):
        dump = open("./ISRAM_dump", "bw")
        for byte in self.ram:
            dump.write(int(byte).to_bytes(1))
        dump.close()
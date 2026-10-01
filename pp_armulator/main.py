import unknownDevice
import nand
import bootRom
import helper

#CPU Mode Table
MODE_USR = 0b10000 #User
MODE_FIQ = 0b10001
MODE_IRQ = 0b10010
MODE_SVC = 0b10011 #Supervisor
MODE_ABT = 0b10111 #Abort
MODE_UND = 0b11011 #Undefined
MODE_SYS = 0b11111 #System

class EmuError(Exception):
    pass

class CPU:
    def __init__(self):
        self.mode = MODE_SVC
        self.halted = False
        self.bus = [bootRom.bootRom(0x0, 0x5FF0),
                    unknownDevice.unknown(0xc00000c4, 0xc00000cc),
                    unknownDevice.unknown(0xff000000, 0xff000000),
                    unknownDevice.unknown(0xd0100038, 0xd0100038),
                    unknownDevice.unknown(0xd0300000, 0xd03000c0),
                    unknownDevice.unknown(0xd05001fc, 0xd05001fc),
                    unknownDevice.unknown(0xd07023c0, 0xd07023c0),
                    nand.nand(0xD0900000, 0xD09001FF),
                    unknownDevice.unknown(0xd0d00000, 0xd0d00000),
                    unknownDevice.unknown(0xf8000000, 0xf8001FFF), #ISRAM (internal_static_ram)
                    #unknownDevice.unknown(0xfffffffc, 0xfffffffc), #Stack (but it cant be the stack)
                    ]
        self.registers = {"R0": 0x0, "pc":0x0, "cpsr":0xD3}
        self.registerBanks = {}
        #steps
        self.stepFetch = 0xE1A00000 #Nop
        self.stepDecode = 0xE1A00000 #Nop
        self.stepExecute = 0xE1A00000 #Nop
        self.programmCounters = [0, 0]
        
        self.instructionMap = {"dp": self.dp}
    
    # --- Memory ---
    def read(self, address, lenght):
        for device in self.bus:
            if device.mapingBegin <= address and address + lenght <= device.mapingEnd:
                return int.from_bytes(device.read(address, lenght), "little")
        raise EmuError("no device is maped to {add}")
    
    #--- Instructions ---
    def dp(self, instruction):
        opcode = (instruction >> 21) & 0xF
        print(opcode)
    
    #--- Execute instruction ---
    def execute(self):
        instruction = self.instructionMap.get(helper.decode(self.stepExecute))
        if instruction is None:
            raise EmuError(f"Unsuported instruction: {helper.decode(self.stepExecute)}")
        instruction(self.stepExecute)
    
    #--- CPU control ---
    def step(self, disassemble=False):
        pc = self.registers["pc"]
        exeThumb = (self.registers["cpsr"] >> 5) & 1
        self.stepExecute = self.stepDecode
        self.stepDecode = self.stepFetch
        self.stepFetch = self.read(pc, 2 if exeThumb else 4)
        self.programmCounters.append(pc)
        self.registers["pc"] += 2 if exeThumb else 4
        self.execute()
        instr = self.stepExecute
        text = ""
        pcAtInsLoad = self.programmCounters.pop(0)
        if disassemble:
            text = helper.disasm(instr, pcAtInsLoad)
            print(f"{pcAtInsLoad:08x}: {text}")
            
        

if __name__ == "__main__":
    cpu = CPU()
    cpu.bus[0].mountImage("boot_rom")
    while True:
        cpu.step()
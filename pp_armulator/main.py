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
        self.registers = {"R0": 0x0,
                          "R1": 0x0,
                          "R2": 0x0,
                          "R3": 0x0,
                          "pc":0x0,
                          "cpsr":0xD3}
        self.regLookup = {0: "R0",
                          1: "R1",
                          2: "R2",
                          3: "R3",
                          15: "pc"}
        self.registerBanks = {}
        self.flags = {"carry": 0, "overflow": 0}
        #steps
        self.stepFetch = 0xE1A00000 #Nop
        self.stepDecode = 0xE1A00000 #Nop
        self.stepExecute = 0xE1A00000 #Nop
        self.programmCounters = [0, 0]
        
        self.instructionMap = {"dp": self.Idp, "sdt": self.Isdt}
    
    # --- Memory ---
    def read(self, address, lenght):
        for device in self.bus:
            if device.mapingBegin <= address and address + lenght <= device.mapingEnd:
                return int.from_bytes(device.read(address, lenght), "little")
        raise EmuError(f"no device is maped to 0x{address:08x}")
    
    #--- Instructions ---
    def Isdt(self, instruction):
        isPreIndexed = helper.bit(instruction, 24)
        addsOffset = helper.bit(instruction, 23)
        transfersByte = helper.bit(instruction, 22)
        writesBack = helper.bit(instruction, 21)
        isLoad = helper.bit(instruction, 20)
        baseRegister = (instruction >> 16) & 0xF
        dataRegister = (instruction >> 12) & 0xF
        usesRegisterOffset = helper.bit(instruction, 25)
        if usesRegisterOffset: offset = self.shift_reg(instruction)[0]
        else: offset = instruction & 0xFFF
        baseAddress = self.readRegister(baseRegister)
        if addsOffset: updatedAddress = baseAddress + offset
        else: updatedAddress = baseAddress - offset
        updatedAddress &= helper.M32
        memoryAddress = (updatedAddress if isPreIndexed else baseAddress)
        if isLoad:
            if transfersByte: value = self.read(memoryAddress, 1)
            else:
                alignedAddress = memoryAddress & ~3
                rotation = 8 * (memoryAddress & 3)
                value = helper.ror(self.read(alignedAddress, 4), rotation)
        else:
            # Get the value to store.
            value = self.readRegister(dataRegister)
            if dataRegister == 15:value += 4

            # Store a byte.
            if transfersByte: self.write(memoryAddress, 1, value)

            # Store a word.
            else:
                alignedAddress = memoryAddress & ~3
                self.write(aligned_address, 4, value)
        if writesBack or not isPreIndexed: self.setRegister(baseRegister, updatedAddress)
        if isLoad: self.setRegister(dataRegister, value)
    
    def Idp(self, instruction):
        opcode = (instruction >> 21) & 0xF
        updateFlags = helper.bit(instruction, 20)
        firstRegister = (instruction >> 16) & 0xF
        destinationRegister = (instruction >> 12) & 0xF
        operand1 = self.registers[self.regLookup[firstRegister]]
        operand2, shifterCarry = self.shifter(instruction)
        usesImmediateOperand = not helper.bit(instruction, 25)
        carryIn = self.flags["carry"]
        carryOut = 0
        overflow = 0
        arithmeticOperation = opcode in {2, 3, 4, 5, 6, 7, 10, 11}
        match opcode:
            case 0 | 8:
                result = operand1 & operand2
            case 1 | 9:
                result = operand1 ^ operand2
            case 12:
                result = operand1 | operand2
            case 13:
                result = operand2
            case 14:
                result = operand1 & ~operand2 & helper.M32
            case 15:
                result = ~operand2 & helper.M32
            case 2 | 10:
                left = operand1
                right = ~operand2
                extraCarry = 1
            case 3:
                left = operand2
                right = ~operand1
                extraCarry = 1
            case 4 | 11:
                left = operand1
                right = operand2
                extraCarry = 0
            case 5:
                left = operand1
                right = operand2
                extraCarry = carryIn
            case 6:
                left = operand1
                right = ~operand2
                extraCarry = carryIn
            case 7:
                left = operand2
                right = ~operand1
                extraCarry = carryIn
        if arithmeticOperation:
            left &= helper.M32
            right &= helper.M32
            fullResult = left + right + extraCarry
            result = fullResult & helper.M32
            carryOut = fullResult >> 32
            overflow = (((~(left ^ right)) & (left ^ result)) >> 31) & 1
        isTestOperation = 8 <= opcode <= 11
        if updateFlags and (isTestOperation or destinationRegister != 15):
            self.set_nz(result)
            if arithmeticOperation:
                self.flags['carry'] = carryOut
                self.flags["overflow"] = overflow
        else:
            # Logical instructions take C from the barrel shifter.
            self.flags['carry'] = shifterCarry
        if not isTestOperation:
            self.setRegister(destinationRegister, result)
    
    #-- register stuff ---
    def set_nz(self, v, bits=32):
        self.registers['cpsr'] = (self.registers['cpsr'] & ~(1 << 31)) | (((v >> (bits - 1)) & 1) << 31)
        self.registers['cpsr'] = (self.registers['cpsr'] & ~(1 << 30)) | (((int(v == 0)) & 1) << 30)
    
    def setRegister(self, i, v):
        v &= helper.M32
        if i == 15:
            v &= ~3
            self.branched = True
            self.flushPipeline()
        self.registers[self.regLookup[i]] = v
    
    def readRegister(self, regId: int):
        return self.registers[self.regLookup[regId]]
    #--- Helper functions ---
    def shifter(self, i):
        if helper.bit(i, 25):
            rot = ((i >> 8) & 15) * 2
            v = helper.ror(i & 0xFF, rot)
            return v, (v >> 31 if rot else self.flags["carry"])
        return self.shift_reg(i)
    
    def shift_reg(self, i):
        rm, typ = i & 15, (i >> 5) & 3
        val = self.registers[self.regLookup[rm]]
        if i & 0x10:                              # shift amount from register
            if rm == 15:
                val += 4                          # PC reads as addr+12 here
            return helper.do_shift(typ, val, self.registers[self.regLookup[(i >> 8) & 15]] & 0xFF, self.flags['carry'], False)
        return helper.do_shift(typ, val, (i >> 7) & 31, self.flags['carry'], True)
    
    def flushPipeline(self):
        self.stepFetch = 0xE1A00000
        self.stepDecode = 0xE1A00000
        self.stepExecute = 0xE1A00000
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
        text = ""
        instr = self.stepExecute
        pcAtInsLoad = self.programmCounters.pop(0)
        if disassemble:
            text = helper.disasm(instr, pcAtInsLoad)
            print(f"{pcAtInsLoad:08x}: {text}")
        try:
            self.execute()
            self.registers["pc"] += 2 if exeThumb else 4
        except EmuError as e:
            print(f"Emu error: {e}")
            print("pipeline dump:")
            for n in range (2):
                pcAtInsLoad = self.programmCounters.pop(0)
                instr = self.stepExecute
                self.stepExecute = self.stepDecode
                self.stepDecode = None
                print(f"    {pcAtInsLoad:08x}: {text}")
            exit()
        
            
        

if __name__ == "__main__":
    cpu = CPU()
    cpu.bus[0].mountImage("boot_rom")
    while True:
        cpu.step(True)
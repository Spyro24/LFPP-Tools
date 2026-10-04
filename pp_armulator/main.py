import unknownDevice
import nand
import bootRom
import ram
import helper

#CPU Mode Table
MODE_USR = 0b10000 #User
MODE_FIQ = 0b10001
MODE_IRQ = 0b10010
MODE_SVC = 0b10011 #Supervisor
MODE_ABT = 0b10111 #Abort
MODE_UND = 0b11011 #Undefined
MODE_SYS = 0b11111 #System

modeNames = {MODE_SVC: "Supervisor", MODE_FIQ: "FIQ"}

class EmuError(Exception):
    pass

class CPU:
    def __init__(self, debug=False):
        self.debug = debug
        self.mode = MODE_SVC
        self.halted = False
        self.bus = [ram.SDRAM(0x0, 0x00FFFFFF),
                    unknownDevice.unknown(0xc00000c4, 0xc00000cc),
                    unknownDevice.unknown(0xff000000, 0xff000000), #P_CACHE_CTRL
                    unknownDevice.unknown(0xd0100038, 0xd0100038),
                    unknownDevice.unknown(0xd0300000, 0xd03000c0),
                    unknownDevice.unknown(0xd05001fc, 0xd05001fc),
                    unknownDevice.unknown(0xd07023c0, 0xd07023c0),
                    nand.nand(0xD0900000, 0xD09001FF),
                    unknownDevice.unknown(0xd0d00000, 0xd0d00000),
                    ram.ISRAM(0xf8000000, 0xf8009800), #ISRAM (internal_static_ram)
                    #unknownDevice.unknown(0xfffffffc, 0xfffffffc), #Stack (but it cant be the stack)
                    ]
        self.registers = {"R0": 0x0,
                          "R1": 0x0,
                          "R2": 0x0,
                          "R3": 0x0,
                          "R12": 0x0,
                          "pc":0x0,
                          "cpsr":0xD3}
        self.regLookup = {0: "R0",
                          1: "R1",
                          2: "R2",
                          3: "R3",
                          15: "pc"}
        self.registerBanks = {MODE_SVC:{13: 0x0, 14: 0x0}}
        self.flags = {"carry": 0, "overflow": 0}
        #steps
        self.stepFetch = 0xE1A00000 #Nop
        self.stepDecode = 0xE1A00000 #Nop
        self.stepExecute = 0xE1A00000 #Nop
        self.programmCounters = [-1, -1]
        
        self.instructionMap = {"dp": self.Idp, "sdt": self.Isdt, "b": self.Ibranch, "bdt": self.Ibdt,
                               "bx": self.Ibx, "msr": self.Imsr}
    
    # --- Memory ---
    def read(self, address, lenght):
        for device in self.bus:
            if device.mapingBegin <= address <= device.mapingEnd:
                return int.from_bytes(device.read(address, lenght), "little")
        raise EmuError(f"Read: No device is maped to 0x{address:08x}")
    
    def write(self, a, n, v):
        isDevice = False
        for device in self.bus:
            if device.mapingBegin <= a <= device.mapingEnd:
                device.write(a, v)
                isDevice = True
                break
        if not isDevice:
            raise EmuError(f"bus fault writing 0x{a:08x}")
    
    #--- Instructions ---
    def Imsr(self, instruction):
        # Wert bestimmen
        if helper.bit(instruction, 25):
            # Sofortwert mit Rotation
            immediate_value = instruction & 0xFF
            rotation = ((instruction >> 8) & 0x0F) * 2
            value = helper.ror(immediate_value, rotation)
        else:
            # Wert aus einem Register
            register_number = instruction & 0x0F
            value = self.readRegister(register_number)

        # Bestimmen, welche PSR-Felder geschrieben werden
        field_mask = 0

        # Control-Feld: Bits 0–7
        if helper.bit(instruction, 16):
            field_mask |= 0x000000FF

        # Extension-Feld: Bits 8–15
        if helper.bit(instruction, 17):
            field_mask |= 0x0000FF00

        # Status-Feld: Bits 16–23
        if helper.bit(instruction, 18):
            field_mask |= 0x00FF0000

        # Flags-Feld: Bits 24–31
        if helper.bit(instruction, 19):
            field_mask |= 0xFF000000

        # SPSR schreiben
        if helper.bit(instruction, 22):
            self.spsr = (
                (self.spsr & ~field_mask) |
                (value & field_mask)
            )
            return

        # Aktuellen CPSR-Wert aus dem Register holen
        cpsr = self.registers["cpsr"]
        lastMode = cpsr & 0x1F

        # Flags-Feld schreiben
        if field_mask & 0xFF000000:
            cpsr = (
                (cpsr & ~0xFF000000) |
                (value & 0xFF000000)
            )

        # Control-Feld schreiben
        if field_mask & 0x000000FF:
            # Das Thumb-Bit befindet sich in Bit 5.
            # Es darf durch MSR nicht verändert werden.
            old_thumb_bit = cpsr & 0x20

            new_control = value & 0xDF
            cpsr = (
                (cpsr & ~0x000000FF) |
                new_control |
                old_thumb_bit
            )

        # Geänderten CPSR-Wert zurückschreiben
        self.registers["cpsr"] = cpsr
        if self.debug:
            print(f"Mode change: changed {modeNames[lastMode]} to {modeNames[cpsr & 0x1F]}")
    
    def Ibx(self, instruction):
        print(instruction & 15)
        target = self.readRegister(instruction & 15)
        if target & 1: raise EmuError("BX to Thumb state is not supported")
        self.setRegister(15, target)
    
    def Ibdt(self, instruction):
        # Decode instruction flags
        pre_index = helper.bit(instruction, 24)   # P: use pre-indexed addressing
        increment = helper.bit(instruction, 23)   # U: increment instead of decrement
        write_back = helper.bit(instruction, 21)  # W: update the base register
        load = helper.bit(instruction, 20)        # L: load from memory; otherwise store

        # Bits 19–16 specify the base register
        base_register = (instruction >> 16) & 0xF

        # Bits 0–15 specify which registers participate in the transfer
        registers = [register_number for register_number in range(16) if helper.bit(instruction, register_number)]

        register_count = len(registers)
        base_address = self.readRegister(base_register)

        # Calculate the first memory address and the write-back value
        if increment:
            if pre_index:
                address = base_address + 4
            else:
                address = base_address

            write_back_address = base_address + 4 * register_count

        else:
            if pre_index:
                address = base_address - 4 * register_count
            else:
                address = base_address - 4 * register_count + 4

            write_back_address = base_address - 4 * register_count

        # Keep the address within 32 bits
        address &= helper.M32

        # For a load with write-back, update the base register first.
        # If the base register is also in the register list, the loaded
        # value will replace this value later.
        if write_back and load:
            self.setRegister(base_register, write_back_address)

        # Transfer each selected register
        for register_number in registers:
            aligned_address = address & ~3

            if load:
                # Load a value from memory into the register
                value = self.read(aligned_address, 4)
                self.setRegister(register_number, value)

            else:
                # Store the register value in memory.
                # Reading r15 uses its ARM-visible PC value, four bytes ahead.
                value = self.readRegister(register_number)

                if register_number == 15:
                    value += 4

                self.write(aligned_address, 4, value)

            # Move to the next word in memory
            address = (address + 4) & helper.M32

        # For stores, write back after the transfer.
        # This prevents the updated base address from affecting the stored value
        # when the base register is included in the register list.
        if write_back and not load:
            self.setRegister(base_register, write_back_address)
    
    def Ibranch(self, instruction):
        rawOffset = instruction & 0xFFFFFF
        branchOffset = helper.sext(rawOffset, 24) << 2
        linkBitSet = helper.bit(instruction, 24)
        if linkBitSet: self.setRegister(14, (self.readRegister(15) - 4) & helper.M32)
        self.setRegister(15, self.readRegister(15) + branchOffset)
    
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
                self.write(alignedAddress, 4, value)
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

        cpsr = self.registers["cpsr"]
        carryIn = (cpsr >> 29) & 1

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
                result = (operand1 & ~operand2) & helper.M32

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

            carryOut = (fullResult >> 32) & 1
            overflow = (((~(left ^ right)) & (left ^ result)) >> 31) & 1

        isTestOperation = 8 <= opcode <= 11

        if updateFlags and (isTestOperation or destinationRegister != 15):
            # N flag
            if result & 0x80000000:
                cpsr |= (1 << 31)
            else:
                cpsr &= ~(1 << 31)

            # Z flag
            if result == 0:
                cpsr |= (1 << 30)
            else:
                cpsr &= ~(1 << 30)

            if arithmeticOperation:
                # C flag
                if carryOut:
                    cpsr |= (1 << 29)
                else:
                    cpsr &= ~(1 << 29)

                # V flag
                if overflow:
                    cpsr |= (1 << 28)
                else:
                    cpsr &= ~(1 << 28)

            self.registers["cpsr"] = cpsr

        elif updateFlags:
            # Logical instructions take C from the barrel shifter.
            if shifterCarry:
                cpsr |= (1 << 29)
            else:
                cpsr &= ~(1 << 29)

            self.registers["cpsr"] = cpsr

        if not isTestOperation:
            self.setRegister(destinationRegister, result)
    
    #-- register stuff ---
    def set_nz(self, value, bits=32):
        self.registers['cpsr'] = (self.registers['cpsr'] & ~(1 << 31)) | (((value >> (bits - 1)) & 1) << 31)
        self.registers['cpsr'] = (self.registers['cpsr'] & ~(1 << 30)) | (((int(value == 0)) & 1) << 30)
        
    
    def setRegister(self, regId, value):
        value &= helper.M32
        if 0 <= regId <= 7:
            self.registers[f"R{regId}"] = value
        if 8 <= regId <= 12:
            if self.registers['cpsr'] & 0x1F == MODE_FIQ:
                pass
            else:
                pass
        
        if regId in (13, 14):
            self.registerBanks[self.registers['cpsr'] & 0x1F][regId] = value
        if regId == 15:
            value &= ~3
            self.branched = True
            self.flushPipeline()
            self.registers["pc"] = value
    
    def readRegister(self, regId: int):
        if 0 <= regId <= 7:
            return self.registers[f"R{regId}"]
        if 8 <= regId <= 12:
            if self.registers['cpsr'] & 0x1F == MODE_FIQ:
                pass
            else:
                return self.registers[f"R{regId}"]
        if regId in (13, 14):
            return self.registerBanks[self.registers['cpsr'] & 0x1F][regId]
        if regId == 15:
            return self.registers["pc"]
    #--- Helper functions ---
    def shifter(self, i):
        if helper.bit(i, 25):
            rot = ((i >> 8) & 15) * 2
            v = helper.ror(i & 0xFF, rot)
            return v, (v >> 31 if rot else self.flags["carry"])
        return self.shift_reg(i)
    
    def shift_reg(self, i):
        rm, typ = i & 15, (i >> 5) & 3
        val = self.readRegister(rm)
        if i & 0x10:                              # shift amount from register
            if rm == 15:
                val += 4                          # PC reads as addr+12 here
            return helper.do_shift(typ, val, self.registers[self.regLookup[(i >> 8) & 15]] & 0xFF, self.flags['carry'], False)
        return helper.do_shift(typ, val, (i >> 7) & 31, self.flags['carry'], True)
    
    def flushPipeline(self):
        self.stepFetch = 0xE1A00000
        self.stepDecode = 0xE1A00000
        self.stepExecute = 0xE1A00000
        self.programmCounters = [-1, -1]
    
    def checkCondition(self, condCode):
        cpsr = self.registers['cpsr']
        n = (cpsr >> 31) & 1
        z = (cpsr >> 30) & 1
        c = (cpsr >> 29) & 1
        v = (cpsr >> 28) & 1
        
        conditions = {
            0x0: z,                # EQ
            0x1: not z,            # NE
            0x2: c,                # CS
            0x3: not c,            # CC
            0x4: n,                # MI
            0x5: not n,            # PL
            0x6: v,                # VS
            0x7: not v,            # VC
            0x8: c and not z,      # HI
            0x9: not c or z,       # LS
            0xA: n == v,           # GE
            0xB: n != v,           # LT
            0xC: not z and n == v, # GT
            0xD: z or n != v,      # LE
            0xE: True,             # AL
            0xF: False,            # NV
        }
        return conditions.get(condCode)
    #--- Execute instruction ---
    def execute(self):
        instruction = self.instructionMap.get(helper.decode(self.stepExecute))
        if instruction is None:
            raise EmuError(f"Unsuported instruction: {helper.decode(self.stepExecute)}")
        instruction(self.stepExecute)
    
    #--- CPU control ---
    def step(self, disassemble=False):
        self.branched = False
        pc = self.registers["pc"]
        exeThumb = (self.registers["cpsr"] >> 5) & 1
        self.stepExecute = self.stepDecode
        self.stepDecode = self.stepFetch
        self.stepFetch = self.read(pc, 2 if exeThumb else 4)
        self.programmCounters.append(pc)
        text = ""
        instr = self.stepExecute
        pcAtInsLoad = self.programmCounters.pop(0)
        skiped = True
        try:
            if self.checkCondition((self.stepExecute >> 28) & 0xF):
                self.execute()
                skiped = False
            if not self.branched:
                self.registers["pc"] += 2 if exeThumb else 4
            if disassemble:
                if pcAtInsLoad != -1:
                    text = helper.disasm(instr, pcAtInsLoad)
                    print(f"{pcAtInsLoad:08x}: {text}{'; skiped' if skiped else ''}")
        except EmuError as e:
            print(f"Emu error: {e}")
            print("pipeline dump:")
            for n in range (2):
                pcAtInsLoad = self.programmCounters.pop(0)
                instr = self.stepExecute
                self.stepExecute = self.stepDecode
                self.stepDecode = None
                if pcAtInsLoad != -1:
                    text = helper.disasm(instr, pcAtInsLoad)
                    print(f"    {pcAtInsLoad:08x}: {text}")
                else: print("    Pipeline was flushed")
            print("Registers:")
            print(self.registers)
            self.halted = True
        
            
        

if __name__ == "__main__":
    cpu = CPU(debug=True)
    cpu.bus[0].mountImage("boot_rom")
    while not cpu.halted:
        cpu.step(True)
    cpu.bus[-1].dump()
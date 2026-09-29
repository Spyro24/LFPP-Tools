#!/usr/bin/env python3
"""
Simple ARM7TDMI (ARMv4T) emulator that prints a disassembly line for every
instruction it executes.

Supported (ARM state, 32-bit instructions):
  data processing (all 16 ops, all shifter operands, S flag), B/BL/BX,
  LDR/STR/LDRB/STRB, LDRH/STRH/LDRSB/LDRSH, LDM/STM, MUL/MLA,
  UMULL/UMLAL/SMULL/SMLAL, SWI (tiny host interface, see below).

MRS/MSR (single unbanked SPSR), SWI host interface.

Not supported: Thumb state, banked registers/mode switching, SWP, coprocessors,
exceptions, cycle timing.  Unsupported instructions stop the emulation.

Host interface via SWI:  swi 0 = exit,  swi 1 = print r0 as a char,
                         swi 2 = print r0 as a decimal number.

Usage:
  python3 arm7tdmi.py                    # run the built-in demo program
  python3 arm7tdmi.py prog.bin -v        # run a raw ARM binary, show reg changes
  python3 arm7tdmi.py prog.bin --base 0x8000 --max 5000
"""
import argparse
import struct
import sys

M32 = 0xFFFFFFFF
REG = [f"r{i}" for i in range(11)] + ["fp", "ip", "sp", "lr", "pc"]
COND = ["eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc",
        "hi", "ls", "ge", "lt", "gt", "le", "", "nv"]
DPOP = ["and", "eor", "sub", "rsb", "add", "adc", "sbc", "rsc",
        "tst", "teq", "cmp", "cmn", "orr", "mov", "bic", "mvn"]
SHIFT = ["lsl", "lsr", "asr", "ror"]


class EmuError(Exception):
    pass


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def ror(v, n):
    n &= 31
    return v if n == 0 else ((v >> n) | (v << (32 - n))) & M32


def sext(v, bits):
    return (v ^ (1 << (bits - 1))) - (1 << (bits - 1))


def bit(i, n):
    return (i >> n) & 1


def do_shift(typ, val, amt, carry, by_imm):
    """Barrel shifter. Returns (result, carry_out)."""
    if amt == 0 and not (by_imm and typ != 0):
        return val, carry                       # LSL #0 / register shift by 0
    if by_imm and amt == 0:                     # LSR/ASR #0 -> 32, ROR #0 -> RRX
        if typ == 3:
            return ((carry << 31) | (val >> 1)) & M32, val & 1
        amt = 32
    if typ == 0:                                # LSL
        if amt < 32:
            return (val << amt) & M32, (val >> (32 - amt)) & 1
        return 0, (val & 1) if amt == 32 else 0
    if typ == 1:                                # LSR
        if amt < 32:
            return val >> amt, (val >> (amt - 1)) & 1
        return 0, (val >> 31) if amt == 32 else 0
    if typ == 2:                                # ASR
        if amt >= 32:
            return (M32 if val >> 31 else 0), val >> 31
        return (sext(val, 32) >> amt) & M32, (val >> (amt - 1)) & 1
    amt &= 31                                   # ROR
    if amt == 0:
        return val, val >> 31
    return ror(val, amt), (val >> (amt - 1)) & 1


# --------------------------------------------------------------------------
# decoder / disassembler
# --------------------------------------------------------------------------
def decode(i):
    top = (i >> 25) & 7
    if top == 0:
        if (i & 0x0FFFFFF0) == 0x012FFF10:
            return "bx"
        if (i & 0x0FC000F0) == 0x00000090:
            return "mul"
        if (i & 0x0F8000F0) == 0x00800090:
            return "mull"
        if (i & 0x0E000090) == 0x00000090:
            return "hdt" if (i >> 5) & 3 else "swp"
    if top in (0, 1):
        if 8 <= (i >> 21) & 15 <= 11 and not bit(i, 20):
            if (i & 0x0FBF0FFF) == 0x010F0000:
                return "mrs"
            if (i & 0x0FB0FFF0) == 0x0120F000 or (i & 0x0FB0F000) == 0x0320F000:
                return "msr"
            return "psr"
        return "dp"
    if top in (2, 3):
        return "undef" if top == 3 and i & 0x10 else "sdt"
    if top == 4:
        return "bdt"
    if top == 5:
        return "b"
    if top == 7 and bit(i, 24):
        return "swi"
    return "cop"


def imm(v, neg=False):
    return "#" + ("-" if neg else "") + (str(v) if v < 10 else hex(v))


def fmt_shift(i):
    """Register operand with optional shift (bits 11..0)."""
    rm, typ = REG[i & 15], (i >> 5) & 3
    if i & 0x10:
        return f"{rm}, {SHIFT[typ]} {REG[(i >> 8) & 15]}"
    amt = (i >> 7) & 31
    if amt == 0:
        if typ == 0:
            return rm
        if typ == 3:
            return f"{rm}, rrx"
        amt = 32
    return f"{rm}, {SHIFT[typ]} #{amt}"


def fmt_addr(rn, off, pre, wb):
    base = REG[rn] + (f", {off}" if off else "")
    return f"[{base}]" + ("!" if wb else "") if pre else f"[{REG[rn]}]" + (f", {off}" if off else "")


def disasm(i, pc):
    c = COND[i >> 28]
    k = decode(i)
    rd, rn, rs, rm = (i >> 12) & 15, (i >> 16) & 15, (i >> 8) & 15, i & 15

    if i == 0xE1A00000:
        return "nop"

    if k == "b":
        off = sext(i & 0xFFFFFF, 24) << 2
        return f"b{'l' if bit(i, 24) else ''}{c} 0x{(pc + 8 + off) & M32:x}"

    if k == "bx":
        return f"bx{c} {REG[rm]}"

    if k == "swi":
        return f"swi{c} 0x{i & 0xFFFFFF:x}"

    if k == "mul":
        s = "s" if bit(i, 20) else ""
        d = rn                                   # Rd is in bits 19..16 for multiplies
        acc = (i >> 12) & 15
        if bit(i, 21):
            return f"mla{s}{c} {REG[d]}, {REG[rm]}, {REG[rs]}, {REG[acc]}"
        return f"mul{s}{c} {REG[d]}, {REG[rm]}, {REG[rs]}"

    if k == "mull":
        name = ("s" if bit(i, 22) else "u") + ("mlal" if bit(i, 21) else "mull")
        s = "s" if bit(i, 20) else ""
        return f"{name}{s}{c} {REG[rd]}, {REG[rn]}, {REG[rm]}, {REG[rs]}"

    if k == "mrs":
        return f"mrs{c} {REG[rd]}, {'spsr' if bit(i, 22) else 'cpsr'}"

    if k == "msr":
        fields = "".join(ch for ch, b in zip("fsxc", (19, 18, 17, 16)) if bit(i, b))
        src = imm(ror(i & 0xFF, ((i >> 8) & 15) * 2)) if bit(i, 25) else REG[rm]
        return f"msr{c} {'spsr' if bit(i, 22) else 'cpsr'}_{fields}, {src}"

    if k == "dp":
        op, s = (i >> 21) & 15, bit(i, 20)
        if bit(i, 25):
            op2 = imm(ror(i & 0xFF, ((i >> 8) & 15) * 2))
        else:
            op2 = fmt_shift(i)
        name = DPOP[op]
        if 8 <= op <= 11:
            return f"{name}{c} {REG[rn]}, {op2}"
        name += ("s" if s else "") + c
        if op in (13, 15):
            return f"{name} {REG[rd]}, {op2}"
        return f"{name} {REG[rd]}, {REG[rn]}, {op2}"

    if k == "sdt":
        p, u, b, w, l = (bit(i, n) for n in (24, 23, 22, 21, 20))
        name = ("ldr" if l else "str") + ("b" if b else "") + ("t" if not p and w else "") + c
        comment = ""
        if bit(i, 25):
            off = ("" if u else "-") + fmt_shift(i)
        else:
            v = i & 0xFFF
            off = imm(v, not u) if (v or not u) else ""
            if rn == 15 and p:
                comment = f"  ; 0x{(pc + 8 + (v if u else -v)) & M32:x}"
        return f"{name} {REG[rd]}, {fmt_addr(rn, off, p, w)}{comment}"

    if k == "hdt":
        p, u, w, l = (bit(i, n) for n in (24, 23, 21, 20))
        name = ("ldr" if l else "str") + ("h", "sb", "sh")[((i >> 5) & 3) - 1] + c
        if bit(i, 22):
            v = ((i >> 8) & 15) << 4 | (i & 15)
            off = imm(v, not u) if (v or not u) else ""
        else:
            off = ("" if u else "-") + REG[rm]
        return f"{name} {REG[rd]}, {fmt_addr(rn, off, p, w)}"

    if k == "bdt":
        p, u, s, w, l = (bit(i, n) for n in (24, 23, 22, 21, 20))
        regs = ", ".join(REG[r] for r in range(16) if bit(i, r))
        mode = ("ib" if p else "ia") if u else ("db" if p else "da")
        if rn == 13 and w and not l and mode == "db":
            return f"push{c} {{{regs}}}"
        if rn == 13 and w and l and mode == "ia":
            return f"pop{c} {{{regs}}}"
        return f"{'ldm' if l else 'stm'}{mode}{c} {REG[rn]}{'!' if w else ''}, {{{regs}}}{'^' if s else ''}"

    return f".word 0x{i:08x}"


# --------------------------------------------------------------------------
# CPU
# --------------------------------------------------------------------------
class CPU:
    def __init__(self, size=1 << 32):
        self.mem = bytearray(size)
        self.r = [0] * 16
        self.n = self.z = self.c = self.v = 0
        self.psr_ctl = 0xD3                       # mode/I/F/T bits (starts in SVC, IRQ+FIQ masked)
        self.spsr = 0                             # single, unbanked SPSR
        self.halted = False
        self.branched = False
        self.notes = []
        self.output = []

    # ---- memory -----------------------------------------------------------
    def read(self, a, n):
        if a < 0 or a + n > len(self.mem):
            raise EmuError(f"bus fault reading 0x{a:08x}")
        return int.from_bytes(self.mem[a:a + n], "little")

    def write(self, a, n, v):
        if a < 0 or a + n > len(self.mem):
            raise EmuError(f"bus fault writing 0x{a:08x}")
        self.mem[a:a + n] = (v & ((1 << (8 * n)) - 1)).to_bytes(n, "little")

    # ---- registers / flags -----------------------------------------------
    def setr(self, i, v):
        v &= M32
        if i == 15:
            v &= ~3
            self.branched = True
        self.r[i] = v

    def set_nz(self, v, bits=32):
        self.n = (v >> (bits - 1)) & 1
        self.z = int(v == 0)

    def flags_str(self):
        return "".join(ch.upper() if f else ch
                       for ch, f in zip("nzcv", (self.n, self.z, self.c, self.v)))

    def cond_pass(self, c):
        n, z, cf, v = self.n, self.z, self.c, self.v
        return bool([z, not z, cf, not cf, n, not n, v, not v,
                     cf and not z, not cf or z, n == v, n != v,
                     not z and n == v, z or n != v, True, False][c])

    # ---- shifter operand --------------------------------------------------
    def shift_reg(self, i):
        rm, typ = i & 15, (i >> 5) & 3
        val = self.r[rm]
        if i & 0x10:                              # shift amount from register
            if rm == 15:
                val += 4                          # PC reads as addr+12 here
            return do_shift(typ, val, self.r[(i >> 8) & 15] & 0xFF, self.c, False)
        return do_shift(typ, val, (i >> 7) & 31, self.c, True)

    def shifter(self, i):
        if bit(i, 25):
            rot = ((i >> 8) & 15) * 2
            v = ror(i & 0xFF, rot)
            return v, (v >> 31 if rot else self.c)
        return self.shift_reg(i)

    # ---- execute ----------------------------------------------------------
    def x_dp(self, i):
        op, s = (i >> 21) & 15, bit(i, 20)
        rn, rd = (i >> 16) & 15, (i >> 12) & 15
        b, sc = self.shifter(i)
        a = self.r[rn] + (4 if rn == 15 and not bit(i, 25) and i & 0x10 else 0)
        cin = self.c
        cout = ov = 0
        arith = op in (2, 3, 4, 5, 6, 7, 10, 11)
        if op in (0, 8):
            res = a & b
        elif op in (1, 9):
            res = a ^ b
        elif op == 12:
            res = a | b
        elif op == 13:
            res = b
        elif op == 14:
            res = a & ~b & M32
        elif op == 15:
            res = ~b & M32
        else:
            x, y, cy = {2: (a, ~b, 1), 10: (a, ~b, 1), 3: (b, ~a, 1),
                        4: (a, b, 0), 11: (a, b, 0), 5: (a, b, cin),
                        6: (a, ~b, cin), 7: (b, ~a, cin)}[op]
            x, y = x & M32, y & M32
            t = x + y + cy
            res = t & M32
            cout = t >> 32
            ov = ((~(x ^ y)) & (x ^ res)) >> 31 & 1
        testop = 8 <= op <= 11
        if s and (testop or rd != 15):
            self.set_nz(res)
            if arith:
                self.c, self.v = cout, ov
            else:
                self.c = sc
        if not testop:
            self.setr(rd, res)

    def x_branch(self, i):
        off = sext(i & 0xFFFFFF, 24) << 2
        if bit(i, 24):
            self.r[14] = (self.r[15] - 4) & M32   # address of next instruction
        self.setr(15, self.r[15] + off)

    def x_bx(self, i):
        t = self.r[i & 15]
        if t & 1:
            raise EmuError("BX to Thumb state is not supported")
        self.setr(15, t)

    def x_mul(self, i):
        rd, rn, rs, rm = (i >> 16) & 15, (i >> 12) & 15, (i >> 8) & 15, i & 15
        res = self.r[rm] * self.r[rs]
        if bit(i, 21):
            res += self.r[rn]
        res &= M32
        self.setr(rd, res)
        if bit(i, 20):
            self.set_nz(res)

    def x_mull(self, i):
        hi, lo, rs, rm = (i >> 16) & 15, (i >> 12) & 15, (i >> 8) & 15, i & 15
        a, b = self.r[rm], self.r[rs]
        if bit(i, 22):
            a, b = sext(a, 32), sext(b, 32)
        res = a * b
        if bit(i, 21):
            res += (self.r[hi] << 32) | self.r[lo]
        res &= (1 << 64) - 1
        self.setr(lo, res & M32)
        self.setr(hi, res >> 32)
        if bit(i, 20):
            self.set_nz(res, 64)

    def x_sdt(self, i):
        p, u, b, w, l = (bit(i, n) for n in (24, 23, 22, 21, 20))
        rn, rd = (i >> 16) & 15, (i >> 12) & 15
        off = self.shift_reg(i)[0] if bit(i, 25) else i & 0xFFF
        base = self.r[rn]
        ea = (base + off if u else base - off) & M32
        addr = ea if p else base
        if l:
            if b:
                val = self.read(addr, 1)
            else:
                val = ror(self.read(addr & ~3, 4), 8 * (addr & 3))
        else:
            val = self.r[rd] + (4 if rd == 15 else 0)
            if b:
                self.write(addr, 1, val)
            else:
                self.write(addr & ~3, 4, val)
        if w or not p:
            self.setr(rn, ea)
        if l:
            self.setr(rd, val)

    def x_hdt(self, i):
        p, u, w, l = (bit(i, n) for n in (24, 23, 21, 20))
        rn, rd, sh = (i >> 16) & 15, (i >> 12) & 15, (i >> 5) & 3
        if bit(i, 22):
            off = ((i >> 8) & 15) << 4 | (i & 15)
        else:
            off = self.r[i & 15]
        base = self.r[rn]
        ea = (base + off if u else base - off) & M32
        addr = ea if p else base
        if l:
            if sh == 1:
                val = self.read(addr & ~1, 2)
            elif sh == 2:
                val = sext(self.read(addr, 1), 8) & M32
            else:
                val = sext(self.read(addr & ~1, 2), 16) & M32
        else:
            if sh != 1:
                raise EmuError("LDRD/STRD are not part of ARMv4T")
            self.write(addr & ~1, 2, self.r[rd] + (4 if rd == 15 else 0))
        if w or not p:
            self.setr(rn, ea)
        if l:
            self.setr(rd, val)

    def x_bdt(self, i):
        p, u, w, l = (bit(i, n) for n in (24, 23, 21, 20))
        rn = (i >> 16) & 15
        regs = [r for r in range(16) if bit(i, r)]
        n, base = len(regs), self.r[rn]
        if u:
            addr, wb = base + (4 if p else 0), base + 4 * n
        else:
            addr, wb = base - 4 * n + (0 if p else 4), base - 4 * n
        addr &= M32
        if w and l:
            self.setr(rn, wb)                     # loaded value wins if rn in list
        for r in regs:
            if l:
                self.setr(r, self.read(addr & ~3, 4))
            else:
                self.write(addr & ~3, 4, self.r[r] + (4 if r == 15 else 0))
            addr = (addr + 4) & M32
        if w and not l:
            self.setr(rn, wb)

    def cpsr(self):
        return (self.n << 31 | self.z << 30 | self.c << 29 | self.v << 28) | self.psr_ctl

    def x_mrs(self, i):
        self.setr((i >> 12) & 15, self.spsr if bit(i, 22) else self.cpsr())

    def x_msr(self, i):
        if bit(i, 25):
            val = ror(i & 0xFF, ((i >> 8) & 15) * 2)
        else:
            val = self.r[i & 15]
        mask = 0
        for k in range(4):                        # fields c, x, s, f -> bytes 0..3
            if bit(i, 16 + k):
                mask |= 0xFF << (8 * k)
        if bit(i, 22):
            self.spsr = (self.spsr & ~mask) | (val & mask)
            return
        if mask & 0xF0000000:
            self.n, self.z, self.c, self.v = (val >> 31) & 1, (val >> 30) & 1, (val >> 29) & 1, (val >> 28) & 1
        if mask & 0xFF:                           # control byte; T bit can't be changed by MSR
            self.psr_ctl = (val & 0xDF) | (self.psr_ctl & 0x20)

    def x_swi(self, i):
        n = i & 0xFFFFFF
        r0 = self.r[0]
        if n == 0:
            self.halted = True
            self.notes.append("exit")
        elif n == 1:
            self.output.append(chr(r0 & 0xFF))
            self.notes.append(f"output {chr(r0 & 0xFF)!r}")
        elif n == 2:
            self.output.append(f"{r0}\n")
            self.notes.append(f"output {r0}")
        else:
            raise EmuError(f"unhandled SWI 0x{n:x}")

    def execute(self, i):
        k = decode(i)
        h = {"dp": self.x_dp, "b": self.x_branch, "bx": self.x_bx,
             "mul": self.x_mul, "mull": self.x_mull, "sdt": self.x_sdt,
             "hdt": self.x_hdt, "bdt": self.x_bdt, "swi": self.x_swi,
             "mrs": self.x_mrs, "msr": self.x_msr}.get(k)
        if h is None:
            raise EmuError(f"unsupported instruction type '{k}'")
        h(i)

    def step(self, verbose=False):
        pc = self.r[15]
        instr = self.read(pc, 4)
        text = disasm(instr, pc)
        before = self.r[:15]
        self.notes = []
        self.r[15] = pc + 8                       # ARM pipeline: PC reads as addr+8
        self.branched = False
        try:
            if self.cond_pass(instr >> 28):
                self.execute(instr)
            else:
                self.notes.append("skipped")
        except EmuError:
            self.r[15] = pc
            print(f"{pc:08x}:  {instr:08x}  {text}")
            raise
        if not self.branched:
            self.r[15] = pc + 4
        if verbose:
            self.notes += [f"{REG[k]}={self.r[k]:08x}" for k in range(15) if self.r[k] != before[k]]
            self.notes.append(f"[{self.flags_str()}]")
        line = f"{pc:08x}:  {instr:08x}  {text:<30}"
        if self.notes:
            line += " ; " + ", ".join(self.notes)
        print(line.rstrip())


# --------------------------------------------------------------------------
# demo program: sum 1..10, store/load it, call a subroutine that squares it
# --------------------------------------------------------------------------
DEMO = [
    0xE3A00000,  # 00: mov   r0, #0
    0xE3A0100A,  # 04: mov   r1, #10
    0xE0800001,  # 08: add   r0, r0, r1          <- loop
    0xE2511001,  # 0c: subs  r1, r1, #1
    0x1AFFFFFC,  # 10: bne   0x08
    0x13A05001,  # 14: movne r5, #1              (skipped: Z is set)
    0xE3A02C01,  # 18: mov   r2, #0x100
    0xE5820000,  # 1c: str   r0, [r2]
    0xE5923000,  # 20: ldr   r3, [r2]
    0xEB000002,  # 24: bl    0x34
    0xE1A00004,  # 28: mov   r0, r4
    0xEF000002,  # 2c: swi   2                   (print r0)
    0xEF000000,  # 30: swi   0                   (exit)
    0xE0040393,  # 34: mul   r4, r3, r3          <- subroutine
    0xE1A0F00E,  # 38: mov   pc, lr
]


def main():
    ap = argparse.ArgumentParser(description="Simple ARM7TDMI emulator with disassembly trace")
    ap.add_argument("binary", nargs="?", help="raw ARM-state binary (default: built-in demo)")
    ap.add_argument("--base", type=lambda x: int(x, 0), default=0,
                    help="load/entry address (default 0)")
    ap.add_argument("--max", type=int, default=100000, help="max instructions to run")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="also show changed registers and flags")
    args = ap.parse_args()

    if args.binary:
        with open(args.binary, "rb") as f:
            code = f.read()
    else:
        code = struct.pack(f"<{len(DEMO)}I", *DEMO)

    cpu = CPU()
    if args.base + len(code) > len(cpu.mem):
        sys.exit("program does not fit in emulated memory")
    cpu.mem[args.base:args.base + len(code)] = code
    cpu.r[15] = args.base
    cpu.r[13] = len(cpu.mem)                      # stack at top of RAM

    steps = 0
    try:
        while not cpu.halted and steps < args.max:
            cpu.step(args.verbose)
            steps += 1
        if not cpu.halted:
            print(f"\ninstruction limit ({args.max}) reached")
    except EmuError as e:
        print(f"\nemulation stopped: {e}")

    print(f"\n{steps} instructions executed, flags [{cpu.flags_str()}]")
    for row in range(0, 16, 4):
        print("  ".join(f"{REG[k]:>3}={cpu.r[k]:08x}" for k in range(row, row + 4)))
    if cpu.output:
        print("\nprogram output:\n" + "".join(cpu.output))


if __name__ == "__main__":
    main()
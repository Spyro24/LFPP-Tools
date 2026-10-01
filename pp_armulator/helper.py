M32 = 0xFFFFFFFF
REG = [f"r{i}" for i in range(11)] + ["fp", "ip", "sp", "lr", "pc"]
COND = ["eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc",
        "hi", "ls", "ge", "lt", "gt", "le", "", "nv"]
DPOP = ["and", "eor", "sub", "rsb", "add", "adc", "sbc", "rsc",
        "tst", "teq", "cmp", "cmn", "orr", "mov", "bic", "mvn"]
SHIFT = ["lsl", "lsr", "asr", "ror"]

def decode(i):
    top = (i >> 25) & 7
    if top == 0:
        if (i & 0x0FFFFFF0) == 0x012FFF10:
            return "bx"
        if (i & 0x0FC000F0) == 0x00000090:
            return "mul"
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
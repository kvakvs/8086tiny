// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

// 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
// Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) -
// http://www.megalith.co.uk/8086tiny
//
// Revision 1.25
//
// This work is licensed under the MIT License. See included LICENSE.TXT.

#include "8086tiny.h"

#include <memory.h>
#include <stdint.h>
#include <sys/timeb.h>
#include <time.h>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#ifndef NO_GRAPHICS
const uint16_t CPU::cga_colors[4] = {
        0 /* Black */,
        0x1F1F /* Cyan */,
        0xE3E3 /* Magenta */,
        0xFFFF /* White */
};
#endif

int main(int argc, char** argv) {
    auto cpu = CPU::get();
    cpu->main(argc, argv);
}

int CPU::main(int argc, char** argv) {
#ifndef NO_GRAPHICS
    // Initialise SDL
    SDL_Init(SDL_INIT_AUDIO);
    sdl_audio.callback = audio_callback;
#ifdef _WIN32
    sdl_audio.samples = 512;
#endif
    SDL_OpenAudio(&sdl_audio, 0);
#endif

    // regs16 and reg8 point to F000:0, the start of memory-mapped registers. CS
    // is initialised to F000
    regs16 = (uint16_t *) (regs8 = mem + REGS_BASE);
    regs16[REG_CS] = 0xF000;

    // Trap flag off
    regs8[FLAG_TF] = 0;

    // Set DL equal to the boot device: 0 for the FD, or 0x80 for the HD.
    // Normally, boot from the FD.
    // But, if the HD image file is prefixed with @, then boot from the HD
    regs8[REG_DL] = ((argc > 3) && (*argv[3] == '@')) ? argv[3]++, 0x80 : 0;

    // Open BIOS (file id disk[2]), floppy disk image (disk[1]), and hard disk
    // image (disk[0]) if specified
    for (file_index = 3; file_index;) {
        disk[--file_index] = *++argv ? open((const char *) *argv, 32898) : 0;
    }

    // Set CX:AX equal to the hard disk image size, if present
    CAST(unsigned)regs16[REG_AX] = *disk ? lseek(*disk, 0, 2) >> 9 : 0;

    // Load BIOS image into F000:0100, and set IP to 0100
    read(disk[2], regs8 + (reg_ip = 0x100), 0xFF00);

    // Load instruction decoding helper table
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 256; j++) {
            bios_table_lookup[i][j] = regs8[regs16[0x81 + i] + j];
        }
    }

    // Instruction execution loop. Terminates if CS:IP = 0:0
    for (; opcode_stream = mem + 16 * regs16[REG_CS] + reg_ip,
           opcode_stream != mem;) {
        // Set up variables to prepare for decoding an opcode
        set_opcode(*opcode_stream);

        // Extract i_w and i_d fields from instruction
        i_w = (i_reg4bit = raw_opcode_id & 7) & 1;
        i_d = i_reg4bit / 2 & 1;

        // Extract instruction data fields
        i_data0 = CAST(int16_t) opcode_stream[1];
        i_data1 = CAST(int16_t) opcode_stream[2];
        i_data2 = CAST(int16_t) opcode_stream[3];

        // seg_override_en and rep_override_en contain number of instructions to
        // hold segment override and REP prefix respectively
        if (seg_override_en)
            seg_override_en--;
        if (rep_override_en)
            rep_override_en--;

        // i_mod_size > 0 indicates that opcode uses i_mod/i_rm/i_reg, so decode
        // them
        if (i_mod_size) {
            i_mod = (i_data0 & 0xFF) >> 6;
            i_rm = i_data0 & 7;
            i_reg = i_data0 / 8 & 7;

            if ((!i_mod && i_rm == 6) || (i_mod == 2))
                i_data2 = CAST(int16_t) opcode_stream[4];
            else if (i_mod != 1)
                i_data2 = i_data1;
            else  // If i_mod is 1, operand is (usually) 8 bits rather than 16
                // bits
                i_data1 = (int8_t)i_data1;

            DECODE_RM_REG;
        }

        // Instruction execution unit
        switch (xlat_opcode_id) {
            case 0: { // Conditional jump (JAE, JNAE, etc.)
                // i_w is the invert flag, e.g. i_w == 1 means
                // JNAE, whereas i_w == 0 means JAE
                uint8_t tmp = raw_opcode_id / 2 & 7;
                reg_ip +=
                        (int8_t) i_data0 *
                        (i_w ^ (regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_A][tmp]] ||
                                regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_B][tmp]] ||
                                regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_C][tmp]] ^
                                regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_D][tmp]]));
            } break;
            case 1:  // MOV reg, imm
                i_w = !!(raw_opcode_id & 8);
                R_M_OP(mem[GET_REG_ADDR(i_reg4bit)], =, i_data0);
                break;
            case 3:  // PUSH regs16
                R_M_PUSH(regs16[i_reg4bit]);
                break;
            case 4:  // POP regs16
                R_M_POP(regs16[i_reg4bit]);
                break;
            case 2:  // INC|DEC regs16
                i_w = 1;
                i_d = 0;
                i_reg = i_reg4bit;
                DECODE_RM_REG;
                i_reg = extra;  /* fall through */
            case 5:             // INC|DEC|JMP|CALL|PUSH
                if (i_reg < 2)  // INC|DEC
                    MEM_OP(op_from_addr, += 1 - 2 * i_reg +,
                           REGS_BASE + 2 * REG_ZERO)
                    ,
                        op_source = 1, set_AF_OF_arith(),
                        set_OF(op_dest + 1 - i_reg == 1 << (TOP_BIT - 1)),
                        (xlat_opcode_id == 5) &&
                            (set_opcode(0x10), 0);          // Decode like ADC
                else if (i_reg != 6)                        // JMP|CALL
                    i_reg - 3 || R_M_PUSH(regs16[REG_CS]),  // CALL (far)
                        i_reg & 2 &&
                            R_M_PUSH(reg_ip + 2 + i_mod * (i_mod != 3) +
                                     2 * (!i_mod &&
                                          i_rm == 6)),  // CALL (near or far)
                        i_reg & 1 &&
                            (regs16[REG_CS] = CAST(int16_t)
                                 mem[op_from_addr + 2]),  // JMP|CALL (far)
                        R_M_OP(reg_ip, =, mem[op_from_addr]),
                        set_opcode(0x9A);  // Decode like CALL
                else                       // PUSH
                    R_M_PUSH(mem[rm_addr]);
                break;
            case 6:  // TEST r/m, imm16 / NOT|NEG|MUL|IMUL|DIV|IDIV reg
                op_to_addr = op_from_addr;

                switch (i_reg) {
                    ;                      /* fall through */
                    case 0:                // TEST
                        set_opcode(0x20);  // Decode like AND
                        reg_ip += i_w + 1;
                        R_M_OP(mem[op_to_addr], &, i_data2);
                        break;
                    case 2:  // NOT
                        OP(= ~);
                        break;
                    case 3:  // NEG
                        OP(= -);
                        op_dest = 0;
                        set_opcode(0x28);  // Decode like SUB
                        set_CF(op_result > op_dest);
                        break;
                    case 4:  // MUL
                        i_w ? MUL_MACRO(uint16_t, regs16)
                            : MUL_MACRO(uint8_t, regs8);
                        break;
                    case 5:  // IMUL
                        i_w ? MUL_MACRO(int16_t, regs16)
                            : MUL_MACRO(int8_t, regs8);
                        break;
                    case 6:  // DIV
                        i_w ? DIV_MACRO(uint16_t, unsigned, regs16)
                            : DIV_MACRO(uint8_t, uint16_t, regs8);
                        break;
                    case 7:  // IDIV
                        i_w ? DIV_MACRO(int16_t, int, regs16)
                            : DIV_MACRO(int8_t, int16_t, regs8);
                };
                break;
            case 7:  // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP AL/AX, immed
                rm_addr = REGS_BASE;
                i_data2 = i_data0;
                i_mod = 3;
                i_reg = extra;
                reg_ip--;
                ;    /* fall through */
            case 8:  // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP reg, immed
                op_to_addr = rm_addr;
                regs16[REG_SCRATCH] = (i_d |= !i_w) ? (int8_t)i_data2 : i_data2;
                op_from_addr = REGS_BASE + 2 * REG_SCRATCH;
                reg_ip += !i_d + 1;
                set_opcode(0x08 * (extra = i_reg));
                ;    /* fall through */
            case 9:  // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP|MOV reg, r/m
                switch (extra) {
                    ;        /* fall through */
                    case 0:  // ADD
                        OP(+=), set_CF(op_result < op_dest);
                        break;
                    case 1:  // OR
                        OP(|=);
                        break;
                    case 2:  // ADC
                        ADC_SBB_MACRO(+);
                        break;
                    case 3:  // SBB
                        ADC_SBB_MACRO(-);
                        break;
                    case 4:  // AND
                        OP(&=);
                        break;
                    case 5:  // SUB
                        OP(-=), set_CF(op_result > op_dest);
                        break;
                    case 6:  // XOR
                        OP(^=);
                        break;
                    case 7:  // CMP
                        OP(-), set_CF(op_result > op_dest);
                        break;
                    case 8:  // MOV
                        OP(=);
                };
                break;
            case 10:       // MOV sreg, r/m | POP r/m | LEA reg, r/m
                if (!i_w)  // MOV
                    i_w = 1, i_reg += 8, DECODE_RM_REG, OP(=);
                else if (!i_d)  // LEA
                    seg_override_en = 1, seg_override = REG_ZERO, DECODE_RM_REG,
                    R_M_OP(mem[op_from_addr], =, rm_addr);
                else  // POP
                    R_M_POP(mem[rm_addr]);
                break;
            case 11:  // MOV AL/AX, [loc]
                i_mod = i_reg = 0;
                i_rm = 6;
                i_data1 = i_data0;
                DECODE_RM_REG;
                MEM_OP(op_from_addr, =, op_to_addr);
                break;
            case 12:  // ROL|ROR|RCL|RCR|SHL|SHR|???|SAR reg/mem, 1/CL/imm
                      // (80186)
                scratch2_uint = SIGN_OF(mem[rm_addr]),
                scratch_uint = extra ?  // xxx reg/mem, imm
                    ++reg_ip,
                (int8_t)i_data1 :               // xxx reg/mem, CL
                    i_d ? 31 & regs8[REG_CL] :  // xxx reg/mem, 1
                        1;
                if (scratch_uint) {
                    if (i_reg < 4)  // Rotate operations
                        scratch_uint %= i_reg / 2 + TOP_BIT,
                            R_M_OP(scratch2_uint, =, mem[rm_addr]);
                    if (i_reg & 1)  // Rotate/shift right operations
                        R_M_OP(mem[rm_addr], >>=, scratch_uint);
                    else  // Rotate/shift left operations
                        R_M_OP(mem[rm_addr], <<=, scratch_uint);
                    if (i_reg > 3)         // Shift operations
                        set_opcode(0x10);  // Decode like ADC
                    if (i_reg > 4)         // SHR or SAR
                        set_CF(op_dest >> (scratch_uint - 1) & 1);
                }

                switch (i_reg) {
                    ;        /* fall through */
                    case 0:  // ROL
                        R_M_OP(mem[rm_addr], +=,
                               scratch2_uint >> (TOP_BIT - scratch_uint));
                        set_OF(SIGN_OF(op_result) ^ set_CF(op_result & 1));
                        break;
                    case 1:  // ROR
                        scratch2_uint &= (1 << scratch_uint) - 1,
                            R_M_OP(mem[rm_addr], +=,
                                   scratch2_uint << (TOP_BIT - scratch_uint));
                        set_OF(SIGN_OF(op_result * 2) ^
                               set_CF(SIGN_OF(op_result)));
                        break;
                    case 2:  // RCL
                        R_M_OP(mem[rm_addr],
                               += (regs8[FLAG_CF] << (scratch_uint - 1)) +,
                               scratch2_uint >> (1 + TOP_BIT - scratch_uint));
                        set_OF(SIGN_OF(op_result) ^
                               set_CF(scratch2_uint &
                                      1 << (TOP_BIT - scratch_uint)));
                        break;
                    case 3:  // RCR
                        R_M_OP(mem[rm_addr],
                               += (regs8[FLAG_CF] << (TOP_BIT - scratch_uint)) +
                               , scratch2_uint << (1 + TOP_BIT - scratch_uint));
                        set_CF(scratch2_uint & 1 << (scratch_uint - 1));
                        set_OF(SIGN_OF(op_result) ^ SIGN_OF(op_result * 2));
                        break;
                    case 4:  // SHL
                        set_OF(SIGN_OF(op_result) ^
                               set_CF(SIGN_OF(op_dest << (scratch_uint - 1))));
                        break;
                    case 5:  // SHR
                        set_OF(SIGN_OF(op_dest));
                        break;
                    case 7:  // SAR
                        scratch_uint < TOP_BIT || set_CF(scratch2_uint);
                        set_OF(0);
                        R_M_OP(mem[rm_addr], +=,
                               scratch2_uint *=
                               ~(((1 << TOP_BIT) - 1) >> scratch_uint));
                };
                break;
            case 13:  // LOOPxx|JCZX
                scratch_uint = !!--regs16[REG_CX];

                switch (i_reg4bit) {
                    ;        /* fall through */
                    case 0:  // LOOPNZ
                        scratch_uint &= !regs8[FLAG_ZF];
                        break;
                    case 1:  // LOOPZ
                        scratch_uint &= regs8[FLAG_ZF];
                        break;
                    case 3:  // JCXXZ
                        scratch_uint = !++regs16[REG_CX];
                }
                reg_ip += scratch_uint * (int8_t)i_data0;
                break;
            case 14:  // JMP | CALL int16_t/near
                reg_ip += 3 - i_d;
                if (!i_w) {
                    if (i_d)  // JMP far
                        reg_ip = 0, regs16[REG_CS] = i_data2;
                    else  // CALL
                        R_M_PUSH(reg_ip);
                }
                reg_ip += i_d && i_w ? (int8_t)i_data0 : i_data0;
                break;
            case 15:  // TEST reg, r/m
                MEM_OP(op_from_addr, &, op_to_addr);
                break;
            case 16:  // XCHG AX, regs16
                i_w = 1;
                op_to_addr = REGS_BASE;
                op_from_addr = GET_REG_ADDR(i_reg4bit);
                ;     /* fall through */
            case 24:  // NOP|XCHG reg, r/m
                if (op_to_addr != op_from_addr)
                    OP(^=), MEM_OP(op_from_addr, ^=, op_to_addr), OP(^=);
                break;
            case 17:  // MOVSx (extra=0)|STOSx (extra=1)|LODSx (extra=2)
                scratch2_uint = seg_override_en ? seg_override : REG_DS;

                for (scratch_uint = rep_override_en ? regs16[REG_CX] : 1;
                     scratch_uint; scratch_uint--) {
                    MEM_OP(
                        extra < 2 ? SEGREG(REG_ES, REG_DI, ) : REGS_BASE, =,
                        extra & 1 ? REGS_BASE : SEGREG(scratch2_uint, REG_SI, ))
                    , extra & 1 || INDEX_INC(REG_SI),
                        extra & 2 || INDEX_INC(REG_DI);
                }

                if (rep_override_en)
                    regs16[REG_CX] = 0;
                break;
            case 18:  // CMPSx (extra=0)|SCASx (extra=1)
                scratch2_uint = seg_override_en ? seg_override : REG_DS;

                if ((scratch_uint = rep_override_en ? regs16[REG_CX] : 1)) {
                    for (; scratch_uint; rep_override_en || scratch_uint--) {
                        MEM_OP(
                            extra ? REGS_BASE : SEGREG(scratch2_uint, REG_SI, ),
                            -, SEGREG(REG_ES, REG_DI, ))
                        , extra || INDEX_INC(REG_SI), INDEX_INC(REG_DI),
                            rep_override_en &&
                                !(--regs16[REG_CX] &&
                                  (!op_result == rep_mode)) &&
                                (scratch_uint = 0);
                    }

                    set_flags_type =
                        FLAGS_UPDATE_SZP |
                        FLAGS_UPDATE_AO_ARITH;  // Funge to set SZP/AO flags
                    set_CF(op_result > op_dest);
                };
                break;
            case 19:  // RET|RETF|IRET
                i_d = i_w;
                R_M_POP(reg_ip);
                if (extra)  // IRET|RETF|RETF imm16
                    R_M_POP(regs16[REG_CS]);
                if (extra & 2)  // IRET
                    set_flags(R_M_POP(scratch_uint));
                else if (!i_d)  // RET|RETF imm16
                    regs16[REG_SP] += i_data0;
                break;
            case 20:  // MOV r/m, immed
                R_M_OP(mem[op_from_addr], =, i_data2);
                break;
            case 21:                 // IN AL/AX, DX/imm8
                io_ports[0x20] = 0;  // PIC EOI
                io_ports[0x42] =
                    --io_ports[0x40];  // PIT channel 0/2 read placeholder
                io_ports[0x3DA] ^= 9;  // CGA refresh
                scratch_uint = extra ? regs16[REG_DX] : (uint8_t)i_data0;
                scratch_uint == 0x60 &&
                    (io_ports[0x64] = 0);  // Scancode read flag
                scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) &&
                    (io_ports[0x3D5] =
                         ((mem[0x49E] * 80 + mem[0x49D] + CAST(int16_t)
                                                              mem[0x4AD]) &
                          (io_ports[0x3D4] & 1 ? 0xFF : 0xFF00)) >>
                         (io_ports[0x3D4] & 1 ? 0 : 8));  // CRT cursor position
                R_M_OP(regs8[REG_AL], =, io_ports[scratch_uint]);
                ;
                break;
            case 22:  // OUT DX/imm8, AL/AX
                scratch_uint = extra ? regs16[REG_DX] : (uint8_t)i_data0;
                R_M_OP(io_ports[scratch_uint], =, regs8[REG_AL]);
                scratch_uint == 0x61 &&
                    (io_hi_lo = 0,
                     spkr_en |= regs8[REG_AL] & 3);  // Speaker control

                (scratch_uint == 0x40 || scratch_uint == 0x42) &&
                    (io_ports[0x43] & 6) &&
                    (mem[0x469 + scratch_uint - (io_hi_lo ^= 1)] =
                         regs8[REG_AL]);  // PIT rate programming
                
#ifndef NO_GRAPHICS
                scratch_uint == 0x43 &&
                    (io_hi_lo = 0, regs8[REG_AL] >> 6 == 2) &&
                    (SDL_PauseAudio((regs8[REG_AL] & 0xF7) != 0xB6),
                     0);  // Speaker enable
#endif
                scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 6) &&
                    (mem[0x4AD + !(io_ports[0x3D4] & 1)] =
                         regs8[REG_AL]);  // CRT video RAM start offset
                scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) &&
                    (scratch2_uint =
                         ((mem[0x49E] * 80 + mem[0x49D] + CAST(int16_t)
                                                              mem[0x4AD]) &
                          (io_ports[0x3D4] & 1 ? 0xFF00 : 0xFF)) +
                         (regs8[REG_AL] << (io_ports[0x3D4] & 1 ? 0 : 8)) -
                         CAST(int16_t) mem[0x4AD],
                     mem[0x49D] = scratch2_uint % 80,
                     mem[0x49E] = scratch2_uint / 80);  // CRT cursor position
                scratch_uint == 0x3B5 && io_ports[0x3B4] == 1 &&
                    (GRAPHICS_X = regs8[REG_AL] * 16);  // Hercules resolution
                // reprogramming. Defaults
                // are set in the BIOS
                scratch_uint == 0x3B5 && io_ports[0x3B4] == 6 &&
                    (GRAPHICS_Y = regs8[REG_AL] * 4);
                ;
                break;
            case 23:  // REPxx
                rep_override_en = 2;
                rep_mode = i_w;
                seg_override_en&& seg_override_en++;
                break;
            case 25:  // PUSH reg
                R_M_PUSH(regs16[extra]);
                break;
            case 26:  // POP reg
                R_M_POP(regs16[extra]);
                break;
            case 27:  // xS: segment overrides
                seg_override_en = 2;
                seg_override = extra;
                rep_override_en&& rep_override_en++;
                break;
            case 28:  // DAA/DAS
                i_w = 0;
                extra ? DAA_DAS(-=, >=, 0xFF, 0x99)
                      : DAA_DAS(+=, <, 0xF0,
                                0x90)  // extra = 0 for DAA, 1 for DAS
                    ;
                break;
            case 29:  // AAA/AAS
                op_result = AAA_AAS(extra - 1);
                break;
            case 30:  // CBW
                regs8[REG_AH] = -SIGN_OF(regs8[REG_AL]);
                break;
            case 31:  // CWD
                regs16[REG_DX] = -SIGN_OF(regs16[REG_AX]);
                break;
            case 32:  // CALL FAR imm16:imm16
                R_M_PUSH(regs16[REG_CS]);
                R_M_PUSH(reg_ip + 5);
                regs16[REG_CS] = i_data2;
                reg_ip = i_data0;
                break;
            case 33:  // PUSHF
                make_flags();
                R_M_PUSH(scratch_uint);
                break;
            case 34:  // POPF
                set_flags(R_M_POP(scratch_uint));
                break;
            case 35:  // SAHF
                make_flags();
                set_flags((scratch_uint & 0xFF00) + regs8[REG_AH]);
                break;
            case 36:  // LAHF
                make_flags(), regs8[REG_AH] = scratch_uint;
                break;
            case 37:  // LES|LDS reg, r/m
                i_w = i_d = 1;
                DECODE_RM_REG;
                OP(=);
                MEM_OP(REGS_BASE + extra, =, rm_addr + 2);
                break;
            case 38:  // INT 3
                ++reg_ip;
                pc_interrupt(3);
                break;
            case 39:  // INT imm8
                reg_ip += 2;
                pc_interrupt(i_data0);
                break;
            case 40:  // INTO
                ++reg_ip;
                regs8[FLAG_OF] && pc_interrupt(4);
                break;
            case 41:  // AAM
                if (i_data0 &= 0xFF)
                    regs8[REG_AH] = regs8[REG_AL] / i_data0,
                    op_result = regs8[REG_AL] %= i_data0;
                else  // Divide by zero
                    pc_interrupt(0);
                break;
            case 42:  // AAD
                i_w = 0;
                regs16[REG_AX] = op_result =
                    0xFF & regs8[REG_AL] + i_data0 * regs8[REG_AH];
                break;
            case 43:  // SALC
                regs8[REG_AL] = -regs8[FLAG_CF];
                break;
            case 44:  // XLAT
                regs8[REG_AL] =
                    mem[SEGREG(seg_override_en ? seg_override : REG_DS, REG_BX,
                               regs8[REG_AL] +)];
                break;
            case 45:  // CMC
                regs8[FLAG_CF] ^= 1;
                break;
            case 46:  // CLC|STC|CLI|STI|CLD|STD
                regs8[extra / 2] = extra & 1;
                break;
            case 47:  // TEST AL/AX, immed
                R_M_OP(regs8[REG_AL], &, i_data0);
                break;
            case 48:  // Emulator-specific 0F xx opcodes
                switch ((int8_t)i_data0) {
                    case 0:  // PUTCHAR_AL
                        write(1, regs8, 1);
                        break;
                    case 1:  // GET_RTC
                        time(&clock_buf);
                        ftime(&ms_clock);
                        memcpy(mem + SEGREG(REG_ES, REG_BX, ),
                               localtime(&clock_buf), sizeof(struct tm));
                        CAST(int16_t)
                        mem[SEGREG(REG_ES, REG_BX, 36 +)] = ms_clock.millitm;
                        ;
                        break;
                    case 2:  // DISK_READ
                        ;    /* fall through */
                    case 3:  // DISK_WRITE
                        auto ls = ~lseek(disk[regs8[REG_DL]],
                                         CAST(unsigned) regs16[REG_BP] << 9, 0);
                        if (ls) {
                            if ((int8_t)i_data0 == 3) {
                                regs8[REG_AL] = (uint8_t)write(
                                    disk[regs8[REG_DL]],
                                    mem + SEGREG(REG_ES, REG_BX, ),
                                    regs16[REG_AX]);
                            } else {
                                regs8[REG_AL] = (uint8_t)read(
                                    disk[regs8[REG_DL]],
                                    mem + SEGREG(REG_ES, REG_BX, ),
                                    regs16[REG_AX]);
                            }
                        } else {
                            regs8[REG_AL] = 0;
                        }
                }
        }

        // Increment instruction pointer by computed instruction length. Tables
        // in the BIOS binary
        // help us here.
        reg_ip +=
            (i_mod * (i_mod != 3) + 2 * (!i_mod && i_rm == 6)) * i_mod_size +
            bios_table_lookup[TABLE_BASE_INST_SIZE][raw_opcode_id] +
            bios_table_lookup[TABLE_I_W_SIZE][raw_opcode_id] * (i_w + 1);

        // If instruction needs to update SF, ZF and PF, set them as appropriate
        if (set_flags_type & FLAGS_UPDATE_SZP) {
            regs8[FLAG_SF] = SIGN_OF(op_result);
            regs8[FLAG_ZF] = !op_result;
            regs8[FLAG_PF] =
                bios_table_lookup[TABLE_PARITY_FLAG][(uint8_t)op_result];

            // If instruction is an arithmetic or logic operation, also set
            // AF/OF/CF as appropriate.
            if (set_flags_type & FLAGS_UPDATE_AO_ARITH)
                set_AF_OF_arith();
            if (set_flags_type & FLAGS_UPDATE_OC_LOGIC)
                set_CF(0), set_OF(0);
        }

        // Poll timer/keyboard every KEYBOARD_TIMER_UPDATE_DELAY instructions
        if (!(++inst_counter % KEYBOARD_TIMER_UPDATE_DELAY))
            int8_asap = 1;

#ifndef NO_GRAPHICS
        // Update the video graphics display every GRAPHICS_UPDATE_DELAY
        // instructions
        if (!(inst_counter % GRAPHICS_UPDATE_DELAY)) {
            // Video card in graphics mode?
            if (io_ports[0x3B8] & 2) {
                // If we don't already have an SDL window open, set it up and
                // compute color and video memory translation tables
                if (!sdl_screen) {
                    for (int i = 0; i < 16; i++)
                        pixel_colors[i] =
                            mem[0x4AC]
                                ?  // CGA?
                                cga_colors[(i & 12) >> 2] +
                                    (cga_colors[i & 3] << 16)  // CGA -> RGB332
                                : 0xFF *
                                      (((i & 1) << 24) + ((i & 2) << 15) +
                                       ((i & 4) << 6) +
                                       ((i & 8) >> 3));  // Hercules -> RGB332

                    for (int i = 0; i < GRAPHICS_X * GRAPHICS_Y / 4; i++)
                        vid_addr_lookup[i] =
                            i / GRAPHICS_X * (GRAPHICS_X / 8) +
                            (i / 2) % (GRAPHICS_X / 8) +
                            0x2000 * (mem[0x4AC] ? (2 * i / GRAPHICS_X) % 2
                                                 : (4 * i / GRAPHICS_X) % 4);

                    SDL_Init(SDL_INIT_VIDEO);
                    sdl_screen = SDL_SetVideoMode(GRAPHICS_X, GRAPHICS_Y, 8, 0);
                    SDL_EnableUNICODE(1);
                    SDL_EnableKeyRepeat(500, 30);
                }

                // Refresh SDL display from emulated graphics card video RAM
                vid_mem_base = mem + 0xB0000 +
                               0x8000 * (mem[0x4AC] ? 1 : io_ports[0x3B8] >>
                                                              7);  // B800:0 for
                // CGA/Hercules bank 2,
                // B000:0 for Hercules
                // bank 1
                for (int i = 0; i < GRAPHICS_X * GRAPHICS_Y / 4; i++)
                    ((unsigned*)sdl_screen->pixels)[i] =
                        pixel_colors[15 & (vid_mem_base[vid_addr_lookup[i]] >>
                                           4 * !(i & 1))];

                SDL_Flip(sdl_screen);
            } else if (sdl_screen)  // Application has gone back to text mode,
            // so close the SDL window
            {
                SDL_QuitSubSystem(SDL_INIT_VIDEO);
                sdl_screen = 0;
            }
            SDL_PumpEvents();
        }
#endif

        // Application has set trap flag, so fire INT 1
        if (trap_flag)
            pc_interrupt(1);

        trap_flag = regs8[FLAG_TF];

        // If a timer tick is pending, interrupts are enabled, and no
        // overrides/REP are active,
        // then process the tick and check for new keystrokes
        if (int8_asap && !seg_override_en && !rep_override_en &&
            regs8[FLAG_IF] && !regs8[FLAG_TF])
            pc_interrupt(0xA), int8_asap = 0, SDL_KEYBOARD_DRIVER;
    }

#ifndef NO_GRAPHICS
    SDL_Quit();
#endif
    return 0;
}

void CPU::audio_callback(void* data, uint8_t* stream, int len) {
    auto self = get();
    for (int i = 0; i < len; i++)
        stream[i] = (self->spkr_en == 3) && CAST(uint16_t) self->mem[0x4AA]
                        ? -((54 * self->wave_counter++ /
                             CAST(uint16_t) self->mem[0x4AA]) &
                            1)
                        : self->sdl_audio.silence;

    self->spkr_en = self->io_ports[0x61] & 3;
}

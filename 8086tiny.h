// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#pragma once

#include <memory.h>
#include <stdint.h>
#include <sys/timeb.h>
#include <time.h>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#ifndef NO_GRAPHICS
#include <SDL/SDL.h>
#endif

// Emulator system constants
#define IO_PORT_COUNT 0x10000
#define RAM_SIZE 0x10FFF0
#define REGS_BASE 0xF0000
#define VIDEO_RAM_SIZE 0x10000

// Graphics/timer/keyboard update delays (explained later)
#ifndef GRAPHICS_UPDATE_DELAY
#define GRAPHICS_UPDATE_DELAY 360000
#endif
#define KEYBOARD_TIMER_UPDATE_DELAY 20000

// 16-bit register decodes
#define REG_AX 0
#define REG_CX 1
#define REG_DX 2
#define REG_BX 3
#define REG_SP 4
#define REG_BP 5
#define REG_SI 6
#define REG_DI 7

#define REG_ES 8
#define REG_CS 9
#define REG_SS 10
#define REG_DS 11

#define REG_ZERO 12
#define REG_SCRATCH 13

// 8-bit register decodes
#define REG_AL 0
#define REG_AH 1
#define REG_CL 2
#define REG_CH 3
#define REG_DL 4
#define REG_DH 5
#define REG_BL 6
#define REG_BH 7

// FLAGS register decodes
#define FLAG_CF 40
#define FLAG_PF 41
#define FLAG_AF 42
#define FLAG_ZF 43
#define FLAG_SF 44
#define FLAG_TF 45
#define FLAG_IF 46
#define FLAG_DF 47
#define FLAG_OF 48

// Lookup tables in the BIOS binary
#define TABLE_XLAT_OPCODE 8
#define TABLE_XLAT_SUBFUNCTION 9
#define TABLE_STD_FLAGS 10
#define TABLE_PARITY_FLAG 11
#define TABLE_BASE_INST_SIZE 12
#define TABLE_I_W_SIZE 13
#define TABLE_I_MOD_SIZE 14
#define TABLE_COND_JUMP_DECODE_A 15
#define TABLE_COND_JUMP_DECODE_B 16
#define TABLE_COND_JUMP_DECODE_C 17
#define TABLE_COND_JUMP_DECODE_D 18
#define TABLE_FLAGS_BITFIELDS 19

// Bitfields for TABLE_STD_FLAGS values
#define FLAGS_UPDATE_SZP 1
#define FLAGS_UPDATE_AO_ARITH 2
#define FLAGS_UPDATE_OC_LOGIC 4

// Helper macros

// Decode mod, r_m and reg fields in instruction
#define DECODE_RM_REG                                                         \
    scratch2_uint = 4 * !i_mod,                                               \
    op_to_addr = rm_addr =                                                    \
        i_mod < 3                                                             \
            ? SEGREG(                                                         \
                  seg_override_en                                             \
                      ? seg_override                                          \
                      : bios_table_lookup[scratch2_uint + 3][i_rm],           \
                  bios_table_lookup[scratch2_uint][i_rm],                     \
                  regs16[bios_table_lookup[scratch2_uint + 1][i_rm]] +        \
                      bios_table_lookup[scratch2_uint + 2][i_rm] * i_data1 +) \
            : GET_REG_ADDR(i_rm),                                             \
    op_from_addr = GET_REG_ADDR(i_reg),                                       \
    i_d && (scratch_uint = op_from_addr, op_from_addr = rm_addr,              \
            op_to_addr = scratch_uint)

// Return memory-mapped register location (offset into mem array) for register
// #reg_id
#define GET_REG_ADDR(reg_id) \
    (REGS_BASE + (i_w ? 2 * reg_id : 2 * reg_id + reg_id / 4 & 7))

// Returns number of top bit in operand (i.e. 8 for 8-bit operands, 16 for
// 16-bit operands)
#define TOP_BIT 8 * (i_w + 1)


// [I]MUL/[I]DIV/DAA/DAS/ADC/SBB helpers
#define MUL_MACRO(op_data_type, out_regs)                               \
    (set_opcode(0x10),                                                  \
     out_regs[i_w + 1] = (op_result = CAST(op_data_type) mem[rm_addr] * \
                                      (op_data_type)*out_regs) >>       \
                         16,                                            \
     regs16[REG_AX] = op_result,                                        \
     set_OF(set_CF(op_result - (op_data_type)op_result)))
#define DIV_MACRO(out_data_type, in_data_type, out_regs)                    \
    (scratch_int = CAST(out_data_type) mem[rm_addr]) &&                     \
            !(scratch2_uint =                                               \
                  (in_data_type)(scratch_uint = (out_regs[i_w + 1] << 16) + \
                                                regs16[REG_AX]) /           \
                  scratch_int,                                              \
              scratch2_uint - (out_data_type)scratch2_uint)                 \
        ? out_regs[i_w + 1] =                                               \
              scratch_uint - scratch_int * (*out_regs = scratch2_uint)      \
        : pc_interrupt(0)
#define DAA_DAS(op1, op2, mask, min)                                          \
    set_AF((((scratch2_uint = regs8[REG_AL]) & 0x0F) > 9) ||                  \
           regs8[FLAG_AF]) &&                                                 \
        (op_result = regs8[REG_AL] op1 6,                                     \
         set_CF(regs8[FLAG_CF] || (regs8[REG_AL] op2 scratch2_uint))),        \
        set_CF((((mask & 1 ? scratch2_uint : regs8[REG_AL]) & mask) > min) || \
               regs8[FLAG_CF]) &&                                             \
            (op_result = regs8[REG_AL] op1 0x60)
#define ADC_SBB_MACRO(a)                                 \
    OP(a## = regs8[FLAG_CF] +)                           \
    , set_CF(regs8[FLAG_CF] && (op_result == op_dest) || \
             (a op_result < a(int) op_dest)),            \
        set_AF_OF_arith()

// Execute arithmetic/logic operations in emulator memory/registers
#define R_M_OP(dest, op, src)                                           \
    (i_w                                                                \
     ? op_dest = CAST(uint16_t) dest,                                   \
     op_result = CAST(uint16_t) dest op(op_source = CAST(uint16_t) src) \
     : (op_dest = dest, op_result = dest op(op_source = CAST(uint8_t) src)))
#define MEM_OP(dest, op, src) R_M_OP(mem[dest], op, mem[src])
#define OP(op) MEM_OP(op_to_addr, op, op_from_addr)

// Increment or decrement a register #reg_id (usually SI or DI), depending on
// direction flag and operand size (given by i_w)
#define INDEX_INC(reg_id) \
    (regs16[reg_id] -= (2 * regs8[FLAG_DF] - 1) * (i_w + 1))

// Helpers for stack operations
#define R_M_PUSH(a) (i_w = 1, R_M_OP(mem[SEGREG(REG_SS, REG_SP, --)], =, a))
#define R_M_POP(a)                 \
    (i_w = 1, regs16[REG_SP] += 2, \
     R_M_OP(a, =, mem[SEGREG(REG_SS, REG_SP, -2 +)]))

// Convert segment:offset to linear address in emulator memory space
#define SEGREG(reg_seg, reg_ofs, op) \
    16 * regs16[reg_seg] + (uint16_t)(op regs16[reg_ofs])

// Returns sign bit of an 8-bit or 16-bit operand
#define SIGN_OF(a) (1 & (i_w ? CAST(int16_t) a : a) >> (TOP_BIT - 1))

// Reinterpretation cast
#define CAST(a) *(a*)&

// Keyboard driver for console. This may need changing for UNIX/non-UNIX
// platforms
const uint32_t KEYBOARD_OFFSET = 0x4a6;
#ifdef _WIN32
#define KEYBOARD_DRIVER kbhit() && (mem[KEYBOARD_OFFSET] = getch(), pc_interrupt(7))
#else
#define KEYBOARD_DRIVER        \
    read(0, mem + KEYBOARD_OFFSET, 1) && \
        (int8_asap = (mem[KEYBOARD_OFFSET] == 0x1B), pc_interrupt(7))
#endif

// Keyboard driver for SDL
#ifdef NO_GRAPHICS
#define SDL_KEYBOARD_DRIVER KEYBOARD_DRIVER
#else
#define SDL_KEYBOARD_DRIVER                                              \
    sdl_screen                                                           \
        ? SDL_PollEvent(&sdl_event) && (sdl_event.type == SDL_KEYDOWN || \
                                        sdl_event.type == SDL_KEYUP) &&  \
              (scratch_uint = sdl_event.key.keysym.unicode,              \
               scratch2_uint = sdl_event.key.keysym.mod,                 \
               CAST(int16_t) mem[KEYBOARD_OFFSET] =                                \
                   0x400 + 0x800 * !!(scratch2_uint & KMOD_ALT) +        \
                   0x1000 * !!(scratch2_uint & KMOD_SHIFT) +             \
                   0x2000 * !!(scratch2_uint & KMOD_CTRL) +              \
                   0x4000 * (sdl_event.type == SDL_KEYUP) +              \
                   ((!scratch_uint || scratch_uint > 0x7F)               \
                        ? sdl_event.key.keysym.sym                       \
                        : scratch_uint),                                 \
               pc_interrupt(7))                                          \
        : (KEYBOARD_DRIVER)
#endif

class CPU {
   public:
    static CPU* get() {
        static CPU* singleton_ = nullptr;
        if (!singleton_) {
            singleton_ = new CPU();
        }
        return singleton_;
    }

   private:
    // Global variable definitions
    uint8_t mem[RAM_SIZE];
    uint8_t io_ports[IO_PORT_COUNT];
    uint8_t *opcode_stream;
    uint8_t *regs8;
    uint8_t i_rm;
    uint8_t i_w;
    uint8_t i_reg;
    uint8_t i_mod;
    uint8_t i_mod_size;
    uint8_t i_d;
    uint8_t i_reg4bit;
    uint8_t raw_opcode_id;
    uint8_t xlat_opcode_id;
    uint8_t extra;
    uint8_t rep_mode;
    uint8_t seg_override_en;
    uint8_t rep_override_en;
    uint8_t trap_flag;
    uint8_t int8_asap;
    uint8_t io_hi_lo;
    uint8_t *vid_mem_base;
    uint8_t spkr_en;
    uint8_t bios_table_lookup[20][256];

    uint16_t *regs16;
    uint16_t reg_ip;
    uint16_t seg_override;
    uint16_t file_index;
    uint16_t wave_counter;

    uint32_t op_source;
    uint32_t op_dest;
    uint32_t rm_addr;
    uint32_t op_to_addr;
    uint32_t op_from_addr;
    uint32_t i_data0, i_data1, i_data2;
    uint32_t scratch_uint;
    uint32_t scratch2_uint;
    uint32_t inst_counter;
    uint32_t set_flags_type;
    uint32_t GRAPHICS_X;
    uint32_t GRAPHICS_Y;
    uint32_t pixel_colors[16];
//    uint32_t vmem_ctr;
    int op_result;
    int disk[3];
    int scratch_int;
    time_t clock_buf;
    struct timeb ms_clock;

#ifndef NO_GRAPHICS
    SDL_AudioSpec sdl_audio = {44100, AUDIO_U8, 1, 0, 128};
    SDL_Surface* sdl_screen;
    SDL_Event sdl_event;
    uint16_t vid_addr_lookup[VIDEO_RAM_SIZE];

    const static uint16_t cga_colors[4];
#endif

    // Helper functions

    // Set carry flag
    int8_t set_CF(int new_CF) {
        return regs8[FLAG_CF] = (uint8_t) (new_CF != 0);
    }

    // Set auxiliary flag
    int8_t set_AF(int new_AF) {
        return regs8[FLAG_AF] = (uint8_t) (new_AF != 0);
    }

    // Set overflow flag
    int8_t set_OF(int new_OF) {
        return regs8[FLAG_OF] = (uint8_t) (new_OF != 0);
    }

    // Set auxiliary and overflow flag after arithmetic operations
    int8_t set_AF_OF_arith() {
        set_AF((op_source ^= op_dest ^ op_result) & 0x10);
        if (op_result == op_dest)
            return set_OF(0);
        else
            return set_OF(1 & (regs8[FLAG_CF] ^ op_source >> (TOP_BIT - 1)));
    }

    // Assemble and return emulated CPU FLAGS register in scratch_uint
    void make_flags() {
        scratch_uint = 0xF002;  // 8086 has reserved and unused flags set to 1
        for (int i = 9; i--;)
            scratch_uint += regs8[FLAG_CF + i]
                            << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i];
    }

    // Set emulated CPU FLAGS register from regs8[FLAG_xx] values
    void set_flags(int new_flags) {
        for (int i = 9; i--;) {
            auto f = (1 << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i] &
                      new_flags);
            regs8[FLAG_CF + i] = (uint8_t) (f != 0);
        }
    }

        // Convert raw opcode to translated opcode index. This condenses a large
    // number
    // of different encodings of similar
    // instructions into a much smaller number of distinct functions, which we
    // then
    // execute
    void set_opcode(uint8_t opcode) {
        xlat_opcode_id =
            bios_table_lookup[TABLE_XLAT_OPCODE][raw_opcode_id = opcode];
        extra = bios_table_lookup[TABLE_XLAT_SUBFUNCTION][opcode];
        i_mod_size = bios_table_lookup[TABLE_I_MOD_SIZE][opcode];
        set_flags_type = bios_table_lookup[TABLE_STD_FLAGS][opcode];
    }

    // Execute INT #interrupt_num on the emulated machine
    int8_t pc_interrupt(uint8_t interrupt_num) {
        set_opcode(0xCD);  // Decode like INT

        make_flags();
        R_M_PUSH(scratch_uint);
        R_M_PUSH(regs16[REG_CS]);
        R_M_PUSH(reg_ip);
        MEM_OP(REGS_BASE + 2 * REG_CS, =, 4 * interrupt_num + 2);
        R_M_OP(reg_ip, =, mem[4 * interrupt_num]);

        return regs8[FLAG_TF] = regs8[FLAG_IF] = 0;
    }

    // AAA and AAS instructions - which_operation is +1 for AAA, and -1 for AAS
    int AAA_AAS(int8_t which_operation) {
        return (regs16[REG_AX] +=
                262 * which_operation *
                set_AF(set_CF(((regs8[REG_AL] & 0x0F) > 9) || regs8[FLAG_AF])),
                regs8[REG_AL] &= 0x0F);
    }

#ifndef NO_GRAPHICS
    static void audio_callback(void* data, uint8_t* stream, int len);
#endif

    // using WriteFn = int(*)(int, const void *, size_t);
    // using ReadFn = int(*)(int, void *, size_t);

   public:
    // Emulator entry point
    int main(int argc, char** argv);

};  // end CPU

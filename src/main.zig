const std = @import("std");
const root = @import("root");
const builtin = @import("builtin");

const mcu = @import("STM32F7x2/registers.zig");
const regs = mcu.registers;

pub fn sei() void {
    asm volatile ("cpsie i");
}

pub fn cli() void {
    asm volatile ("cpsid i");
}

pub fn enable_fault_irq() void {
    asm volatile ("cpsie f");
}
pub fn disable_fault_irq() void {
    asm volatile ("cpsid f");
}

pub fn nop() void {
    asm volatile ("nop");
}
pub fn wfi() void {
    asm volatile ("wfi");
}
pub fn wfe() void {
    asm volatile ("wfe");
}
pub fn sev() void {
    asm volatile ("sev");
}
pub fn isb() void {
    asm volatile ("isb");
}
pub fn dsb() void {
    asm volatile ("dsb");
}
pub fn dmb() void {
    asm volatile ("dmb");
}
pub fn clrex() void {
    asm volatile ("clrex");
}

extern var __data_start: anyopaque;
extern var __data_end: anyopaque;
extern var __bss_start: anyopaque;
extern var __bss_end: anyopaque;
extern const __data_load_start: anyopaque;

pub fn _start() callconv(.C) noreturn {

    // fill .bss with zeroes
    {
        const bss_start = @ptrCast([*]u8, &__bss_start);
        const bss_end = @ptrCast([*]u8, &__bss_end);
        const bss_len = @ptrToInt(bss_end) - @ptrToInt(bss_start);

        std.mem.set(u8, bss_start[0..bss_len], 0);
    }

    // load .data from flash
    {
        const data_start = @ptrCast([*]u8, &__data_start);
        const data_end = @ptrCast([*]u8, &__data_end);
        const data_len = @ptrToInt(data_end) - @ptrToInt(data_start);
        const data_src = @ptrCast([*]const u8, &__data_load_start);

        std.mem.copy(u8, data_start[0..data_len], data_src[0..data_len]);
    }

    __main();
}

fn isValidField(field_name: []const u8) bool {
    return !std.mem.startsWith(u8, field_name, "reserved") and
        !std.mem.eql(u8, field_name, "initial_stack_pointer") and
        !std.mem.eql(u8, field_name, "reset");
}

pub fn __panic(message: []const u8, maybe_stack_trace: ?*std.builtin.StackTrace) noreturn {

    // utilize logging functions
    std.log.err("PANIC: {s}", .{message});

    if (builtin.cpu.arch != .avr) {
        // var writer = debug.writer();
        // writer.print("PANIC: {s}\r\n", .{message}) catch unreachable;

        if (maybe_stack_trace) |stack_trace| {
            var frame_index: usize = 0;
            var frames_left: usize = std.math.min(stack_trace.index, stack_trace.instruction_addresses.len);
            while (frames_left != 0) : ({
                frames_left -= 1;
                frame_index = (frame_index + 1) % stack_trace.instruction_addresses.len;
            }) {
                const return_address = stack_trace.instruction_addresses[frame_index];
                _ = return_address;
                // writer.print("0x{X:0>8}\r\n", .{return_address}) catch unreachable;
            }
        }
    }
    hang();
}

/// Hangs the processor and will stop doing anything useful. Use with caution!
pub fn hang() noreturn {
    while (true) {
        cli();

        // "this loop has side effects, don't optimize the endless loop away please. thanks!"
        asm volatile ("" ::: "memory");
    }
}

const VectorTable = mcu.VectorTable;

export var vector_table linksection("__flash_start") = VectorTable{
    .initial_stack_pointer = 0x20040000,
    .Reset = .{ .C = _start },
};

export fn __main() noreturn {
    main() catch |err| {
        // TODO:
        // - Compute maximum size on the type of "err"
        // - Do not emit error names when std.builtin.strip is set.
        var msg: [64]u8 = undefined;
        @panic(std.fmt.bufPrint(&msg, "main() returned error {s}", .{@errorName(err)}) catch @panic("main() returned error."));
    };

    // main returned, just hang around here a bit
    hang();
}

pub fn init() anyerror!void {

    // 1. Enable HSE and wait for the HSE to become ready
    regs.RCC.CR.modify(.{ .HSEON = 1 });
    while (regs.RCC.CR.read().HSERDY != 1) {}

    // 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
    regs.RCC.APB1ENR.modify(.{ .PWREN = 1 });
    regs.PWR.CR1.modify(.{ .VOS = 1 });

    // 3. Configurer the FLASH PREFETCH and the LATENCY related settings
    regs.FLASH.ACR.modify(.{ .ARTEN = 1, .PRFTEN = 1, .LATENCY = 3 });

    // 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
    regs.RCC.CFGR.modify(.{
        .HPRE = 0, // DIV 1
        .PPRE1 = 5, // DIV 4
        .PPRE2 = 4, // DIV 2
    });

    regs.RCC.CR.modify(.{ .PLLON = 0 });
    while (regs.RCC.CR.read().PLLRDY != 0) {}

    regs.RCC.CIR.raw = 0;
    // 5. Configure the MAIN PLL
    // #define PLL_M     8
    // #define PLL_N     432
    // #define PLL_P     RCC_PLLP_DIV2 /* 2 */
    // #define PLL_Q     9

    // #define PLL_SAIN  384
    // #define PLL_SAIQ  7
    // #define PLL_SAIP  RCC_PLLSAIP_DIV8

    regs.RCC.PLLCFGR.modify(.{
        .PLLSRC = 1, // HSE
        .PLLM = 8,
        .PLLN = 432,
        .PLLP = 0, // div 2
        .PLLQ = 9,
    });

    // 6. Enable the PLL and wait for it to become ready
    regs.RCC.CR.modify(.{ .PLLON = 1 });
    while (regs.RCC.CR.read().PLLRDY != 1) {}

    // 7. Select the clock source and waoit for it to be set
    regs.RCC.CFGR.modify(.{ .SW = 2 }); // System clock use PLL
    while (regs.RCC.CFGR.read().SWS != 2) {}

    // enable instruction cache
    regs.SCB.CCR.modify(.{ .IC = 1, .DC = 1 });
}

pub fn main() anyerror!void {
    try init();

    // const led_pin = microzig.Pin("PC13");

    // const led = microzig.Gpio(led_pin, .{ .mode = .output, .initial_state = .low });

    // led.init();

    while (true) {
        // rtt.write("Hello world!\n");
        // led.toggle();
        delay(1000);
    }
}

pub fn delay(ms: u32) void {
    // CPU run at 16mHz on HSI16
    // each tick is 5 instructions (1000 * 16 / 5) = 3200
    var ticks = ms * (1000 * 16 / 5);
    var i: u32 = 0;
    // One loop is 5 instructions
    while (i < ticks) {
        nop();
        i += 1;
    }
}

const std = @import("std");
const root = @import("root");
const builtin = @import("builtin");
const arm = @import("arm.zig");
const arch = arm.Arm;

const mcu = @import("STM32F7x2/stm32f7x2.zig");
const r = mcu.registers;


pub fn SysTick_Handler() callconv(.C) void {
    arch.nop();
} //@panic("SysTick"); }
pub fn HardFault_Handler() callconv(.C) void {
    arch.nop();
} //@panic("HardFault"); }
pub fn BusFault_Handler() callconv(.C) void {
    arch.nop();
} //@panic("BusFault"); }
pub fn RCC_Handler() callconv(.C) void {
    arch.nop();
} //@panic("RCC"); }

const core = @import("core.zig").Core(arch, main);
export fn __start() callconv(.C) noreturn {
    core.start();
}


export var vector_table linksection("__flash_start") = mcu.VectorTable{
    .Reset = .{ .C = __start },
    .SysTick = .{ .C = SysTick_Handler },
    .HardFault = .{ .C = HardFault_Handler },
    .BusFault = .{ .C = BusFault_Handler },
    .RCC = .{ .C = RCC_Handler },
};



// ================ application code ============================
pub const cpu_frequency = 216_000_000;

pub const pin_map = .{
    .@"SWD" = "PA13",
    .@"SWCLK" = "PA14",

    // Status LED
    .@"LED" = "PC13",
};



pub fn init() anyerror!void {

    //regs.RCC.CR.reset();
// 1. Enable HSE and wait for the HSE to become ready
    var CR = r.RCC.CR.read();
    
    while (CR.HSERDY != 1) {
        r.RCC.CR.modify(.{.HSEON=1});
        CR = r.RCC.CR.read();
    }

// 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
    r.RCC.APB1ENR.modify(.{.PWREN = 1});
    r.PWR.CR1.modify(.{.VOS = 1});

// 3. Configure the FLASH PREFETCH and the LATENCY related settings
    r.FLASH.ACR.modify(.{
        .ARTEN = 1, 
        .PRFTEN = 1, 
        .LATENCY = 7
        });

// 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
    r.RCC.CFGR.modify(.{
        .HPRE = 0, // DIV 1
        .PPRE1 = 5, // DIV 4
        .PPRE2 = 4
     }); // DIV 2

    // CR.PLLON = 0;
    // while (CR.PLLRDY != 0) {}

    // regs.RCC.CIR.* = 0;
// 5. Configure the MAIN PLL
    // #define PLL_M     8
    // #define PLL_N     432
    // #define PLL_P     RCC_PLLP_DIV2 /* 2 */
    // #define PLL_Q     9

    // #define PLL_SAIN  384
    // #define PLL_SAIQ  7
    // #define PLL_SAIP  RCC_PLLSAIP_DIV8

    r.RCC.PLLCFGR.modify(.{
        .PLLSRC = 1, // HSE
        .PLLM = 8,
        .PLLN = 432,
        .PLLP = 0, // div 2
        .PLLQ = 9
        });

// 6. Enable the PLL and wait for it to become ready
    // CR.PLLON = 1;
    // while (CR.PLLRDY != 1) {}

    r.PWR.CR1.modify(.{.ODEN=1});
    while (r.PWR.CSR1.read().ODRDY != 1) {}

    // regs.RCC.CR.HSION = 0;

// 7. Select the clock source and waoit for it to be set
    // regs.RCC.CFGR.SW = 2; // System clock use PLL
    // while (regs.RCC.CFG.SWS != 2) {}


// enable instruction and data cache
    r.SCB.CCR.modify(.{.IC=1, .DC=1});
}

pub fn main() anyerror!void {
    
    try init();

    // const led_pin = microzig.Pin("PC13");

    // const led_pin = mcu.parsePin(pin_map.LED);

    // enable gpio port c
    r.RCC.AHB1ENR.modify(.{.GPIOCEN=1});

    // set PC13 output
    r.GPIOC.MODER.modify(.{.MODER13 = 1});

    // mcu.gpio.setOutput(led_pin);
    // mcu.gpio.write(led_pin, 1);
    // const led = microzig.Gpio(led_pin, .{ .mode = .output, .initial_state = .low });

    var value:u1 = 1;
    // led.init();

    while (true) {
        // rtt.write("Hello world!\n");
        // led.toggle();
        // mcu.gpio.write(led_pin, value);

        if(value == 0)
            r.GPIOC.BSRR.modify(.{.BR13 = 0});

        if(value == 1)
            r.GPIOC.BSRR.modify(.{.BR13 = 1});

        value = ~value;
        delay(1000);
    }
}

pub fn delay(ms: u32) void {
    // CPU run at 16mHz on HSI16
    // each tick is 5 instructions (1000 * 16 / 5) = 3200
    var ticks = ms * (1000 *  16 / 8);
    var i: u32 = 0;
    // One loop is 5 instructions
    while (i < ticks) {
        arch.nop();
        i += 1;
    }
}

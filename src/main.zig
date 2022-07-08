const std = @import("std");
const root = @import("root");
const builtin = @import("builtin");
const arm = @import("arm.zig");
const arch = arm.Arm;

const mcu = @import("STM32F7x2/stm32f7x2.zig");
const regs = mcu.registers;


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
export const start = core.start;


export var vector_table linksection("__flash_start") = mcu.VectorTable{
    // .initial_stack_pointer = 0x20040000,
     .Reset = .{ .C = core.start },
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
// 1. Enable HSE and wait for the HSE to become ready
    regs.RCC.CR.HSEON = 1;
    while (regs.RCC.CR.HSERDY != 1) {}

// 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
    regs.RCC.APB1ENR.PWREN = 1;
    regs.PWR.CR1.VOS = 1;

// 3. Configure the FLASH PREFETCH and the LATENCY related settings
    regs.FLASH.ACR.ARTEN = 1;
    regs.FLASH.ACR.PRFTEN = 1;
    regs.FLASH.ACR.LATENCY = 7;

// 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
    regs.RCC.CFGR.HPRE = 0; // DIV 1
    regs.RCC.CFGR.PPRE1 = 5; // DIV 4
    regs.RCC.CFGR.PPRE2 = 4; // DIV 2

    regs.RCC.CR.PLLON = 0;
    while (regs.RCC.CR.PLLRDY != 0) {}

    // regs.RCC.CIR.* = 0;
// 5. Configure the MAIN PLL
    // #define PLL_M     8
    // #define PLL_N     432
    // #define PLL_P     RCC_PLLP_DIV2 /* 2 */
    // #define PLL_Q     9

    // #define PLL_SAIN  384
    // #define PLL_SAIQ  7
    // #define PLL_SAIP  RCC_PLLSAIP_DIV8

    regs.RCC.PLLCFGR.PLLSRC = 1; // HSE
    regs.RCC.PLLCFGR.PLLM = 8;
    regs.RCC.PLLCFGR.PLLN = 432;
    regs.RCC.PLLCFGR.PLLP = 0; // div 2
    regs.RCC.PLLCFGR.PLLQ = 9;

// 6. Enable the PLL and wait for it to become ready
    regs.RCC.CR.PLLON = 1;
    while (regs.RCC.CR.PLLRDY != 1) {}

    regs.PWR.CR1.ODEN = 1;
    while (regs.PWR.CSR1.ODRDY != 1) {}

    // regs.RCC.CR.HSION = 0;

// 7. Select the clock source and waoit for it to be set
    // regs.RCC.CFGR.SW = 2; // System clock use PLL
    // while (regs.RCC.CFGR.SWS != 2) {}


// enable instruction and data cache
    regs.SCB.CCR.IC = 1;
    regs.SCB.CCR.DC = 1;
}

pub fn main() anyerror!void {
    {
        try init();
    }

    // const led_pin = microzig.Pin("PC13");

    // const led_pin = mcu.parsePin(pin_map.LED);

    regs.RCC.AHB1ENR.GPIOCEN=1;
    regs.GPIOC.MODER.MODER13 = 1;

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
            regs.GPIOC.BSRR.BR13 = 1;

        if(value == 1)
            regs.GPIOC.BSRR.BS13 = 1;
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

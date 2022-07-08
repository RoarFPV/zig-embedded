//! For now we keep all clock settings on the chip defaults.
//! This code should work with all the STM32F7x2 line
//!
//! Specifically, TIM6 is running on a 16 MHz clock,
//! HSI = 16 MHz is the SYSCLK after reset
//! default AHB prescaler = /1 (= values 0..7):
//!
//! ```
//! regs.RCC.CFGR.modify(.{ .HPRE = 0 });
//! ```
//!
//! so also HCLK = 16 MHz.
//! And with the default APB1 prescaler = /1:
//!
//! ```
//! regs.RCC.CFGR.modify(.{ .PPRE1 = 0 });
//! ```
//!
//! results in PCLK1 = 16 MHz.
//!
//! TODO: add more clock calculations when adding Uart

const std = @import("std");
// const micro = @import("microzig");
const chip = @import("registers.zig");
const regs = chip.registers;

pub usingnamespace chip;

pub fn parsePin(comptime spec: []const u8) type {
    const invalid_format_msg = "The given pin '" ++ spec ++ "' has an invalid format. Pins must follow the format \"P{Port}{Pin}\" scheme.";

    if (spec[0] != 'P')
        @compileError(invalid_format_msg);
    if (spec[1] < 'A' or spec[1] > 'K')
        @compileError(invalid_format_msg);

    const pin_number: comptime_int = std.fmt.parseInt(u4, spec[2..], 10) catch @compileError(invalid_format_msg);

    return struct {
        /// 'A'...'K'
        const gpio_port_name = spec[1..2];
        const gpio_port = @field(regs, "GPIO" ++ gpio_port_name);
        const suffix = std.fmt.comptimePrint("{d}", .{pin_number});
    };
}

fn setRegField(reg: anytype, comptime field_name: anytype, value: anytype) void {
    @field(reg, field_name) = value;
}

pub const gpio = struct {
    pub fn setOutput(comptime pin: type) void {
        setRegField(regs.RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
        setRegField(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b01);
    }

    pub fn setInput(comptime pin: type) void {
        setRegField(regs.RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
        setRegField(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b00);
    }

    pub fn read(comptime pin: type) u1 {
        const idr_reg = pin.gpio_port.IDR;
        const reg_value = @field(idr_reg, "IDR" ++ pin.suffix); // TODO extract to getRegField()?
        return @intCast(u1, reg_value);
    }

    pub fn write(comptime pin: type, state: u1) void {
        _ = pin;
        switch (state) {
            0 => setRegField(pin.gpio_port.BSRR, "BR" ++ pin.suffix, 1),
            1 => setRegField(pin.gpio_port.BSRR, "BS" ++ pin.suffix, 1),
        }
    }
};

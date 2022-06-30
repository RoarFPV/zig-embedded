pub const chip = @import("chip");
pub const micro = @import("microzig");

pub const cpu_frequency = 216_000_000;

pub const pin_map = .{
    .@"SWD" = "PA13",
    .@"SWCLK" = "PA14",

    // Status LED
    .@"LED" = "PC13",
};

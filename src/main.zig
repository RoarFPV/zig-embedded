const std = @import("std");
const root = @import("root");
const builtin = @import("builtin");
const arm = @import("arm.zig");
const arch = arm.Arm;

const mcu = @import("STM32F7x2/registers.zig");
const regs = mcu.registers;

extern var __data_start: anyopaque;
extern var __data_end: anyopaque;
extern var __bss_start: anyopaque;
extern var __bss_end: anyopaque;
extern const __data_load_start: anyopaque;
extern var __stack_end: u32;





pub fn _start() callconv(.C) noreturn {

    // fill .bss with zeroes
    
    const bss_start = @ptrCast([*]u8, &__bss_start);
    const bss_end = @ptrCast([*]u8, &__bss_end);
    arch.Program.initBss(bss_start, bss_end);

    // load .data from flash
    const data_start = @ptrCast([*]u8, &__data_start);
    const data_end = @ptrCast([*]u8, &__data_end);
    const data_src = @ptrCast([*]const u8, &__data_load_start);
    arch.Program.copyData(data_src, data_start, data_end);

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

pub fn __panic(message: []const u8, maybe_stack_trace: ?*std.builtin.StackTrace) noreturn {

    // utilize logging functions
    std.log.err("PANIC: {s}", .{message});

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

pub fn SysTick_Handler() callconv(.C) void {
    nop();
} //@panic("SysTick"); }
pub fn HardFault_Handler() callconv(.C) void {
    nop();
} //@panic("HardFault"); }
pub fn BusFault_Handler() callconv(.C) void {
    nop();
} //@panic("BusFault"); }
pub fn RCC_Handler() callconv(.C) void {
    nop();
} //@panic("RCC"); }

const VectorTable = mcu.VectorTable;

export var vector_table linksection("__flash_start") = VectorTable{
    .initial_stack_pointer = 0x20040000,
    .Reset = .{ .C = _start },
    .SysTick = .{ .C = SysTick_Handler },
    .HardFault = .{ .C = HardFault_Handler },
    .BusFault = .{ .C = BusFault_Handler },
    .RCC = .{ .C = RCC_Handler },
};

export fn __main() noreturn {
    
}

// ================ application code ============================

pub const RCC = struct {
    pub const base_address = 0x40023800;

    /// address: 0x40023800
    /// clock control register
    pub const CR = @intToPtr(*volatile packed struct {
        /// Internal high-speed clock
        /// enable
        HSION: u1,
        /// Internal high-speed clock ready
        /// flag
        HSIRDY: u1,
        reserved0: u1,
        /// Internal high-speed clock
        /// trimming
        HSITRIM: u5,
        /// Internal high-speed clock
        /// calibration
        HSICAL: u8,
        /// HSE clock enable
        HSEON: u1,
        /// HSE clock ready flag
        HSERDY: u1,
        /// HSE clock bypass
        HSEBYP: u1,
        /// Clock security system
        /// enable
        CSSON: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        reserved4: u1,
        /// Main PLL (PLL) enable
        PLLON: u1,
        /// Main PLL (PLL) clock ready
        /// flag
        PLLRDY: u1,
        /// PLLI2S enable
        PLLI2SON: u1,
        /// PLLI2S clock ready flag
        PLLI2SRDY: u1,
        /// PLLSAI enable
        PLLSAION: u1,
        /// PLLSAI clock ready flag
        PLLSAIRDY: u1,
        padding0: u1,
        padding1: u1,
    }, base_address + 0x0);

    /// address: 0x40023804
    /// PLL configuration register
    pub const PLLCFGR = @intToPtr(*volatile packed struct {
        /// Division factor for the main PLL (PLL)
        /// and audio PLL (PLLI2S) input clock
        PLLM: u6,

        /// Main PLL (PLL) multiplication factor for
        /// VCO
        PLLN: u9,

        reserved0: u1,
        /// Main PLL (PLL) division factor for main
        /// system clock
        PLLP: u2,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        reserved4: u1,
        /// Main PLL(PLL) and audio PLL (PLLI2S)
        /// entry clock source
        PLLSRC: u1,
        reserved5: u1,
        /// Main PLL (PLL) division factor for USB
        /// OTG FS, SDIO and random number generator
        /// clocks
        PLLQ: u4,

        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
    }, base_address + 0x4);

    /// address: 0x40023808
    /// clock configuration register
    pub const CFGR = @intToPtr(*volatile packed struct {
        /// System clock switch
        SW: u2,
        /// System clock switch status
        SWS: u2,
        /// AHB prescaler
        HPRE: u4,
        reserved0: u1,
        reserved1: u1,
        /// APB Low speed prescaler
        /// (APB1)
        PPRE1: u3,
        /// APB high-speed prescaler
        /// (APB2)
        PPRE2: u3,
        /// HSE division factor for RTC
        /// clock
        RTCPRE: u5,
        /// Microcontroller clock output
        /// 1
        MCO1: u2,
        /// I2S clock selection
        I2SSRC: u1,
        /// MCO1 prescaler
        MCO1PRE: u3,
        /// MCO2 prescaler
        MCO2PRE: u3,
        /// Microcontroller clock output
        /// 2
        MCO2: u2,
    }, base_address + 0x8);

    /// address: 0x4002380c
    /// clock interrupt register
    pub const CIR = @intToPtr(*volatile packed struct {
        /// LSI ready interrupt flag
        LSIRDYF: u1,
        /// LSE ready interrupt flag
        LSERDYF: u1,
        /// HSI ready interrupt flag
        HSIRDYF: u1,
        /// HSE ready interrupt flag
        HSERDYF: u1,
        /// Main PLL (PLL) ready interrupt
        /// flag
        PLLRDYF: u1,
        /// PLLI2S ready interrupt
        /// flag
        PLLI2SRDYF: u1,
        /// PLLSAI ready interrupt
        /// flag
        PLLSAIRDYF: u1,
        /// Clock security system interrupt
        /// flag
        CSSF: u1,
        /// LSI ready interrupt enable
        LSIRDYIE: u1,
        /// LSE ready interrupt enable
        LSERDYIE: u1,
        /// HSI ready interrupt enable
        HSIRDYIE: u1,
        /// HSE ready interrupt enable
        HSERDYIE: u1,
        /// Main PLL (PLL) ready interrupt
        /// enable
        PLLRDYIE: u1,
        /// PLLI2S ready interrupt
        /// enable
        PLLI2SRDYIE: u1,
        /// PLLSAI Ready Interrupt
        /// Enable
        PLLSAIRDYIE: u1,
        reserved0: u1,
        /// LSI ready interrupt clear
        LSIRDYC: u1,
        /// LSE ready interrupt clear
        LSERDYC: u1,
        /// HSI ready interrupt clear
        HSIRDYC: u1,
        /// HSE ready interrupt clear
        HSERDYC: u1,
        /// Main PLL(PLL) ready interrupt
        /// clear
        PLLRDYC: u1,
        /// PLLI2S ready interrupt
        /// clear
        PLLI2SRDYC: u1,
        /// PLLSAI Ready Interrupt
        /// Clear
        PLLSAIRDYC: u1,
        /// Clock security system interrupt
        /// clear
        CSSC: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
    }, base_address + 0xc);

    /// address: 0x40023810
    /// AHB1 peripheral reset register
    pub const AHB1RSTR = @intToPtr(*volatile packed struct {
        /// IO port A reset
        GPIOARST: u1,
        /// IO port B reset
        GPIOBRST: u1,
        /// IO port C reset
        GPIOCRST: u1,
        /// IO port D reset
        GPIODRST: u1,
        /// IO port E reset
        GPIOERST: u1,
        /// IO port F reset
        GPIOFRST: u1,
        /// IO port G reset
        GPIOGRST: u1,
        /// IO port H reset
        GPIOHRST: u1,
        /// IO port I reset
        GPIOIRST: u1,
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        /// CRC reset
        CRCRST: u1,
        reserved3: u1,
        reserved4: u1,
        reserved5: u1,
        reserved6: u1,
        reserved7: u1,
        reserved8: u1,
        reserved9: u1,
        reserved10: u1,
        /// DMA2 reset
        DMA1RST: u1,
        /// DMA2 reset
        DMA2RST: u1,
        reserved11: u1,
        reserved12: u1,
        reserved13: u1,
        reserved14: u1,
        reserved15: u1,
        reserved16: u1,
        /// USB OTG HS module reset
        OTGHSRST: u1,
        padding0: u1,
        padding1: u1,
    }, base_address + 0x10);

    /// address: 0x40023814
    /// AHB2 peripheral reset register
    pub const AHB2RSTR = @intToPtr(*volatile packed struct {
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        /// AES module reset
        AESRST: u1,
        reserved4: u1,
        /// Random number generator module
        /// reset
        RNGRST: u1,
        /// USB OTG FS module reset
        OTGFSRST: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
        padding8: u1,
        padding9: u1,
        padding10: u1,
        padding11: u1,
        padding12: u1,
        padding13: u1,
        padding14: u1,
        padding15: u1,
        padding16: u1,
        padding17: u1,
        padding18: u1,
        padding19: u1,
        padding20: u1,
        padding21: u1,
        padding22: u1,
        padding23: u1,
    }, base_address + 0x14);

    /// address: 0x40023818
    /// AHB3 peripheral reset register
    pub const AHB3RSTR = @intToPtr(*volatile packed struct {
        /// Flexible memory controller module
        /// reset
        FMCRST: u1,
        /// Quad SPI memory controller
        /// reset
        QSPIRST: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
        padding8: u1,
        padding9: u1,
        padding10: u1,
        padding11: u1,
        padding12: u1,
        padding13: u1,
        padding14: u1,
        padding15: u1,
        padding16: u1,
        padding17: u1,
        padding18: u1,
        padding19: u1,
        padding20: u1,
        padding21: u1,
        padding22: u1,
        padding23: u1,
        padding24: u1,
        padding25: u1,
        padding26: u1,
        padding27: u1,
        padding28: u1,
        padding29: u1,
    }, base_address + 0x18);

    /// address: 0x40023820
    /// APB1 peripheral reset register
    pub const APB1RSTR = @intToPtr(*volatile packed struct {
        /// TIM2 reset
        TIM2RST: u1,
        /// TIM3 reset
        TIM3RST: u1,
        /// TIM4 reset
        TIM4RST: u1,
        /// TIM5 reset
        TIM5RST: u1,
        /// TIM6 reset
        TIM6RST: u1,
        /// TIM7 reset
        TIM7RST: u1,
        /// TIM12 reset
        TIM12RST: u1,
        /// TIM13 reset
        TIM13RST: u1,
        /// TIM14 reset
        TIM14RST: u1,
        /// Low power timer 1 reset
        LPTIM1RST: u1,
        reserved0: u1,
        /// Window watchdog reset
        WWDGRST: u1,
        reserved1: u1,
        reserved2: u1,
        /// SPI 2 reset
        SPI2RST: u1,
        /// SPI 3 reset
        SPI3RST: u1,
        reserved3: u1,
        /// USART 2 reset
        UART2RST: u1,
        /// USART 3 reset
        UART3RST: u1,
        /// USART 4 reset
        UART4RST: u1,
        /// USART 5 reset
        UART5RST: u1,
        /// I2C 1 reset
        I2C1RST: u1,
        /// I2C 2 reset
        I2C2RST: u1,
        /// I2C3 reset
        I2C3RST: u1,
        reserved4: u1,
        /// CAN1 reset
        CAN1RST: u1,
        reserved5: u1,
        /// HDMI-CEC reset
        CECRST: u1,
        /// Power interface reset
        PWRRST: u1,
        /// DAC reset
        DACRST: u1,
        /// UART7 reset
        UART7RST: u1,
        /// UART8 reset
        UART8RST: u1,
    }, base_address + 0x20);

    /// address: 0x40023824
    /// APB2 peripheral reset register
    pub const APB2RSTR = @intToPtr(*volatile packed struct {
        /// TIM1 reset
        TIM1RST: u1,
        /// TIM8 reset
        TIM8RST: u1,
        reserved0: u1,
        reserved1: u1,
        /// USART1 reset
        USART1RST: u1,
        /// USART6 reset
        USART6RST: u1,
        reserved2: u1,
        /// SDMMC2 reset
        SDMMC2RST: u1,
        /// ADC interface reset (common to all
        /// ADCs)
        ADCRST: u1,
        reserved3: u1,
        reserved4: u1,
        /// SDMMC1 reset
        SDMMC1RST: u1,
        /// SPI 1 reset
        SPI1RST: u1,
        /// SPI4 reset
        SPI4RST: u1,
        /// System configuration controller
        /// reset
        SYSCFGRST: u1,
        reserved5: u1,
        /// TIM9 reset
        TIM9RST: u1,
        /// TIM10 reset
        TIM10RST: u1,
        /// TIM11 reset
        TIM11RST: u1,
        reserved6: u1,
        /// SPI5 reset
        SPI5RST: u1,
        reserved7: u1,
        /// SAI1 reset
        SAI1RST: u1,
        /// SAI2 reset
        SAI2RST: u1,
        reserved8: u1,
        reserved9: u1,
        reserved10: u1,
        reserved11: u1,
        reserved12: u1,
        reserved13: u1,
        reserved14: u1,
        /// USB OTG HS PHY controller
        /// reset
        USBPHYCRST: u1,
    }, base_address + 0x24);

    /// address: 0x40023830
    /// AHB1 peripheral clock register
    pub const AHB1ENR = @intToPtr(*volatile packed struct {
        /// IO port A clock enable
        GPIOAEN: u1,
        /// IO port B clock enable
        GPIOBEN: u1,
        /// IO port C clock enable
        GPIOCEN: u1,
        /// IO port D clock enable
        GPIODEN: u1,
        /// IO port E clock enable
        GPIOEEN: u1,
        /// IO port F clock enable
        GPIOFEN: u1,
        /// IO port G clock enable
        GPIOGEN: u1,
        /// IO port H clock enable
        GPIOHEN: u1,
        /// IO port I clock enable
        GPIOIEN: u1,
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        /// CRC clock enable
        CRCEN: u1,
        reserved3: u1,
        reserved4: u1,
        reserved5: u1,
        reserved6: u1,
        reserved7: u1,
        /// Backup SRAM interface clock
        /// enable
        BKPSRAMEN: u1,
        reserved8: u1,
        /// CCM data RAM clock enable
        DTCMRAMEN: u1,
        /// DMA1 clock enable
        DMA1EN: u1,
        /// DMA2 clock enable
        DMA2EN: u1,
        reserved9: u1,
        reserved10: u1,
        reserved11: u1,
        reserved12: u1,
        reserved13: u1,
        reserved14: u1,
        /// USB OTG HS clock enable
        OTGHSEN: u1,
        /// USB OTG HSULPI clock
        /// enable
        OTGHSULPIEN: u1,
        padding0: u1,
    }, base_address + 0x30);

    /// address: 0x40023834
    /// AHB2 peripheral clock enable
    /// register
    pub const AHB2ENR = @intToPtr(*volatile packed struct {
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        /// AES module clock enable
        AESEN: u1,
        reserved4: u1,
        /// Random number generator clock
        /// enable
        RNGEN: u1,
        /// USB OTG FS clock enable
        OTGFSEN: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
        padding8: u1,
        padding9: u1,
        padding10: u1,
        padding11: u1,
        padding12: u1,
        padding13: u1,
        padding14: u1,
        padding15: u1,
        padding16: u1,
        padding17: u1,
        padding18: u1,
        padding19: u1,
        padding20: u1,
        padding21: u1,
        padding22: u1,
        padding23: u1,
    }, base_address + 0x34);

    /// address: 0x40023838
    /// AHB3 peripheral clock enable
    /// register
    pub const AHB3ENR = @intToPtr(*volatile packed struct {
        /// Flexible memory controller module clock
        /// enable
        FMCEN: u1,
        /// Quad SPI memory controller clock
        /// enable
        QSPIEN: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
        padding8: u1,
        padding9: u1,
        padding10: u1,
        padding11: u1,
        padding12: u1,
        padding13: u1,
        padding14: u1,
        padding15: u1,
        padding16: u1,
        padding17: u1,
        padding18: u1,
        padding19: u1,
        padding20: u1,
        padding21: u1,
        padding22: u1,
        padding23: u1,
        padding24: u1,
        padding25: u1,
        padding26: u1,
        padding27: u1,
        padding28: u1,
        padding29: u1,
    }, base_address + 0x38);

    /// address: 0x40023840
    /// APB1 peripheral clock enable
    /// register
    pub const APB1ENR = @intToPtr(*volatile packed struct {
        /// TIM2 clock enable
        TIM2EN: u1,
        /// TIM3 clock enable
        TIM3EN: u1,
        /// TIM4 clock enable
        TIM4EN: u1,
        /// TIM5 clock enable
        TIM5EN: u1,
        /// TIM6 clock enable
        TIM6EN: u1,
        /// TIM7 clock enable
        TIM7EN: u1,
        /// TIM12 clock enable
        TIM12EN: u1,
        /// TIM13 clock enable
        TIM13EN: u1,
        /// TIM14 clock enable
        TIM14EN: u1,
        /// Low power timer 1 clock
        /// enable
        LPTIM1EN: u1,
        /// RTCAPB clock enable
        RTCAPBEN: u1,
        /// Window watchdog clock
        /// enable
        WWDGEN: u1,
        reserved0: u1,
        reserved1: u1,
        /// SPI2 clock enable
        SPI2EN: u1,
        /// SPI3 clock enable
        SPI3EN: u1,
        reserved2: u1,
        /// USART 2 clock enable
        USART2EN: u1,
        /// USART3 clock enable
        USART3EN: u1,
        /// UART4 clock enable
        UART4EN: u1,
        /// UART5 clock enable
        UART5EN: u1,
        /// I2C1 clock enable
        I2C1EN: u1,
        /// I2C2 clock enable
        I2C2EN: u1,
        /// I2C3 clock enable
        I2C3EN: u1,
        reserved3: u1,
        /// CAN 1 clock enable
        CAN1EN: u1,
        reserved4: u1,
        reserved5: u1,
        /// Power interface clock
        /// enable
        PWREN: u1,
        /// DAC interface clock enable
        DACEN: u1,
        /// UART7 clock enable
        UART7EN: u1,
        /// UART8 clock enable
        UART8EN: u1,
    }, base_address + 0x40);

    /// address: 0x40023844
    /// APB2 peripheral clock enable
    /// register
    pub const APB2ENR = @intToPtr(*volatile packed struct {
        /// TIM1 clock enable
        TIM1EN: u1,
        /// TIM8 clock enable
        TIM8EN: u1,
        reserved0: u1,
        reserved1: u1,
        /// USART1 clock enable
        USART1EN: u1,
        /// USART6 clock enable
        USART6EN: u1,
        reserved2: u1,
        /// SDMMC2 clock enable
        SDMMC2EN: u1,
        /// ADC1 clock enable
        ADC1EN: u1,
        /// ADC2 clock enable
        ADC2EN: u1,
        /// ADC3 clock enable
        ADC3EN: u1,
        /// SDMMC1 clock enable
        SDMMC1EN: u1,
        /// SPI1 clock enable
        SPI1EN: u1,
        /// SPI4 clock enable
        SPI4EN: u1,
        /// System configuration controller clock
        /// enable
        SYSCFGEN: u1,
        reserved3: u1,
        /// TIM9 clock enable
        TIM9EN: u1,
        /// TIM10 clock enable
        TIM10EN: u1,
        /// TIM11 clock enable
        TIM11EN: u1,
        reserved4: u1,
        /// SPI5 clock enable
        SPI5EN: u1,
        reserved5: u1,
        /// SAI1 clock enable
        SAI1EN: u1,
        /// SAI2 clock enable
        SAI2EN: u1,
        reserved6: u1,
        reserved7: u1,
        reserved8: u1,
        reserved9: u1,
        reserved10: u1,
        reserved11: u1,
        reserved12: u1,
        /// USB OTG HS PHY controller clock
        /// enable
        USBPHYCEN: u1,
    }, base_address + 0x44);

    /// address: 0x40023850
    /// AHB1 peripheral clock enable in low power
    /// mode register
    pub const AHB1LPENR = @intToPtr(*volatile packed struct {
        /// IO port A clock enable during sleep
        /// mode
        GPIOALPEN: u1,
        /// IO port B clock enable during Sleep
        /// mode
        GPIOBLPEN: u1,
        /// IO port C clock enable during Sleep
        /// mode
        GPIOCLPEN: u1,
        /// IO port D clock enable during Sleep
        /// mode
        GPIODLPEN: u1,
        /// IO port E clock enable during Sleep
        /// mode
        GPIOELPEN: u1,
        /// IO port F clock enable during Sleep
        /// mode
        GPIOFLPEN: u1,
        /// IO port G clock enable during Sleep
        /// mode
        GPIOGLPEN: u1,
        /// IO port H clock enable during Sleep
        /// mode
        GPIOHLPEN: u1,
        /// IO port I clock enable during Sleep
        /// mode
        GPIOILPEN: u1,
        /// IO port J clock enable during Sleep
        /// mode
        GPIOJLPEN: u1,
        /// IO port K clock enable during Sleep
        /// mode
        GPIOKLPEN: u1,
        reserved0: u1,
        /// CRC clock enable during Sleep
        /// mode
        CRCLPEN: u1,
        /// AXI to AHB bridge clock enable during
        /// Sleep mode
        AXILPEN: u1,
        reserved1: u1,
        /// Flash interface clock enable during
        /// Sleep mode
        FLITFLPEN: u1,
        /// SRAM 1interface clock enable during
        /// Sleep mode
        SRAM1LPEN: u1,
        /// SRAM 2 interface clock enable during
        /// Sleep mode
        SRAM2LPEN: u1,
        /// Backup SRAM interface clock enable
        /// during Sleep mode
        BKPSRAMLPEN: u1,
        /// SRAM 3 interface clock enable during
        /// Sleep mode
        SRAM3LPEN: u1,
        /// DTCM RAM interface clock enable during
        /// Sleep mode
        DTCMLPEN: u1,
        /// DMA1 clock enable during Sleep
        /// mode
        DMA1LPEN: u1,
        /// DMA2 clock enable during Sleep
        /// mode
        DMA2LPEN: u1,
        /// DMA2D clock enable during Sleep
        /// mode
        DMA2DLPEN: u1,
        reserved2: u1,
        /// Ethernet MAC clock enable during Sleep
        /// mode
        ETHMACLPEN: u1,
        /// Ethernet transmission clock enable
        /// during Sleep mode
        ETHMACTXLPEN: u1,
        /// Ethernet reception clock enable during
        /// Sleep mode
        ETHMACRXLPEN: u1,
        /// Ethernet PTP clock enable during Sleep
        /// mode
        ETHMACPTPLPEN: u1,
        /// USB OTG HS clock enable during Sleep
        /// mode
        OTGHSLPEN: u1,
        /// USB OTG HS ULPI clock enable during
        /// Sleep mode
        OTGHSULPILPEN: u1,
        padding0: u1,
    }, base_address + 0x50);

    /// address: 0x40023854
    /// AHB2 peripheral clock enable in low power
    /// mode register
    pub const AHB2LPENR = @intToPtr(*volatile packed struct {
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        /// AES module clock enable during Sleep
        /// mode
        AESLPEN: u1,
        reserved4: u1,
        /// Random number generator clock enable
        /// during Sleep mode
        RNGLPEN: u1,
        /// USB OTG FS clock enable during Sleep
        /// mode
        OTGFSLPEN: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
        padding8: u1,
        padding9: u1,
        padding10: u1,
        padding11: u1,
        padding12: u1,
        padding13: u1,
        padding14: u1,
        padding15: u1,
        padding16: u1,
        padding17: u1,
        padding18: u1,
        padding19: u1,
        padding20: u1,
        padding21: u1,
        padding22: u1,
        padding23: u1,
    }, base_address + 0x54);

    /// address: 0x40023858
    /// AHB3 peripheral clock enable in low power
    /// mode register
    pub const AHB3LPENR = @intToPtr(*volatile packed struct {
        /// Flexible memory controller module clock
        /// enable during Sleep mode
        FMCLPEN: u1,
        /// Quand SPI memory controller clock enable
        /// during Sleep mode
        QSPILPEN: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
        padding8: u1,
        padding9: u1,
        padding10: u1,
        padding11: u1,
        padding12: u1,
        padding13: u1,
        padding14: u1,
        padding15: u1,
        padding16: u1,
        padding17: u1,
        padding18: u1,
        padding19: u1,
        padding20: u1,
        padding21: u1,
        padding22: u1,
        padding23: u1,
        padding24: u1,
        padding25: u1,
        padding26: u1,
        padding27: u1,
        padding28: u1,
        padding29: u1,
    }, base_address + 0x58);

    /// address: 0x40023860
    /// APB1 peripheral clock enable in low power
    /// mode register
    pub const APB1LPENR = @intToPtr(*volatile packed struct {
        /// TIM2 clock enable during Sleep
        /// mode
        TIM2LPEN: u1,
        /// TIM3 clock enable during Sleep
        /// mode
        TIM3LPEN: u1,
        /// TIM4 clock enable during Sleep
        /// mode
        TIM4LPEN: u1,
        /// TIM5 clock enable during Sleep
        /// mode
        TIM5LPEN: u1,
        /// TIM6 clock enable during Sleep
        /// mode
        TIM6LPEN: u1,
        /// TIM7 clock enable during Sleep
        /// mode
        TIM7LPEN: u1,
        /// TIM12 clock enable during Sleep
        /// mode
        TIM12LPEN: u1,
        /// TIM13 clock enable during Sleep
        /// mode
        TIM13LPEN: u1,
        /// TIM14 clock enable during Sleep
        /// mode
        TIM14LPEN: u1,
        /// low power timer 1 clock enable during
        /// Sleep mode
        LPTIM1LPEN: u1,
        reserved0: u1,
        /// Window watchdog clock enable during
        /// Sleep mode
        WWDGLPEN: u1,
        reserved1: u1,
        reserved2: u1,
        /// SPI2 clock enable during Sleep
        /// mode
        SPI2LPEN: u1,
        /// SPI3 clock enable during Sleep
        /// mode
        SPI3LPEN: u1,
        reserved3: u1,
        /// USART2 clock enable during Sleep
        /// mode
        USART2LPEN: u1,
        /// USART3 clock enable during Sleep
        /// mode
        USART3LPEN: u1,
        /// UART4 clock enable during Sleep
        /// mode
        UART4LPEN: u1,
        /// UART5 clock enable during Sleep
        /// mode
        UART5LPEN: u1,
        /// I2C1 clock enable during Sleep
        /// mode
        I2C1LPEN: u1,
        /// I2C2 clock enable during Sleep
        /// mode
        I2C2LPEN: u1,
        /// I2C3 clock enable during Sleep
        /// mode
        I2C3LPEN: u1,
        reserved4: u1,
        /// CAN 1 clock enable during Sleep
        /// mode
        CAN1LPEN: u1,
        /// CAN 2 clock enable during Sleep
        /// mode
        CAN2LPEN: u1,
        reserved5: u1,
        /// Power interface clock enable during
        /// Sleep mode
        PWRLPEN: u1,
        /// DAC interface clock enable during Sleep
        /// mode
        DACLPEN: u1,
        /// UART7 clock enable during Sleep
        /// mode
        UART7LPEN: u1,
        /// UART8 clock enable during Sleep
        /// mode
        UART8LPEN: u1,
    }, base_address + 0x60);

    /// address: 0x40023864
    /// APB2 peripheral clock enabled in low power
    /// mode register
    pub const APB2LPENR = @intToPtr(*volatile packed struct {
        /// TIM1 clock enable during Sleep
        /// mode
        TIM1LPEN: u1,
        /// TIM8 clock enable during Sleep
        /// mode
        TIM8LPEN: u1,
        reserved0: u1,
        reserved1: u1,
        /// USART1 clock enable during Sleep
        /// mode
        USART1LPEN: u1,
        /// USART6 clock enable during Sleep
        /// mode
        USART6LPEN: u1,
        reserved2: u1,
        /// SDMMC2 clock enable during Sleep
        /// mode
        SDMMC2LPEN: u1,
        /// ADC1 clock enable during Sleep
        /// mode
        ADC1LPEN: u1,
        /// ADC2 clock enable during Sleep
        /// mode
        ADC2LPEN: u1,
        /// ADC 3 clock enable during Sleep
        /// mode
        ADC3LPEN: u1,
        /// SDMMC1 clock enable during Sleep
        /// mode
        SDMMC1LPEN: u1,
        /// SPI 1 clock enable during Sleep
        /// mode
        SPI1LPEN: u1,
        /// SPI 4 clock enable during Sleep
        /// mode
        SPI4LPEN: u1,
        /// System configuration controller clock
        /// enable during Sleep mode
        SYSCFGLPEN: u1,
        reserved3: u1,
        /// TIM9 clock enable during sleep
        /// mode
        TIM9LPEN: u1,
        /// TIM10 clock enable during Sleep
        /// mode
        TIM10LPEN: u1,
        /// TIM11 clock enable during Sleep
        /// mode
        TIM11LPEN: u1,
        reserved4: u1,
        /// SPI 5 clock enable during Sleep
        /// mode
        SPI5LPEN: u1,
        reserved5: u1,
        /// SAI1 clock enable during sleep
        /// mode
        SAI1LPEN: u1,
        /// SAI2 clock enable during sleep
        /// mode
        SAI2LPEN: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
    }, base_address + 0x64);

    /// address: 0x40023870
    /// Backup domain control register
    pub const BDCR = @intToPtr(*volatile packed struct {
        /// External low-speed oscillator
        /// enable
        LSEON: u1,
        /// External low-speed oscillator
        /// ready
        LSERDY: u1,
        /// External low-speed oscillator
        /// bypass
        LSEBYP: u1,
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        reserved4: u1,
        /// RTC clock source selection
        RTCSEL0: u1,
        /// RTC clock source selection
        RTCSEL1: u1,
        reserved5: u1,
        reserved6: u1,
        reserved7: u1,
        reserved8: u1,
        reserved9: u1,
        /// RTC clock enable
        RTCEN: u1,
        /// Backup domain software
        /// reset
        BDRST: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
        padding7: u1,
        padding8: u1,
        padding9: u1,
        padding10: u1,
        padding11: u1,
        padding12: u1,
        padding13: u1,
        padding14: u1,
    }, base_address + 0x70);

    /// address: 0x40023874
    /// clock control & status
    /// register
    pub const CSR = @intToPtr(*volatile packed struct {
        /// Internal low-speed oscillator
        /// enable
        LSION: u1,
        /// Internal low-speed oscillator
        /// ready
        LSIRDY: u1,
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        reserved4: u1,
        reserved5: u1,
        reserved6: u1,
        reserved7: u1,
        reserved8: u1,
        reserved9: u1,
        reserved10: u1,
        reserved11: u1,
        reserved12: u1,
        reserved13: u1,
        reserved14: u1,
        reserved15: u1,
        reserved16: u1,
        reserved17: u1,
        reserved18: u1,
        reserved19: u1,
        reserved20: u1,
        reserved21: u1,
        /// Remove reset flag
        RMVF: u1,
        /// BOR reset flag
        BORRSTF: u1,
        /// PIN reset flag
        PADRSTF: u1,
        /// POR/PDR reset flag
        PORRSTF: u1,
        /// Software reset flag
        SFTRSTF: u1,
        /// Independent watchdog reset
        /// flag
        WDGRSTF: u1,
        /// Window watchdog reset flag
        WWDGRSTF: u1,
        /// Low-power reset flag
        LPWRRSTF: u1,
    }, base_address + 0x74);

    /// address: 0x40023880
    /// spread spectrum clock generation
    /// register
    pub const SSCGR = @intToPtr(*volatile packed struct {
        /// Modulation period
        MODPER: u13,
        /// Incrementation step
        INCSTEP: u15,
        reserved0: u1,
        reserved1: u1,
        /// Spread Select
        SPREADSEL: u1,
        /// Spread spectrum modulation
        /// enable
        SSCGEN: u1,
    }, base_address + 0x80);

    /// address: 0x40023884
    /// PLLI2S configuration register
    pub const PLLI2SCFGR = @intToPtr(*volatile packed struct {
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        reserved4: u1,
        reserved5: u1,
        /// PLLI2S multiplication factor for
        /// VCO
        PLLI2SN: u9,
        reserved6: u1,
        reserved7: u1,
        reserved8: u1,
        reserved9: u1,
        reserved10: u1,
        reserved11: u1,
        reserved12: u1,
        reserved13: u1,
        reserved14: u1,
        /// PLLI2S division factor for SAI1
        /// clock
        PLLI2SQ: u4,
        /// PLLI2S division factor for I2S
        /// clocks
        PLLI2SR: u3,
        padding0: u1,
    }, base_address + 0x84);

    /// address: 0x40023888
    /// PLL configuration register
    pub const PLLSAICFGR = @intToPtr(*volatile packed struct {
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        reserved3: u1,
        reserved4: u1,
        reserved5: u1,
        /// PLLSAI division factor for
        /// VCO
        PLLSAIN: u9,
        reserved6: u1,
        /// PLLSAI division factor for 48MHz
        /// clock
        PLLSAIP: u2,
        reserved7: u1,
        reserved8: u1,
        reserved9: u1,
        reserved10: u1,
        reserved11: u1,
        reserved12: u1,
        /// PLLSAI division factor for SAI
        /// clock
        PLLSAIQ: u4,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
    }, base_address + 0x88);

    /// address: 0x4002388c
    /// dedicated clocks configuration
    /// register
    pub const DCKCFGR1 = @intToPtr(*volatile packed struct {
        /// PLLI2S division factor for SAI1
        /// clock
        PLLI2SDIV: u5,
        reserved0: u1,
        reserved1: u1,
        reserved2: u1,
        /// PLLSAI division factor for SAI1
        /// clock
        PLLSAIDIVQ: u5,
        reserved3: u1,
        reserved4: u1,
        reserved5: u1,
        reserved6: u1,
        reserved7: u1,
        reserved8: u1,
        reserved9: u1,
        /// SAI1 clock source
        /// selection
        SAI1SEL: u2,
        /// SAI2 clock source
        /// selection
        SAI2SEL: u2,
        /// Timers clocks prescalers
        /// selection
        TIMPRE: u1,
        padding0: u1,
        padding1: u1,
        padding2: u1,
        padding3: u1,
        padding4: u1,
        padding5: u1,
        padding6: u1,
    }, base_address + 0x8c);

    /// address: 0x40023890
    /// dedicated clocks configuration
    /// register
    pub const DCKCFGR2 = @intToPtr(*volatile packed struct {
        /// USART 1 clock source
        /// selection
        UART1SEL: u2,
        /// USART 2 clock source
        /// selection
        UART2SEL: u2,
        /// USART 3 clock source
        /// selection
        UART3SEL: u2,
        /// UART 4 clock source
        /// selection
        UART4SEL: u2,
        /// UART 5 clock source
        /// selection
        UART5SEL: u2,
        /// USART 6 clock source
        /// selection
        UART6SEL: u2,
        /// UART 7 clock source
        /// selection
        UART7SEL: u2,
        /// UART 8 clock source
        /// selection
        UART8SEL: u2,
        /// I2C1 clock source
        /// selection
        I2C1SEL: u2,
        /// I2C2 clock source
        /// selection
        I2C2SEL: u2,
        /// I2C3 clock source
        /// selection
        I2C3SEL: u2,
        reserved0: u1,
        reserved1: u1,
        /// Low power timer 1 clock source
        /// selection
        LPTIM1SEL: u2,
        reserved2: u1,
        /// 48MHz clock source
        /// selection
        CK48MSEL: u1,
        /// SDMMC1 clock source
        /// selection
        SDMMC1SEL: u1,
        /// SDMMC2 clock source
        /// selection
        SDMMC2SEL: u1,
        padding0: u1,
        padding1: u1,
    }, base_address + 0x90);
};

pub fn init() anyerror!void {
    RCC.CR.HSEON = 1;
    // 1. Enable HSE and wait for the HSE to become ready
    // regs.RCC.CR.modify(.{ .HSEON = 1 });
    while (RCC.CR.HSERDY != 1) {}

    // 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
    RCC.APB1ENR.PWREN = 1;
    regs.PWR.CR1.modify(.{ .VOS = 1 });

    // 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
    regs.RCC.CFGR.modify(.{
        .HPRE = 0, // DIV 1
        .PPRE1 = 5, // DIV 4
        .PPRE2 = 4, // DIV 2
    });

    RCC.CR.PLLON = 0;
    while (RCC.CR.PLLRDY != 0) {}

    // RCC.CIR.* = 0;
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
    RCC.CR.PLLON = 1;
    while (RCC.CR.PLLRDY != 1) {}

    regs.PWR.CR1.modify(.{ .ODEN = 1 });
    while (regs.PWR.CSR1.read().ODRDY != 1) {}

    // RCC.CR.HSION = 0;

    // 7. Select the clock source and waoit for it to be set
    RCC.CFGR.SW = 2; // System clock use PLL
    while (RCC.CFGR.SWS != 2) {}

    // 3. Configure the FLASH PREFETCH and the LATENCY related settings
    regs.FLASH.ACR.modify(.{ .ARTEN = 1, .PRFTEN = 1, .LATENCY = 7 });
    // enable instruction cache
    // regs.SCB.CCR.modify(.{ .IC = 1, .DC = 1 });
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

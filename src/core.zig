const std = @import("std");
const root = @import("root");
const builtin = @import("builtin");

extern var __data_start: anyopaque;
extern var __data_end: anyopaque;
extern var __bss_start: anyopaque;
extern var __bss_end: anyopaque;
extern const __data_load_start: anyopaque;
extern var __stack_end: u32;

pub const MainFunc = fn () anyerror!void;
// pub const PanicFunc = fn (message: []const u8, maybe_stack_trace: ?*std.builtin.StackTrace) noreturn;

pub fn Core(comptime arch:anytype, comptime main:MainFunc ) type {
    return struct {
        // pub const arch = arch;
        
        pub fn start() callconv(.C) noreturn {
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

        pub fn panic(message: []const u8, maybe_stack_trace: ?*std.builtin.StackTrace) noreturn {

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
                arch.cli();

                // "this loop has side effects, don't optimize the endless loop away please. thanks!"
                asm volatile ("" ::: "memory");
            }
        }
    };
}






const std = @import("std");


pub const Arm = struct {
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

  pub const Program = struct {
    pub fn initBss(start:[*]u8, end:[*]const u8 ) void {
        const bss_len = @ptrToInt(end) - @ptrToInt(start);

        std.mem.set(u8, start[0..bss_len], 0);
    }

    pub fn copyData(src:[*]const u8, start:[*]u8, end:[*]const u8) void {
        const data_len = @ptrToInt(end) - @ptrToInt(start);

        std.mem.copy(u8, start[0..data_len], src[0..data_len]);
    }
  };
};
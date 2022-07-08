const std = @import("std");

pub fn build(b: *std.build.Builder) void {
    // const mode = b.standardReleaseOptions();
    
    const exe = b.addExecutable(
        "STM32F7x2-firmware.elf",
        root()++"src/main.zig",
    );

    exe.addIncludeDir(root() ++ "src");
    exe.setLinkerScriptPath(.{.path=root() ++ "src/STM32F7x2/linker.ld"});
    exe.setTarget( std.zig.CrossTarget{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m7 },
        .os_tag = .freestanding,
        .abi = .none,
        });

    exe.bundle_compiler_rt = true;
    exe.emit_asm = .emit;
    // exe.emit_analysis = .emit;
    // exe.emit_llvm_bc = .emit;
    // exe.emit_llvm_ir = .emit;
    // This is important, without it, the linker removes the vector table
    exe.want_lto = false;

    //exe.step.dependOn(buildRegz(b, "libs/stm32-svd/f7/STM32F722.svd", "src/STM32F7x2"));

    // exe.setBuildMode(mode);
    exe.install();
}

fn root() []const u8 {
    return (std.fs.path.dirname(@src().file) orelse unreachable) ++ "/";
}

fn buildRegz(b: *std.build.Builder, comptime svdFile:[]const u8, comptime registersPath:[]const u8) *std.build.Step {
    const regz_step = b.step("regz svd", "Build registers.zig from svd file");

    const cmd = [_][]const u8 {
        root() ++ "libs/regz/zig-out/bin/regz",
        root() ++ svdFile,
        "-o", root() ++ registersPath ++ "/registers.zig"
    };

    regz_step.dependOn(&b.addSystemCommand(&cmd).step);
    
    return regz_step;
}

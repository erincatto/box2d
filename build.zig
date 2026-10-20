const std = @import("std");
const Build = std.Build;
const Compile = Build.Step.Compile;
const ResolvedTarget = Build.ResolvedTarget;
const Module = Build.Module;
const OptimizeMode = std.builtin.OptimizeMode;

const Translator = @import("translate_c").Translator;

pub const Options = struct {
    shared: bool,
    c_unit_tests: bool,
    zig_unit_tests: bool,
    disable_simd: bool,

    const defaults = Options{
        .shared = false,
        .c_unit_tests = false,
        .zig_unit_tests = false,
        .disable_simd = false,
    };

    pub fn getOptions(b: *Build) Options {
        return .{
            .shared = b.option(bool, "shared", "Compile as shared library") orelse defaults.shared,
            .c_unit_tests = b.option(bool, "c_unit_tests", "Compile the c unit tests") orelse defaults.c_unit_tests,
            .zig_unit_tests = b.option(bool, "zig_unit_tests", "Compile the zig unit tests") orelse defaults.zig_unit_tests,
            .disable_simd = b.option(bool, "disable_simd", "Disable SIMD math (slower)") orelse defaults.disable_simd,
        };
    }
};

pub fn build(b: *Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const options = Options.getOptions(b);
    const lib = try compileBox2d(b, target, optimize, options);

    b.installArtifact(lib);

    if (options.c_unit_tests) {
        buildCTests(b, target, optimize, lib);
    }

    // Translate the box2d headers and export them as a module
    const translate_c = b.dependency("translate_c", .{});
    const translator: Translator = .init(translate_c, .{
        .name = "translate_box2d",
        .c_source_file = b.path("include/box2d/box2d.h"),
        .target = target,
        .optimize = optimize,
    });
    translator.addIncludePath(b.path("include"));

    const box2d_mod = b.addModule("box2d", .{
        .root_source_file = translator.output_file,
    });
    box2d_mod.linkLibrary(lib);

    if (options.zig_unit_tests) {
        buildZigTest(b, target, optimize, box2d_mod);
    }
}

fn compileBox2d(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, options: Options) !*Compile {
    const module = b.addModule("box2dc", .{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });

    var box2d_flags_arr = std.ArrayList([]const u8).empty;
    defer box2d_flags_arr.deinit(b.allocator);

    try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
        "-std=gnu17",
        "-D_GNU_SOURCE",
        // Contraction into FMA is a value changing transform, keep it off for determinism
        "-ffp-contract=off",
    });

    if (options.shared) {
        try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
            "-fPIC",
        });

        // base.h picks dllexport/visibility from this define
        module.addCMacro("box2d_EXPORTS", "");
    }

    if (options.disable_simd) {
        module.addCMacro("BOX2D_DISABLE_SIMD", "");
    } else if (target.result.os.tag == .emscripten) {
        try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
            "-msimd128",
            "-msse2",
        });
    }

    const linkage: std.builtin.LinkMode = if (options.shared) .dynamic else .static;
    const box2d = b.addLibrary(.{
        .root_module = module,
        .name = "box2d",
        .linkage = linkage,
    });

    const c_source_files = &[_][]const u8{
        "src/aabb.c",
        "src/arena_allocator.c",
        "src/bitset.c",
        "src/body.c",
        "src/broad_phase.c",
        "src/constraint_graph.c",
        "src/contact_solver.c",
        "src/contact_solver_w4.c",
        "src/contact_solver_w8.c",
        "src/contact.c",
        "src/core.c",
        "src/distance_joint.c",
        "src/distance.c",
        "src/dynamic_tree.c",
        "src/geometry.c",
        "src/hull.c",
        "src/id_pool.c",
        "src/island.c",
        "src/joint.c",
        "src/manifold.c",
        "src/math_functions.c",
        "src/motor_joint.c",
        "src/mover_joint.c",
        "src/mover.c",
        "src/parallel_for.c",
        "src/physics_world.c",
        "src/pogo_joint.c",
        "src/prismatic_joint.c",
        "src/recording.c",
        "src/replay.c",
        "src/revolute_joint.c",
        "src/scheduler.c",
        "src/sensor.c",
        "src/shape.c",
        "src/simd.c",
        "src/snapshot.c",
        "src/solver_set.c",
        "src/solver.c",
        "src/table.c",
        "src/timer.c",
        "src/types.c",
        "src/weld_joint.c",
        "src/wheel_joint.c",
    };

    box2d.root_module.addIncludePath(b.path("include"));
    box2d.installHeadersDirectory(b.path("include/box2d"), "box2d", .{});

    box2d.root_module.addCSourceFiles(.{
        .files = c_source_files,
        .flags = box2d_flags_arr.items,
        .language = .c,
    });

    return box2d;
}

fn buildCTests(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, box2d_lib: *Compile) void {
    const test_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
    });

    test_module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "test/main.c",
            "test/test_bitset.c",
            "test/test_chain.c",
            "test/test_collision.c",
            "test/test_container.c",
            "test/test_determinism.c",
            "test/test_distance.c",
            "test/test_dynamic_tree.c",
            "test/test_id.c",
            "test/test_invalid_input.c",
            "test/test_large_world.c",
            "test/test_math.c",
            "test/test_mover.c",
            "test/test_recording.c",
            "test/test_restitution.c",
            "test/test_shape.c",
            "test/test_snapshot.c",
            "test/test_sort.c",
            "test/test_table.c",
            "test/test_thread.c",
            "test/test_world.c",
        },
        .flags = &[_][]const u8{
            "-std=c17",
            "-ffp-contract=off",
        },
        .language = .c,
    });

    // Special access to Box2D internals for testing
    test_module.addIncludePath(b.path("src"));
    test_module.addIncludePath(b.path("shared"));

    const shared_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
    });

    shared_module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "shared/benchmarks.c",
            "shared/determinism.c",
            "shared/human.c",
            "shared/utils.c",
        },
        .flags = &[_][]const u8{
            "-std=c17",
            // The golden hashes in the tests must match the library
            "-ffp-contract=off",
        },
        .language = .c,
    });

    shared_module.addIncludePath(b.path("include"));

    const shared_lib = b.addLibrary(.{
        .root_module = shared_module,
        .name = "shared",
        .linkage = .static,
    });
    shared_module.linkLibrary(box2d_lib);

    const test_exe = b.addExecutable(.{
        .name = "c_test",
        .root_module = test_module,
    });

    test_module.linkLibrary(box2d_lib);
    test_module.linkLibrary(shared_lib);

    b.installArtifact(test_exe);
}

// Build a simple zig test to validate that translate_c works as expected
fn buildZigTest(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, box2d_mod: *Module) void {
    const mod = b.createModule(.{
        .root_source_file = b.path("test/zig/test_world.zig"),
        .target = target,
        .optimize = optimize,
    });

    mod.addImport("box2d", box2d_mod);

    const zig_test = b.addTest(.{
        .name = "zig_test",
        .root_module = mod,
    });

    b.installArtifact(zig_test);
}

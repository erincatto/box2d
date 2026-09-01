const std = @import("std");
const Build = std.Build;
const Compile = Build.Step.Compile;
const ResolvedTarget = Build.ResolvedTarget;
const OptimizeMode = std.builtin.OptimizeMode;
const builtin = @import("builtin");

const min_supported_ver = "0.15.1";
comptime {
    const order = std.SemanticVersion.order;
    const parse = std.SemanticVersion.parse;
    if (order(builtin.zig_version, parse(min_supported_ver) catch unreachable) == .lt)
        @compileError("Box2d requires zig version " ++ min_supported_ver);
}

pub const Options = struct {
    shared: bool,
    unit_tests: bool,
    disable_simd: bool,
    avx2: bool,

    const defaults = Options{
        .shared = false,
        .unit_tests = false,
        .disable_simd = false,
        .avx2 = false,
    };

    pub fn getOptions(b: *Build, target: ResolvedTarget) Options {
        const disable_simd = b.option(bool, "disable_simd", "Disable SIMD math (slower)") orelse defaults.disable_simd;

        const avx2 = avx2_option_blk: {
            const has_avx2 = target.result.cpu.has(.x86, .avx2);
            if (has_avx2 and !disable_simd) {
                break :avx2_option_blk b.option(bool, "avx2", "Enable AVX2") orelse defaults.avx2;
            }

            break :avx2_option_blk false;
        };

        return .{
            .shared = b.option(bool, "shared", "Compile as shared library") orelse defaults.shared,
            .unit_tests = b.option(bool, "unit_tests", "Compile unit tests") orelse defaults.unit_tests,
            .disable_simd = disable_simd,
            .avx2 = avx2,
        };
    }
};

pub fn build(b: *Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const options = Options.getOptions(b, target);
    const lib = try compileBox2d(b, target, optimize, options);

    b.installArtifact(lib);

    if (options.unit_tests) {
        buildTests(b, target, optimize, lib);
    }
}

fn compileBox2d(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, options: Options) !*Compile {
    const module = b.addModule("box2d", .{
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

    if (options.avx2) {
        module.addCMacro("BOX2D_AVX2", "");
        try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
            "-mavx2",
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
        "src/contact.c",
        "src/contact_solver.c",
        "src/core.c",
        "src/distance.c",
        "src/distance_joint.c",
        "src/dynamic_tree.c",
        "src/geometry.c",
        "src/hull.c",
        "src/id_pool.c",
        "src/island.c",
        "src/joint.c",
        "src/manifold.c",
        "src/math_functions.c",
        "src/motor_joint.c",
        "src/mover.c",
        "src/mover_joint.c",
        "src/parallel_for.c",
        "src/physics_world.c",
        "src/pogo_joint.c",
        "src/prismatic_joint.c",
        "src/recording.c",
        "src/recording_replay.c",
        "src/revolute_joint.c",
        "src/scheduler.c",
        "src/sensor.c",
        "src/shape.c",
        "src/solver.c",
        "src/solver_set.c",
        "src/table.c",
        "src/timer.c",
        "src/types.c",
        "src/weld_joint.c",
        "src/wheel_joint.c",
        "src/world_snapshot.c",
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

fn buildTests(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, box2d_lib: *Compile) void {
    const test_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
    });

    test_module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "test/main.c",
            "test/test_bitset.c",
            "test/test_collision.c",
            "test/test_container.c",
            "test/test_determinism.c",
            "test/test_distance.c",
            "test/test_dynamic_tree.c",
            "test/test_id.c",
            "test/test_large_world.c",
            "test/test_math.c",
            "test/test_recording.c",
            "test/test_restitution.c",
            "test/test_shape.c",
            "test/test_snapshot.c",
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
        .name = "test",
        .root_module = test_module,
    });

    test_module.linkLibrary(box2d_lib);
    test_module.linkLibrary(shared_lib);

    b.installArtifact(test_exe);
}

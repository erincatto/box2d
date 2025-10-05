const std = @import("std");
const Build = std.Build;
const Compile = Build.Step.Compile;
const ResolvedTarget = Build.ResolvedTarget;
const OptimizeMode = std.builtin.OptimizeMode;
const builtin = @import("builtin");

const min_supported_ver = "0.15.0";
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
        .avx2 = true,
    };

    pub fn getOptions(b: *Build, target: ResolvedTarget) Options {
        const disable_simd = b.option(bool, "disable_simd", "Disable SIMD math (slower)") orelse defaults.disable_simd;

        const avx2 = avx2_option_blk: {
            const has_avx2 = target.result.cpu.has(.x86, .avx2);
            if (has_avx2 and disable_simd == false) {
                break :avx2_option_blk b.option(bool, "avx2", "Enable AVX2") orelse defaults.avx2;
            }

            break :avx2_option_blk false;
        };

        return .{
            .shared = b.option(bool, "shared", "Compile as shared library") orelse defaults.shared,
            .unit_tests = b.option(bool, "unit_tests", "Compile units tests") orelse defaults.unit_tests,
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

    var test_sample_deps: ?TestAndSamplesDeps = null;
    if (options.unit_tests) {
        test_sample_deps = .init(b, target, optimize, lib);
    }

    if (options.unit_tests) {
        buildTests(b, target, optimize, lib, test_sample_deps.?);
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
        "-std=gnu99",
        "-D_GNU_SOURCE",
        "-ffp-contract=off",
    });

    if (options.shared) {
        try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
            "-fPIC",
            "-DBUILD_LIBTYPE_SHARED",
        });
    }

    if (options.disable_simd) {
        module.addCMacro("BOX2D_DISABLE_SIMD", "");
    } else {
        if (target.result.os.tag == .emscripten) {
            try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
                "-msimd128",
                "-msse2",
            });
        }
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
        "src/aabb.h",
        "src/arena_allocator.c",
        "src/arena_allocator.h",
        "src/array.c",
        "src/array.h",
        "src/atomic.h",
        "src/bitset.c",
        "src/bitset.h",
        "src/body.c",
        "src/body.h",
        "src/broad_phase.c",
        "src/broad_phase.h",
        "src/constants.h",
        "src/constraint_graph.c",
        "src/constraint_graph.h",
        "src/contact.c",
        "src/contact.h",
        "src/contact_solver.c",
        "src/contact_solver.h",
        "src/core.c",
        "src/core.h",
        "src/ctz.h",
        "src/distance.c",
        "src/distance_joint.c",
        "src/dynamic_tree.c",
        "src/geometry.c",
        "src/hull.c",
        "src/id_pool.c",
        "src/id_pool.h",
        "src/island.c",
        "src/island.h",
        "src/joint.c",
        "src/joint.h",
        "src/manifold.c",
        "src/math_functions.c",
        "src/motor_joint.c",
        "src/mover.c",
        "src/physics_world.c",
        "src/physics_world.h",
        "src/prismatic_joint.c",
        "src/revolute_joint.c",
        "src/sensor.c",
        "src/sensor.h",
        "src/shape.c",
        "src/shape.h",
        "src/solver.c",
        "src/solver.h",
        "src/solver_set.c",
        "src/solver_set.h",
        "src/table.c",
        "src/table.h",
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

pub fn buildTests(
    b: *Build,
    target: ResolvedTarget,
    optimize: OptimizeMode,
    box2d_lib: *Compile,
    test_deps: TestAndSamplesDeps,
) void {
    const module = b.createModule(.{
        .target = target,
        .optimize = optimize,
    });

    module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "test/main.c",
            "test/test_bitset.c",
            "test/test_collision.c",
            "test/test_determinism.c",
            "test/test_distance.c",
            "test/test_id.c",
            "test/test_math.c",
            "test/test_shape.c",
            "test/test_table.c",
            "test/test_world.c",
        },
        .flags = &[_][]const u8{
            "-std=c17",
            "-ffp-contract=off",
        },
        .language = .c,
    });

    const exe = b.addExecutable(.{
        .name = "test",
        .root_module = module,
    });

    exe.root_module.addIncludePath(b.path("src"));
    exe.root_module.addIncludePath(b.path("shared"));

    exe.root_module.linkLibrary(box2d_lib);
    exe.root_module.linkLibrary(test_deps.shared_lib.?);
    exe.root_module.linkLibrary(test_deps.enki_ts);

    b.installArtifact(exe);
}

pub fn buildShared(
    b: *Build,
    target: ResolvedTarget,
    optimize: OptimizeMode,
    box2d_lib: *Compile,
) *Compile {
    const module = b.createModule(.{
        .target = target,
        .optimize = optimize,
    });

    module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "shared/benchmarks.c",
            "shared/benchmarks.h",
            "shared/determinism.c",
            "shared/determinism.h",
            "shared/human.c",
            "shared/human.h",
            "shared/random.c",
            "shared/random.h",
        },
        .flags = &[_][]const u8{
            "-ffp-contract=off",
        },
        .language = .c,
    });

    module.linkLibrary(box2d_lib);

    return b.addLibrary(.{
        .root_module = module,
        .name = "shared",
        .linkage = .static,
    });
}

const TestAndSamplesDeps = struct {
    shared_lib: ?*Compile,
    enki_ts: *Compile,

    pub fn init(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, box2d_lib: *Compile) TestAndSamplesDeps {
        const enki_ts = get_enki_ts_artifact: {
            const enki_ts_dep = b.dependency("enkiTS", .{
                .shared = false,
            });

            break :get_enki_ts_artifact enki_ts_dep.artifact("enki_ts");
        };

        return TestAndSamplesDeps{
            .shared_lib = buildShared(b, target, optimize, box2d_lib),
            .enki_ts = enki_ts,
        };
    }
};

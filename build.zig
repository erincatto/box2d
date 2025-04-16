const std = @import("std");
const builtin = @import("builtin");

const min_supported_ver = "0.14.0";

comptime {
    const order = std.SemanticVersion.order;
    const parse = std.SemanticVersion.parse;
    if (order(builtin.zig_version, parse(min_supported_ver) catch unreachable) == .lt)
        @compileError("Box2d requires zig version " ++ min_supported_ver);
}

fn compileBox2d(b: *std.Build, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode, options: Options) !*std.Build.Step.Compile {
    var box2d_flags_arr = std.ArrayList([]const u8).init(b.allocator);
    defer box2d_flags_arr.deinit();

    try box2d_flags_arr.appendSlice(&[_][]const u8{
        "-std=gnu99",
        "-D_GNU_SOURCE",
    });

    if (options.shared) {
        try box2d_flags_arr.appendSlice(&[_][]const u8{
            "-fPIC",
            "-DBUILD_LIBTYPE_SHARED",
        });
    }

    const box2d = if (options.shared)
        b.addSharedLibrary(.{
            .name = "box2d",
            .target = target,
            .optimize = optimize,
        })
    else
        b.addStaticLibrary(.{
            .name = "box2d",
            .target = target,
            .optimize = optimize,
        });
    box2d.linkLibC();

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
        "src/mouse_joint.c",
        "src/mover.c",
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
        "src/world.c",
        "src/world.h",
    };

    box2d.addIncludePath(b.path("include"));

    box2d.root_module.addCSourceFiles(.{
        .files = c_source_files,
        .flags = box2d_flags_arr.items,
    });

    return box2d;
}

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const lib = try compileBox2d(b, target, optimize, Options.getOptions(b));
    lib.installHeader(b.path("include/box2d/base.h"), "base.h");
    lib.installHeader(b.path("include/box2d/box2d.h"), "box2d/box2d.h");
    lib.installHeader(b.path("include/box2d/collision.h"), "collision.h");
    lib.installHeader(b.path("include/box2d/id.h"), "id.h");
    lib.installHeader(b.path("include/box2d/math_functions.h"), "math_functions.h");
    lib.installHeader(b.path("include/box2d/types.h"), "types.h");

    b.installArtifact(lib);
}

pub const Options = struct {
    shared: bool = false,

    const defaults = Options{};

    pub fn getOptions(b: *std.Build) Options {
        return .{
            .shared = b.option(bool, "shared", "Compile as shared library") orelse defaults.shared,
        };
    }
};

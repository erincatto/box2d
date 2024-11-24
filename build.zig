const std = @import("std");

pub const Options = struct {
    // sanitize: bool = false,
    simd: bool = true,
    //
    // samples: bool = true,
    // benchmarks: bool = false,
    // docs: bool = false,
    // profile: bool = false,
    // validate: bool = true,
    // tests: bool = true,

    const defaults = Options{};

    fn getOptions(b: *std.Build) Options {
        return .{
            // .sanitize = b.option(bool, "BOX2D_SANITIZE", "Enable sanitizers for some builds") orelse defaults.sanitize,
            .simd = b.option(bool, "BOX2D_ENABLE_SIMD", "Enable SIMD math (faster)") orelse defaults.simd,
            //
            // .samples = b.option("BOX2D_SAMPLES", "Build the Box2D samples") orelse defaults.samples,
            // .benchmarks = b.option("BOX2D_BENCHMARKS", "Build the Box2D benchmarks") orelse defaults.benchmarks,
            // .docs = b.option("BOX2D_DOCS", "Build the Box2D documentation") orelse defaults.docs,
            // .profile = b.option("BOX2D_PROFILE", "Enable profiling with Tracy") orelse defaults.profile,
            // .validate = b.option("BOX2D_VALIDATE", "Enable heavy validation") orelse defaults.validate,
            // .tests = b.option("BOX2D_UNIT_TESTS", "Build the Box2D unit tests") orelse defaults.tests,
        };
    }
};

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const options = Options.getOptions(b);

    const box2d = b.addStaticLibrary(.{
        .name = "box2d",
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });

    const c_flags: []const []const u8 = blk: {
        var res = std.ArrayList([]const u8).init(b.allocator);
        defer res.deinit();

        // if (options.sanitize) {
        //     try res.append("--fsanitize-c");
        //     try res.append("--fsanitize-thread");
        // }
        if (options.simd) {
            box2d.defineCMacro("BOX2D_ENABLE_SIMD", null);
            try res.append("-DBOX2D_ENABLE_SIMD");
        }

        break :blk try res.toOwnedSlice();
    };

    box2d.addIncludePath(b.path("src"));
    box2d.addIncludePath(b.path("include"));
    box2d.addCSourceFiles(.{
        .files = &.{
            "src/aabb.c",
            "src/aabb.h",
            "src/array.c",
            "src/array.h",
            "src/bitset.c",
            "src/bitset.h",
            "src/body.c",
            "src/body.h",
            "src/broad_phase.c",
            "src/broad_phase.h",
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
            "src/prismatic_joint.c",
            "src/revolute_joint.c",
            "src/shape.c",
            "src/shape.h",
            "src/solver.c",
            "src/solver.h",
            "src/solver_set.c",
            "src/solver_set.h",
            "src/stack_allocator.c",
            "src/stack_allocator.h",
            "src/table.c",
            "src/table.h",
            "src/timer.c",
            "src/types.c",
            "src/weld_joint.c",
            "src/wheel_joint.c",
            "src/world.c",
            "src/world.h",
        },
        .flags = c_flags,
    });

    box2d.installHeadersDirectory(b.path("include"), "include", .{});

    b.installArtifact(box2d);

    // TranslateC step
    const cstep_box2dmod = b.addTranslateC(.{
        .root_source_file = b.addWriteFiles().add(
            "box2d_includes.h",
            \\#include "box2d/base.h"
            \\#include "box2d/box2d.h"
            \\#include "box2d/collision.h"
            \\#include "box2d/id.h"
            \\#include "box2d/math_functions.h"
            \\#include "box2d/types.h"
            ,
        ),
        .target = target,
        .optimize = optimize,
    });
    cstep_box2dmod.addIncludePath(b.path("include"));
    cstep_box2dmod.step.dependOn(&box2d.step);

    // Module for external import
    const box2d_mod = cstep_box2dmod.createModule();
    box2d_mod.linkLibrary(box2d);

    // Questionably
    try b.modules.put("box2d_mod", box2d_mod);
}

// swift-tools-version:5.10
import PackageDescription

let package = Package(
    name: "box2d",
    platforms: [
        // aligned_alloc requires these floors
        .macOS(.v10_15),
        .iOS(.v13),
        .tvOS(.v13),
        .watchOS(.v6)
    ],
    products: [
        .library(
            name: "box2d",
            targets: ["box2d"]
        ),
    ],
    targets: [
        .target(
            name: "box2d",
            path: ".",
            exclude: [
                "src/box2d.natvis",
                "src/box2d.pc.in",
                "src/CMakeLists.txt",
                "src/recording_ops.inl"
            ],
            sources: [
                "src"
            ],
            publicHeadersPath: "include",
            cSettings: [
                .headerSearchPath("include"),
                .headerSearchPath("src")
            ]
        ),
        .testTarget(
            name: "test_world",
            dependencies: ["box2d"],
            path: "test/swift"
        ),
    ],
    cLanguageStandard: .c17
)

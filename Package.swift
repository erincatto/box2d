// swift-tools-version:5.10
import PackageDescription

let package = Package(
    name: "Box2D",
    platforms: [
        // aligned_alloc requires these floors
        .macOS(.v10_15),
        .iOS(.v13),
        .tvOS(.v13),
        .watchOS(.v6)
    ],
    products: [
        .library(
            name: "Box2D",
            targets: ["Box2D"]
        ),
    ],
    targets: [
        .target(
            name: "Box2D",
            path: ".",
            exclude: [
                "src/box2d.natvis",
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
            dependencies: ["Box2D"],
            path: "test/swift"
        ),
    ],
    cLanguageStandard: .c17
)

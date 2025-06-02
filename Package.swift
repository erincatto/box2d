// swift-tools-version:5.10
import PackageDescription

let package = Package(
    name: "box2d",
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
                "src/CMakeLists.txt"
            ],
            sources: [
                "src"
            ],
            publicHeadersPath: "include",
            cSettings: [
                .define("B2_API", to: ""),
                .headerSearchPath("include"),
                .headerSearchPath("src")
            ]
        ),
    ],
    cLanguageStandard: .c17
)

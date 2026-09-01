import XCTest
import box2d

final class Box2DTests: XCTestCase {
    func testVersion() {
        let version = b2GetVersion()
        XCTAssertGreaterThanOrEqual(version.major, 3)
    }

    func testWorldStep() {
        var worldDef = b2DefaultWorldDef()
        let worldId = b2CreateWorld(&worldDef)
        XCTAssertTrue(b2World_IsValid(worldId))

        var groundDef = b2DefaultBodyDef()
        groundDef.position = b2Pos(x: 0.0, y: -10.0)
        let groundId = b2CreateBody(worldId, &groundDef)
        var groundBox = b2MakeBox(100.0, 1.0)
        var shapeDef = b2DefaultShapeDef()
        _ = b2CreatePolygonShape(groundId, &shapeDef, &groundBox)

        var bodyDef = b2DefaultBodyDef()
        bodyDef.type = b2_dynamicBody
        bodyDef.position = b2Pos(x: 0.0, y: 10.0)
        let bodyId = b2CreateBody(worldId, &bodyDef)
        var bodyBox = b2MakeBox(0.5, 0.5)
        _ = b2CreatePolygonShape(bodyId, &shapeDef, &bodyBox)

        for _ in 0..<120 {
            b2World_Step(worldId, 1.0 / 60.0, 4)
        }

        let y = b2Body_GetPosition(bodyId).y
        XCTAssertLessThan(y, 10.0)
        XCTAssertGreaterThan(y, -10.0)

        b2DestroyWorld(worldId)
        XCTAssertFalse(b2World_IsValid(worldId))
    }
}

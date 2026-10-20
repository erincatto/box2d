const std = @import("std");

const box2d = @import("box2d");

// Simple world test to ensure translate_c creates the expected API for us
test "simple world" {
    const world_def = box2d.b2DefaultWorldDef();
    const world_id = box2d.b2CreateWorld(&world_def);
    try std.testing.expect(box2d.b2World_IsValid(world_id));

    var ground_def = box2d.b2DefaultBodyDef();
    ground_def.position = box2d.b2Pos{ .x = 0.0, .y = -10.0 };

    const ground_id = box2d.b2CreateBody(world_id, &ground_def);
    var ground_box = box2d.b2MakeBox(100.0, 1.0);
    var shape_def = box2d.b2DefaultShapeDef();
    _ = box2d.b2CreatePolygonShape(ground_id, &shape_def, &ground_box);

    var body_def = box2d.b2DefaultBodyDef();
    body_def.type = box2d.b2_dynamicBody;
    body_def.position = box2d.b2Pos{ .x = 0.0, .y = 10.0 };
    const body_id = box2d.b2CreateBody(world_id, &body_def);
    var body_box = box2d.b2MakeBox(0.5, 0.5);
    _ = box2d.b2CreatePolygonShape(body_id, &shape_def, &body_box);

    for (0..120) |_| {
        box2d.b2World_Step(world_id, 1.0 / 60.0, 4);
    }

    const y = box2d.b2Body_GetPosition(body_id).y;
    try std.testing.expect(y < 10);
    try std.testing.expect(y > -10);

    box2d.b2DestroyWorld(world_id);
    try std.testing.expect(!box2d.b2World_IsValid(world_id));
}

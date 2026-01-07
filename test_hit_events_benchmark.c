// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
// 
// Hit Events Optimization Benchmark
// Tests Option A (early exit) and Option B (bitset iteration)

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BODY_COUNT 5000
#define STEP_COUNT 100
#define WARMUP_STEPS 20

static void CreateBodies(b2WorldId worldId, b2BodyId* bodies, b2ShapeId* shapes, int count, bool enableHitEvents)
{
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.enableHitEvents = enableHitEvents;
    
    b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
    
    for (int i = 0; i < count; ++i)
    {
        float x = (float)(i % 50) * 1.5f - 37.5f;
        float y = (float)(i / 50) * 1.5f + 5.0f;
        bodyDef.position = (b2Vec2){ x, y };
        
        bodies[i] = b2CreateBody(worldId, &bodyDef);
        shapes[i] = b2CreateCircleShape(bodies[i], &shapeDef, &circle);
    }
}

static void CreateGround(b2WorldId worldId)
{
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_staticBody;
    bodyDef.position = (b2Vec2){ 0.0f, -1.0f };
    
    b2BodyId groundId = b2CreateBody(worldId, &bodyDef);
    
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    b2Polygon box = b2MakeBox(100.0f, 1.0f);
    b2CreatePolygonShape(groundId, &shapeDef, &box);
}

static float RunBenchmark(const char* name, bool enableHitEvents, int hitEventBodyCount)
{
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
    b2WorldId worldId = b2CreateWorld(&worldDef);
    
    CreateGround(worldId);
    
    b2BodyId* bodies = malloc(BODY_COUNT * sizeof(b2BodyId));
    b2ShapeId* shapes = malloc(BODY_COUNT * sizeof(b2ShapeId));
    
    // Create bodies, only first hitEventBodyCount have hit events enabled
    for (int i = 0; i < BODY_COUNT; ++i)
    {
        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_dynamicBody;
        float x = (float)(i % 50) * 1.5f - 37.5f;
        float y = (float)(i / 50) * 1.5f + 5.0f;
        bodyDef.position = (b2Vec2){ x, y };
        
        bodies[i] = b2CreateBody(worldId, &bodyDef);
        
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.enableHitEvents = enableHitEvents && (i < hitEventBodyCount);
        
        b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
        shapes[i] = b2CreateCircleShape(bodies[i], &shapeDef, &circle);
    }
    
    float timeStep = 1.0f / 60.0f;
    int subStepCount = 4;
    
    // Warmup
    for (int i = 0; i < WARMUP_STEPS; ++i)
    {
        b2World_Step(worldId, timeStep, subStepCount);
    }
    
    // Benchmark
    float totalHitEventTime = 0.0f;
    float totalStepTime = 0.0f;
    
    for (int i = 0; i < STEP_COUNT; ++i)
    {
        b2World_Step(worldId, timeStep, subStepCount);
        b2Profile profile = b2World_GetProfile(worldId);
        totalHitEventTime += profile.hitEvents;
        totalStepTime += profile.step;
    }
    
    float avgHitEventTime = totalHitEventTime / STEP_COUNT;
    float avgStepTime = totalStepTime / STEP_COUNT;
    
    printf("%s:\n", name);
    printf("  Bodies: %d, Hit Event Shapes: %d\n", BODY_COUNT, hitEventBodyCount);
    printf("  Avg hitEvents time: %.4f ms\n", avgHitEventTime);
    printf("  Avg step time: %.4f ms\n", avgStepTime);
    printf("  hitEvents %% of step: %.2f%%\n", (avgHitEventTime / avgStepTime) * 100.0f);
    printf("\n");
    
    free(bodies);
    free(shapes);
    b2DestroyWorld(worldId);
    
    return avgHitEventTime;
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    
    printf("==============================================\n");
    printf("Hit Events Optimization Benchmark\n");
    printf("Testing Option A (early exit) and Option B (bitset)\n");
    printf("==============================================\n\n");
    
    printf("Configuration: %d bodies, %d steps\n\n", BODY_COUNT, STEP_COUNT);
    
    // Test 1: No hit events (Option A should skip entirely)
    float time_none = RunBenchmark("Scenario 1: No hit events enabled", false, 0);
    
    // Test 2: 10 bodies with hit events (Option B should iterate only 10)
    float time_10 = RunBenchmark("Scenario 2: 10 shapes with hit events", true, 10);
    
    // Test 3: 100 bodies with hit events
    float time_100 = RunBenchmark("Scenario 3: 100 shapes with hit events", true, 100);
    
    // Test 4: 500 bodies with hit events
    float time_500 = RunBenchmark("Scenario 4: 500 shapes with hit events", true, 500);
    
    // Test 5: All bodies with hit events
    float time_all = RunBenchmark("Scenario 5: All shapes with hit events", true, BODY_COUNT);
    
    printf("==============================================\n");
    printf("Summary\n");
    printf("==============================================\n");
    printf("  No hit events:     %.4f ms (Option A: early exit)\n", time_none);
    printf("  10 hit events:     %.4f ms\n", time_10);
    printf("  100 hit events:    %.4f ms\n", time_100);
    printf("  500 hit events:    %.4f ms\n", time_500);
    printf("  All hit events:    %.4f ms\n", time_all);
    printf("\n");
    
    if (time_none < 0.001f)
    {
        printf("SUCCESS: Option A (early exit) is working - near-zero time with no hit events\n");
    }
    else
    {
        printf("WARNING: Option A may not be working - expected near-zero time with no hit events\n");
    }
    
    if (time_10 < time_all * 0.5f)
    {
        printf("SUCCESS: Option B (bitset) appears to be working - 10 shapes much faster than all\n");
    }
    else
    {
        printf("INFO: Option B scaling - 10 shapes vs all shapes\n");
    }
    
    return 0;
}

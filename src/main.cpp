#include "raylib.h"
#include "raymath.h"   // Vector2 math + C++ operators (+, -, *, +=)
#include <vector>
#include <cmath>

// ---------------- Screen settings ----------------
const int SCREEN_W = 1280;
const int SCREEN_H = 720;

// World -> screen conversion
// World: units, y goes UP. Screen: pixels, y goes DOWN.
const float PX_PER_UNIT = 60.0f;
const float ORIGIN_X = 80.0f;   // screen x of world x = 0
const float ORIGIN_Y = 640.0f;  // screen y of world y = 0 (ground)

Vector2 WorldToScreen(Vector2 p)
{
    return { ORIGIN_X + p.x * PX_PER_UNIT, ORIGIN_Y - p.y * PX_PER_UNIT };
}

// Turn an angle (degrees) + length into a vector
Vector2 VectorFromAngle(float angleDeg, float length)
{
    float rad = angleDeg * DEG2RAD;
    return { cosf(rad) * length, sinf(rad) * length };
}

// =====================================================
// PhysicsBody: one object in the simulation
// =====================================================
class PhysicsBody
{
public:
    Vector2 position = { 0.0f, 0.0f };   // units
    Vector2 velocity = { 0.0f, 0.0f };   // units per second
    float drag = 0.0f;                   // stored, not used in this lab (no air resistance)
    float mass = 1.0f;                   // kg, used in later labs

    bool active = true;                  // false = stopped (landed / left the world)
    Color color = MAROON;
    std::vector<Vector2> trail;          // positions over time, only for drawing

    PhysicsBody(Vector2 startPos, Vector2 startVel, Color c)
        : position(startPos), velocity(startVel), color(c)
    {
        trail.push_back(position);
    }
};

// =====================================================
// PhysicsSimulation: owns the bodies and moves them
// =====================================================
class PhysicsSimulation
{
public:
    float deltaTime = 0.0f;                 // seconds, duration of the last frame
    float time = 0.0f;                      // seconds since the simulation started
    Vector2 gravity = { 0.0f, -9.81f };     // units per second^2 (direction + magnitude)

    std::vector<PhysicsBody> bodies;

    void AddBody(const PhysicsBody& body)
    {
        bodies.push_back(body);
    }

    void Clear()
    {
        bodies.clear();
    }

    // One step forward in time. Same rules apply to every body.
    void Update(float dt)
    {
        deltaTime = dt;
        time += dt;

        for (PhysicsBody& b : bodies)
        {
            if (!b.active) continue;

            // Kinematics (semi-implicit Euler):
            b.velocity += gravity * deltaTime;      // v = v + g * dt
            b.position += b.velocity * deltaTime;   // p = p + v * dt

            b.trail.push_back(b.position);

            // Stop the body when it reaches the ground or leaves the world
            // (not a real collision - just so the trail stays on screen)
            if (b.position.y <= 0.0f)
            {
                b.position.y = 0.0f;
                b.trail.back() = b.position;
                b.active = false;
            }
            else if (b.position.x < -5.0f || b.position.x > 30.0f || b.position.y > 30.0f)
            {
                b.active = false;
            }
        }
    }
};

int main()
{
    InitWindow(SCREEN_W, SCREEN_H, "GAME2005 Lab 2 - Launching Bird Simulation - Myroslav Pochynok");
    SetTargetFPS(60);

    PhysicsSimulation sim;

    // Launch settings
    Vector2 launchPosition = { 1.0f, 1.0f };  // units
    float launchAngle = 45.0f;                // degrees
    float launchSpeed = 10.0f;                // units per second

    // Gravity settings (turned into sim.gravity every frame)
    float gravityMagnitude = 9.81f;           // units per second^2
    float gravityAngle = -90.0f;              // degrees, -90 = straight down

    const Color ballColors[] = { MAROON, ORANGE, DARKGREEN, DARKBLUE, PURPLE, BROWN, MAGENTA, DARKGRAY };
    const int colorCount = sizeof(ballColors) / sizeof(ballColors[0]);
    int launchCount = 0;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // ================= INPUT =================
        // Launch speed / angle
        if (IsKeyDown(KEY_UP))    launchSpeed += 5.0f * dt;
        if (IsKeyDown(KEY_DOWN))  launchSpeed -= 5.0f * dt;
        if (IsKeyDown(KEY_LEFT))  launchAngle += 30.0f * dt;
        if (IsKeyDown(KEY_RIGHT)) launchAngle -= 30.0f * dt;

        // Angle presets for the demo
        if (IsKeyPressed(KEY_ONE))   launchAngle = 0.0f;
        if (IsKeyPressed(KEY_TWO))   launchAngle = 45.0f;
        if (IsKeyPressed(KEY_THREE)) launchAngle = 60.0f;
        if (IsKeyPressed(KEY_FOUR))  launchAngle = 90.0f;

        // Launch position
        if (IsKeyDown(KEY_J)) launchPosition.x -= 3.0f * dt;
        if (IsKeyDown(KEY_L)) launchPosition.x += 3.0f * dt;
        if (IsKeyDown(KEY_I)) launchPosition.y += 3.0f * dt;
        if (IsKeyDown(KEY_K)) launchPosition.y -= 3.0f * dt;

        // Gravity magnitude / direction
        if (IsKeyDown(KEY_W)) gravityMagnitude += 5.0f * dt;
        if (IsKeyDown(KEY_S)) gravityMagnitude -= 5.0f * dt;
        if (IsKeyDown(KEY_A)) gravityAngle -= 45.0f * dt;
        if (IsKeyDown(KEY_D)) gravityAngle += 45.0f * dt;
        if (IsKeyPressed(KEY_G)) { gravityMagnitude = 9.81f; gravityAngle = -90.0f; }

        // Keep values in a sensible range
        launchSpeed = Clamp(launchSpeed, 1.0f, 20.0f);
        launchAngle = Clamp(launchAngle, 0.0f, 180.0f);
        launchPosition.x = Clamp(launchPosition.x, 0.0f, 19.0f);
        launchPosition.y = Clamp(launchPosition.y, 0.0f, 9.0f);
        gravityMagnitude = Clamp(gravityMagnitude, 0.0f, 30.0f);

        // Build the gravity vector from magnitude + direction
        sim.gravity = VectorFromAngle(gravityAngle, gravityMagnitude);

        // SPACE: launch a new ball (can be pressed again and again)
        if (IsKeyPressed(KEY_SPACE))
        {
            Vector2 launchVelocity = VectorFromAngle(launchAngle, launchSpeed);
            sim.AddBody(PhysicsBody(launchPosition, launchVelocity, ballColors[launchCount % colorCount]));
            launchCount++;
        }

        // C: clear all balls
        if (IsKeyPressed(KEY_C))
        {
            sim.Clear();
            launchCount = 0;
        }

        // ================= UPDATE =================
        sim.Update(dt);

        // ================= DRAW =================
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Grid (1 world unit per cell)
        for (int i = 0; i <= 20; i++)
        {
            Vector2 top = WorldToScreen({ (float)i, 10.0f });
            Vector2 bottom = WorldToScreen({ (float)i, 0.0f });
            DrawLineV(top, bottom, Fade(LIGHTGRAY, 0.6f));
            DrawText(TextFormat("%d", i), (int)bottom.x - 4, (int)ORIGIN_Y + 8, 16, GRAY);
        }
        for (int j = 1; j <= 10; j++)
        {
            Vector2 left = WorldToScreen({ 0.0f, (float)j });
            Vector2 right = WorldToScreen({ 20.0f, (float)j });
            DrawLineV(left, right, Fade(LIGHTGRAY, 0.6f));
            DrawText(TextFormat("%d", j), (int)ORIGIN_X - 30, (int)left.y - 8, 16, GRAY);
        }

        // Ground
        DrawLineEx({ 0.0f, ORIGIN_Y }, { (float)SCREEN_W, ORIGIN_Y }, 3.0f, DARKGRAY);

        // Green curve: predicted path with p = p0 + v0*t + 1/2*g*t^2
        {
            Vector2 v0 = VectorFromAngle(launchAngle, launchSpeed);
            Vector2 prev = WorldToScreen(launchPosition);
            for (float t = 0.02f; t < 10.0f; t += 0.02f)
            {
                Vector2 p = launchPosition + v0 * t + sim.gravity * (0.5f * t * t);
                if (p.y < 0.0f || p.x < -5.0f || p.x > 30.0f || p.y > 30.0f) break;

                Vector2 cur = WorldToScreen(p);
                DrawLineEx(prev, cur, 2.0f, Fade(GREEN, 0.6f));
                prev = cur;
            }
        }

        // Balls: trail dots (one per frame), ball, current velocity
        for (const PhysicsBody& b : sim.bodies)
        {
            for (const Vector2& p : b.trail)
            {
                DrawCircleV(WorldToScreen(p), 2.5f, Fade(b.color, 0.7f));
            }

            Vector2 ballScreen = WorldToScreen(b.position);
            DrawCircleV(ballScreen, 10.0f, b.color);

            if (b.active)
            {
                Vector2 tip = b.position + b.velocity * 0.25f;
                DrawLineEx(ballScreen, WorldToScreen(tip), 3.0f, BLUE);
            }
        }

        // Launch point (slingshot) + RED initial velocity vector
        Vector2 launchScreen = WorldToScreen(launchPosition);
        Vector2 launchTip = launchPosition + VectorFromAngle(launchAngle, launchSpeed) * 0.25f;
        DrawCircleV(launchScreen, 6.0f, BLACK);
        DrawLineEx(launchScreen, WorldToScreen(launchTip), 4.0f, RED);

        // Gravity arrow (top right)
        {
            Vector2 center = { 1150.0f, 330.0f };
            Vector2 g = sim.gravity * 4.0f;                    // pixels per (unit/s^2)
            Vector2 tip = { center.x + g.x, center.y - g.y };  // flip y for screen
            DrawCircleLinesV(center, 45.0f, LIGHTGRAY);
            DrawLineEx(center, tip, 3.0f, PURPLE);
            DrawCircleV(tip, 5.0f, PURPLE);
            DrawText("Gravity", 1115, 385, 18, PURPLE);
        }

        // HUD - values
        DrawText("GAME2005 Lab 2 - Launching Bird Simulation", 20, 15, 24, DARKGRAY);
        DrawText(TextFormat("launchPosition: (%.2f, %.2f)   (J/L, I/K)", launchPosition.x, launchPosition.y), 20, 50, 20, BLACK);
        DrawText(TextFormat("launchAngle: %.1f deg   (LEFT/RIGHT, 1=0 2=45 3=60 4=90)", launchAngle), 20, 75, 20, BLACK);
        DrawText(TextFormat("launchSpeed: %.2f u/s   (UP/DOWN)", launchSpeed), 20, 100, 20, BLACK);
        DrawText(TextFormat("gravity: %.2f u/s^2 at %.1f deg = (%.2f, %.2f)   (W/S, A/D, G reset)",
            gravityMagnitude, gravityAngle, sim.gravity.x, sim.gravity.y), 20, 125, 20, BLACK);
        DrawText(TextFormat("time: %.2f s   deltaTime: %.4f s   bodies: %d",
            sim.time, sim.deltaTime, (int)sim.bodies.size()), 20, 150, 20, BLACK);
        DrawText("SPACE: launch    C: clear", 20, 175, 20, DARKGRAY);

        // HUD - legend
        DrawText("Red: initial velocity vector", 900, 50, 18, RED);
        DrawText("Green: predicted path", 900, 72, 18, DARKGREEN);
        DrawText("Dots: one per frame", 900, 94, 18, GRAY);
        DrawText("Blue: current velocity", 900, 116, 18, BLUE);
        DrawText("Every frame:", 900, 150, 18, DARKGRAY);
        DrawText("v = v + g * dt", 920, 172, 18, DARKGRAY);
        DrawText("p = p + v * dt", 920, 194, 18, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

#include "raylib.h"
#include "raymath.h"   // Vector2 math + C++ operators (+, -, *, +=)
#include <vector>
#include <memory>      // std::shared_ptr, std::dynamic_pointer_cast
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

Vector2 ScreenToWorld(Vector2 s)
{
    return { (s.x - ORIGIN_X) / PX_PER_UNIT, (ORIGIN_Y - s.y) / PX_PER_UNIT };
}

// Turn an angle (degrees) + length into a vector
Vector2 VectorFromAngle(float angleDeg, float length)
{
    float rad = angleDeg * DEG2RAD;
    return { cosf(rad) * length, sinf(rad) * length };
}

// =====================================================
// Physics shapes: the shape + dimensions of a body
// (used only by the collision system)
// =====================================================
enum class ShapeType
{
    Sphere
    // Halfspace will be added in Lab 4
};

class PhysicsShape
{
public:
    virtual ~PhysicsShape() = default;
    virtual ShapeType GetType() const = 0;
};

// Sphere: every point closer than "radius" to the body's position.
// In this 2D view a sphere is drawn as a circle - the math is the same.
class Sphere : public PhysicsShape
{
public:
    float radius = 0.5f;   // units

    explicit Sphere(float r) : radius(r) {}

    ShapeType GetType() const override { return ShapeType::Sphere; }
};

// =====================================================
// PhysicsBody: one object in the simulation
// =====================================================
class PhysicsBody
{
public:
    Vector2 position = { 0.0f, 0.0f };   // units
    Vector2 velocity = { 0.0f, 0.0f };   // units per second
    float drag = 0.0f;                   // stored, not used yet (no air resistance)
    float mass = 1.0f;                   // kg, used in later labs

    std::shared_ptr<PhysicsShape> shape; // shape + dimensions for collision

    bool active = true;                  // false = stopped (landed / left the world)
    bool isStatic = false;               // true = simulation does not move it (test targets)
    bool isOverlapping = false;          // set by the collision system every frame

    Color color = DARKBLUE;              // original color
    std::vector<Vector2> trail;          // positions over time, only for drawing

    PhysicsBody(Vector2 startPos, Vector2 startVel, std::shared_ptr<PhysicsShape> s, Color c)
        : position(startPos), velocity(startVel), shape(s), color(c)
    {
        trail.push_back(position);
    }

    // Color to draw with: RED while overlapping, original color otherwise
    Color GetDrawColor() const
    {
        return isOverlapping ? RED : color;
    }

    // Returns the radius if this body is a sphere, 0 otherwise
    float GetSphereRadius() const
    {
        if (shape && shape->GetType() == ShapeType::Sphere)
            return std::static_pointer_cast<Sphere>(shape)->radius;
        return 0.0f;
    }
};

// =====================================================
// CollisionSystem: overlap tests between shapes
// =====================================================
class CollisionSystem
{
public:
    // Two spheres overlap when the distance between centers
    // is smaller than the sum of their radii.
    // Compare squared values -> no sqrt needed.
    static bool SphereSphereOverlap(Vector2 posA, float radiusA, Vector2 posB, float radiusB)
    {
        Vector2 diff = posB - posA;
        float distanceSq = diff.x * diff.x + diff.y * diff.y;
        float radiusSum = radiusA + radiusB;
        return distanceSq < radiusSum * radiusSum;
    }

    // Picks the right test based on the shape types of both bodies
    static bool CheckOverlap(const PhysicsBody& a, const PhysicsBody& b)
    {
        if (!a.shape || !b.shape) return false;

        ShapeType typeA = a.shape->GetType();
        ShapeType typeB = b.shape->GetType();

        if (typeA == ShapeType::Sphere && typeB == ShapeType::Sphere)
        {
            float rA = std::static_pointer_cast<Sphere>(a.shape)->radius;
            float rB = std::static_pointer_cast<Sphere>(b.shape)->radius;
            return SphereSphereOverlap(a.position, rA, b.position, rB);
        }

        return false; // shape pair not supported yet
    }
};

// =====================================================
// PhysicsSimulation: owns the bodies, moves them,
// then checks collisions
// =====================================================
class PhysicsSimulation
{
public:
    float deltaTime = 0.0f;                 // seconds, duration of the last frame
    float time = 0.0f;                      // seconds since the simulation started
    Vector2 gravity = { 0.0f, -9.81f };     // units per second^2 (direction + magnitude)

    std::vector<PhysicsBody> bodies;
    int overlapCount = 0;                   // overlapping pairs found last frame

    void AddBody(const PhysicsBody& body)
    {
        bodies.push_back(body);
    }

    // Removes only the launched (non-static) bodies
    void ClearLaunched()
    {
        std::vector<PhysicsBody> kept;
        for (const PhysicsBody& b : bodies)
            if (b.isStatic) kept.push_back(b);
        bodies = kept;
    }

    // One step forward in time. Same rules apply to every body.
    void Update(float dt)
    {
        deltaTime = dt;
        time += dt;

        MoveBodies();
        DetectCollisions();
    }

private:
    void MoveBodies()
    {
        for (PhysicsBody& b : bodies)
        {
            if (!b.active || b.isStatic) continue;

            // Kinematics (semi-implicit Euler):
            b.velocity += gravity * deltaTime;      // v = v + g * dt
            b.position += b.velocity * deltaTime;   // p = p + v * dt

            b.trail.push_back(b.position);

            // Stop the body when it reaches the ground or leaves the world
            // (not a real collision response - just so it stays on screen)
            float radius = b.GetSphereRadius();
            if (b.position.y <= radius)
            {
                b.position.y = radius;
                b.trail.back() = b.position;
                b.active = false;
            }
            else if (b.position.x < -5.0f || b.position.x > 30.0f || b.position.y > 30.0f)
            {
                b.active = false;
            }
        }
    }

    // Check every pair once (i < j). No collision response:
    // bodies pass through each other, we only mark them.
    void DetectCollisions()
    {
        for (PhysicsBody& b : bodies)
            b.isOverlapping = false;

        overlapCount = 0;

        for (size_t i = 0; i < bodies.size(); i++)
        {
            for (size_t j = i + 1; j < bodies.size(); j++)
            {
                if (CollisionSystem::CheckOverlap(bodies[i], bodies[j]))
                {
                    bodies[i].isOverlapping = true;
                    bodies[j].isOverlapping = true;
                    overlapCount++;
                }
            }
        }
    }
};

// Test spheres that hang in the air (static) - can be dragged with the mouse
void AddTestSpheres(PhysicsSimulation& sim)
{
    struct TestSphere { Vector2 pos; float radius; Color color; };
    const TestSphere tests[] =
    {
        { {  6.0f, 4.0f }, 0.8f, DARKBLUE  },
        { {  9.5f, 2.5f }, 1.2f, DARKGREEN },
        { { 12.0f, 6.0f }, 0.6f, PURPLE    },
        { { 15.0f, 3.0f }, 1.0f, BROWN     },
        { { 16.3f, 3.6f }, 0.8f, DARKGRAY  },   // starts overlapping the brown one
    };

    for (const TestSphere& t : tests)
    {
        PhysicsBody body(t.pos, { 0.0f, 0.0f }, std::make_shared<Sphere>(t.radius), t.color);
        body.isStatic = true;
        sim.AddBody(body);
    }
}

int main()
{
    InitWindow(SCREEN_W, SCREEN_H, "GAME2005 Lab 3 - Sphere-Sphere Overlap - Myroslav Pochynok");
    SetTargetFPS(60);

    PhysicsSimulation sim;
    AddTestSpheres(sim);

    // Launch settings
    Vector2 launchPosition = { 1.0f, 1.0f };  // units
    float launchAngle = 45.0f;                // degrees
    float launchSpeed = 10.0f;                // units per second
    float launchRadius = 0.3f;                // units, radius of the launched sphere

    // Gravity settings (turned into sim.gravity every frame)
    float gravityMagnitude = 9.81f;           // units per second^2
    float gravityAngle = -90.0f;              // degrees, -90 = straight down

    // No red-like colors here, so the RED overlap color is easy to see
    const Color ballColors[] = { DARKBLUE, ORANGE, DARKGREEN, PURPLE, BROWN, SKYBLUE, GOLD, DARKGRAY };
    const int colorCount = sizeof(ballColors) / sizeof(ballColors[0]);
    int launchCount = 0;

    int draggedIndex = -1;                    // index of the sphere held by the mouse

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        Vector2 mouseWorld = ScreenToWorld(GetMousePosition());

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

        // Launched sphere radius
        if (IsKeyDown(KEY_Q)) launchRadius -= 0.5f * dt;
        if (IsKeyDown(KEY_E)) launchRadius += 0.5f * dt;

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
        launchRadius = Clamp(launchRadius, 0.1f, 1.5f);
        gravityMagnitude = Clamp(gravityMagnitude, 0.0f, 30.0f);

        // Build the gravity vector from magnitude + direction
        sim.gravity = VectorFromAngle(gravityAngle, gravityMagnitude);

        // SPACE: launch a new sphere (can be pressed again and again)
        if (IsKeyPressed(KEY_SPACE))
        {
            Vector2 launchVelocity = VectorFromAngle(launchAngle, launchSpeed);
            sim.AddBody(PhysicsBody(launchPosition, launchVelocity,
                std::make_shared<Sphere>(launchRadius), ballColors[launchCount % colorCount]));
            launchCount++;
        }

        // C: clear launched spheres, R: reset test spheres too
        if (IsKeyPressed(KEY_C))
        {
            sim.ClearLaunched();
            launchCount = 0;
            draggedIndex = -1;
        }
        if (IsKeyPressed(KEY_R))
        {
            sim.bodies.clear();
            AddTestSpheres(sim);
            launchCount = 0;
            draggedIndex = -1;
        }

        // Mouse: find the sphere under the cursor (point inside sphere test)
        int hoveredIndex = -1;
        for (int i = (int)sim.bodies.size() - 1; i >= 0; i--)
        {
            const PhysicsBody& b = sim.bodies[i];
            float r = b.GetSphereRadius();
            if (Vector2DistanceSqr(mouseWorld, b.position) < r * r)
            {
                hoveredIndex = i;
                break;
            }
        }

        // LEFT MOUSE: drag a sphere (it stops being moved by gravity)
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hoveredIndex != -1)
        {
            draggedIndex = hoveredIndex;
            sim.bodies[draggedIndex].isStatic = true;
            sim.bodies[draggedIndex].velocity = { 0.0f, 0.0f };
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            draggedIndex = -1;
        }
        if (draggedIndex != -1)
        {
            sim.bodies[draggedIndex].position = mouseWorld;
        }

        // MOUSE WHEEL: change the radius of the sphere under the cursor
        int resizeIndex = (draggedIndex != -1) ? draggedIndex : hoveredIndex;
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f && resizeIndex != -1)
        {
            PhysicsBody& b = sim.bodies[resizeIndex];
            if (b.shape && b.shape->GetType() == ShapeType::Sphere)
            {
                auto sphere = std::static_pointer_cast<Sphere>(b.shape);
                sphere->radius = Clamp(sphere->radius + wheel * 0.1f, 0.1f, 3.0f);
            }
        }

        // ================= UPDATE =================
        sim.Update(dt);   // move bodies, then detect overlaps

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

        // Trails first, so spheres are drawn on top
        for (const PhysicsBody& b : sim.bodies)
        {
            for (const Vector2& p : b.trail)
                DrawCircleV(WorldToScreen(p), 2.0f, Fade(b.color, 0.5f));
        }

        // Thin red line between centers of every overlapping pair
        for (size_t i = 0; i < sim.bodies.size(); i++)
        {
            for (size_t j = i + 1; j < sim.bodies.size(); j++)
            {
                if (CollisionSystem::CheckOverlap(sim.bodies[i], sim.bodies[j]))
                    DrawLineEx(WorldToScreen(sim.bodies[i].position),
                        WorldToScreen(sim.bodies[j].position), 2.0f, Fade(RED, 0.6f));
            }
        }

        // Spheres: RED when overlapping, original color when not
        for (int i = 0; i < (int)sim.bodies.size(); i++)
        {
            const PhysicsBody& b = sim.bodies[i];
            Vector2 center = WorldToScreen(b.position);
            float radiusPx = b.GetSphereRadius() * PX_PER_UNIT;

            DrawCircleV(center, radiusPx, Fade(b.GetDrawColor(), 0.75f));
            DrawCircleLinesV(center, radiusPx, BLACK);
            DrawCircleV(center, 3.0f, BLACK);   // center point

            if (i == hoveredIndex || i == draggedIndex)
            {
                DrawCircleLinesV(center, radiusPx + 3.0f, YELLOW);
                DrawText(TextFormat("r = %.2f", b.GetSphereRadius()),
                    (int)center.x - 25, (int)(center.y - radiusPx - 22), 16, BLACK);
            }

            if (b.active && !b.isStatic)
            {
                Vector2 tip = b.position + b.velocity * 0.25f;
                DrawLineEx(center, WorldToScreen(tip), 3.0f, BLUE);
            }
        }

        // Launch point (slingshot) + RED initial velocity vector
        Vector2 launchScreen = WorldToScreen(launchPosition);
        Vector2 launchTip = launchPosition + VectorFromAngle(launchAngle, launchSpeed) * 0.25f;
        DrawCircleLinesV(launchScreen, launchRadius * PX_PER_UNIT, Fade(BLACK, 0.4f)); // size preview
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
        DrawText("GAME2005 Lab 3 - Sphere-Sphere Overlap", 20, 15, 24, DARKGRAY);
        DrawText(TextFormat("launchPosition: (%.2f, %.2f)   (J/L, I/K)", launchPosition.x, launchPosition.y), 20, 50, 20, BLACK);
        DrawText(TextFormat("launchAngle: %.1f deg   (LEFT/RIGHT, 1=0 2=45 3=60 4=90)", launchAngle), 20, 75, 20, BLACK);
        DrawText(TextFormat("launchSpeed: %.2f u/s   (UP/DOWN)", launchSpeed), 20, 100, 20, BLACK);
        DrawText(TextFormat("launchRadius: %.2f u   (Q/E)", launchRadius), 20, 125, 20, BLACK);
        DrawText(TextFormat("gravity: %.2f u/s^2 at %.1f deg = (%.2f, %.2f)   (W/S, A/D, G reset)",
            gravityMagnitude, gravityAngle, sim.gravity.x, sim.gravity.y), 20, 150, 20, BLACK);
        DrawText(TextFormat("time: %.2f s   deltaTime: %.4f s   bodies: %d   overlapping pairs: %d",
            sim.time, sim.deltaTime, (int)sim.bodies.size(), sim.overlapCount), 20, 175, 20, BLACK);
        DrawText("SPACE: launch   C: clear launched   R: reset all   Mouse: drag sphere, wheel = radius",
            20, 200, 20, DARKGRAY);

        // HUD - legend
        DrawText("Overlap test:", 900, 50, 18, DARKGRAY);
        DrawText("d = |posB - posA|", 920, 72, 18, DARKGRAY);
        DrawText("overlap if d < rA + rB", 920, 94, 18, DARKGRAY);
        DrawText("RED = overlapping", 900, 122, 18, RED);
        DrawText("Red line: overlapping pair", 900, 144, 18, RED);
        DrawText("Blue: current velocity", 900, 166, 18, BLUE);
        DrawText("Green: predicted path", 900, 188, 18, DARKGREEN);

        // Name - bottom left
        DrawText("Myroslav Pochynok", 20, SCREEN_H - 30, 20, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
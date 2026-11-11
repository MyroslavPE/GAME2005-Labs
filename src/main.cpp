#include "raylib.h"
#include "raymath.h"   // Vector2 math + C++ operators (+, -, *, +=)
#include <vector>
#include <memory>      // std::shared_ptr
#include <cmath>

// ---------------- Screen settings ----------------
const int SCREEN_W = 1280;
const int SCREEN_H = 720;

// World -> screen conversion
// World: units, y goes UP. Screen: pixels, y goes DOWN.
const float PX_PER_UNIT = 60.0f;
const float ORIGIN_X = 80.0f;   // screen x of world x = 0
const float ORIGIN_Y = 640.0f;  // screen y of world y = 0

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
// =====================================================
enum class ShapeType
{
    Sphere,
    Halfspace
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

// Halfspace: an infinite plane through the body's position,
// plus EVERYTHING behind it (the side opposite to the normal).
// In 2D the plane is a line and the halfspace is half of the screen.
class Halfspace : public PhysicsShape
{
public:
    float angleDeg = 0.0f;   // tilt of the surface: 0 = flat floor, 30 = rises to the right

    explicit Halfspace(float angle) : angleDeg(angle) {}

    ShapeType GetType() const override { return ShapeType::Halfspace; }

    // Unit normal: points OUT of the solid side.
    // angle 0 -> (0, 1) = straight up
    Vector2 GetNormal() const
    {
        float rad = angleDeg * DEG2RAD;
        return { -sinf(rad), cosf(rad) };
    }

    // Direction along the surface (perpendicular to the normal)
    Vector2 GetTangent() const
    {
        float rad = angleDeg * DEG2RAD;
        return { cosf(rad), sinf(rad) };
    }
};

// =====================================================
// PhysicsBody: one object in the simulation
// =====================================================
class PhysicsBody
{
public:
    Vector2 position = { 0.0f, 0.0f };   // units (for a halfspace: a point on its plane)
    Vector2 velocity = { 0.0f, 0.0f };   // units per second
    float drag = 0.0f;                   // stored, not used yet (no air resistance)
    float mass = 1.0f;                   // kg

    std::shared_ptr<PhysicsShape> shape; // shape + dimensions for collision

    bool active = true;                  // false = left the world, stops updating
    bool isFixed = false;                // true = static, never moved by gravity or collisions
    bool isHeld = false;                 // true while the mouse is dragging it
    bool isOverlapping = false;          // set by the collision system every frame

    Color color = DARKBLUE;              // original color
    std::vector<Vector2> trail;          // recent positions, only for drawing

    PhysicsBody(Vector2 startPos, Vector2 startVel, std::shared_ptr<PhysicsShape> s, Color c)
        : position(startPos), velocity(startVel), shape(s), color(c)
    {
    }

    // 0 = cannot be pushed (fixed or held by the mouse)
    float GetInverseMass() const
    {
        if (isFixed || isHeld || mass <= 0.0f) return 0.0f;
        return 1.0f / mass;
    }

    // Color to draw with: RED while overlapping, original color otherwise
    Color GetDrawColor() const
    {
        return isOverlapping ? RED : color;
    }

    bool IsSphere() const    { return shape && shape->GetType() == ShapeType::Sphere; }
    bool IsHalfspace() const { return shape && shape->GetType() == ShapeType::Halfspace; }

    Sphere* AsSphere() const       { return IsSphere() ? static_cast<Sphere*>(shape.get()) : nullptr; }
    Halfspace* AsHalfspace() const { return IsHalfspace() ? static_cast<Halfspace*>(shape.get()) : nullptr; }
};

// =====================================================
// Contact: result of one collision test
// =====================================================
struct Contact
{
    bool hit = false;
    Vector2 normal = { 0.0f, 1.0f };  // unit vector pointing from body A towards body B
    float penetration = 0.0f;         // how deep they overlap, in units
};

// =====================================================
// CollisionSystem: overlap tests + collision response
// =====================================================
class CollisionSystem
{
public:
    // Sphere vs Sphere:
    // overlap when distance between centers < sum of radii
    static Contact SphereSphere(Vector2 posA, float radiusA, Vector2 posB, float radiusB)
    {
        Contact c;
        Vector2 diff = posB - posA;
        float distanceSq = diff.x * diff.x + diff.y * diff.y;
        float radiusSum = radiusA + radiusB;

        if (distanceSq >= radiusSum * radiusSum) return c;   // no overlap

        float distance = sqrtf(distanceSq);
        c.hit = true;
        c.penetration = radiusSum - distance;
        // Same center -> any direction works, push straight up
        c.normal = (distance > 0.0001f) ? diff * (1.0f / distance) : Vector2{ 0.0f, 1.0f };
        return c;
    }

    // Sphere (A) vs Halfspace (B):
    // signed distance from the plane to the sphere center = dot(center - planePoint, normal)
    // overlap when that distance < radius (negative = center is inside the solid side)
    static Contact SphereHalfspace(Vector2 spherePos, float radius, Vector2 planePoint, Vector2 planeNormal)
    {
        Contact c;
        float distance = Vector2DotProduct(spherePos - planePoint, planeNormal);

        if (distance >= radius) return c;   // fully in front of the plane

        c.hit = true;
        c.penetration = radius - distance;
        c.normal = planeNormal * -1.0f;     // from the sphere (A) into the halfspace (B)
        return c;
    }

    // Picks the right test based on the shape types of both bodies
    static Contact Check(const PhysicsBody& a, const PhysicsBody& b)
    {
        if (a.IsSphere() && b.IsSphere())
            return SphereSphere(a.position, a.AsSphere()->radius, b.position, b.AsSphere()->radius);

        if (a.IsSphere() && b.IsHalfspace())
            return SphereHalfspace(a.position, a.AsSphere()->radius, b.position, b.AsHalfspace()->GetNormal());

        if (a.IsHalfspace() && b.IsSphere())
        {
            // Same test with A and B swapped, then flip the normal back to "A towards B"
            Contact c = SphereHalfspace(b.position, b.AsSphere()->radius, a.position, a.AsHalfspace()->GetNormal());
            c.normal = c.normal * -1.0f;
            return c;
        }

        return Contact{};   // halfspace vs halfspace: not needed (both are always fixed)
    }

    // Collision response (separation):
    // move the bodies apart along the normal until they just touch.
    // The push is shared by inverse mass: a fixed body (inverse mass 0) never moves.
    static void Separate(PhysicsBody& a, PhysicsBody& b, const Contact& c, bool removeApproachVelocity)
    {
        float invA = a.GetInverseMass();
        float invB = b.GetInverseMass();
        float invSum = invA + invB;
        if (invSum <= 0.0f) return;   // both fixed: nothing can move

        a.position -= c.normal * (c.penetration * invA / invSum);
        b.position += c.normal * (c.penetration * invB / invSum);

        if (!removeApproachVelocity) return;

        // Optional: cancel the part of the velocity that drives INTO the other body.
        // Without it, gravity keeps growing the velocity while the body rests,
        // and fast bodies start tunneling through each other.
        if (invA > 0.0f)
        {
            float into = Vector2DotProduct(a.velocity, c.normal);       // A moving towards B?
            if (into > 0.0f) a.velocity -= c.normal * into;
        }
        if (invB > 0.0f)
        {
            float into = Vector2DotProduct(b.velocity, c.normal);       // B moving towards A?
            if (into < 0.0f) b.velocity -= c.normal * into;
        }
    }
};

// =====================================================
// PhysicsSimulation: owns the bodies, moves them,
// then detects and resolves collisions
// =====================================================
class PhysicsSimulation
{
public:
    float deltaTime = 0.0f;                 // seconds, duration of the last frame
    float time = 0.0f;                      // seconds since the simulation started
    Vector2 gravity = { 0.0f, -9.81f };     // units per second^2 (direction + magnitude)

    bool collisionResponse = true;          // false = Lab 4 behaviour (overlap only)
    bool removeApproachVelocity = true;     // see CollisionSystem::Separate
    int solverIterations = 6;               // resolve passes per frame (helps piles of spheres)

    std::vector<PhysicsBody> bodies;
    int contactCount = 0;                   // touching pairs found this frame (before separating)
    int overlapCount = 0;                   // pairs still overlapping at the end of the frame

    void AddBody(const PhysicsBody& body)
    {
        bodies.push_back(body);
    }

    // Removes only the moving (non-fixed) spheres
    void ClearDynamic()
    {
        std::vector<PhysicsBody> kept;
        for (const PhysicsBody& b : bodies)
            if (b.isFixed) kept.push_back(b);
        bodies = kept;
    }

    // One step forward in time. Same rules apply to every body.
    void Update(float dt)
    {
        deltaTime = dt;
        time += dt;

        MoveBodies();
        HandleCollisions();
        RecordTrails();
    }

private:
    void MoveBodies()
    {
        for (PhysicsBody& b : bodies)
        {
            if (!b.active || b.isFixed || b.isHeld) continue;

            // Kinematics (semi-implicit Euler):
            b.velocity += gravity * deltaTime;      // v = v + g * dt
            b.position += b.velocity * deltaTime;   // p = p + v * dt

            // Far outside the world -> stop updating it
            if (b.position.x < -10.0f || b.position.x > 35.0f ||
                b.position.y < -20.0f || b.position.y > 40.0f)
            {
                b.active = false;
            }
        }
    }

    void HandleCollisions()
    {
        for (PhysicsBody& b : bodies)
            b.isOverlapping = false;

        overlapCount = 0;
        contactCount = 0;

        int passes = collisionResponse ? solverIterations : 1;

        for (int pass = 0; pass < passes; pass++)
        {
            // Check every pair once (i < j)
            for (size_t i = 0; i < bodies.size(); i++)
            {
                for (size_t j = i + 1; j < bodies.size(); j++)
                {
                    PhysicsBody& a = bodies[i];
                    PhysicsBody& b = bodies[j];
                    if (!a.active || !b.active) continue;

                    Contact c = CollisionSystem::Check(a, b);
                    if (!c.hit) continue;

                    if (pass == 0) contactCount++;

                    // Colors come from the LAST pass:
                    // response OFF -> every overlap is red (Lab 4)
                    // response ON  -> red only if the overlap is still there after separating
                    if (pass == passes - 1 && c.penetration > (collisionResponse ? 0.02f : 0.0f))
                    {
                        a.isOverlapping = true;
                        b.isOverlapping = true;
                        overlapCount++;
                    }

                    if (collisionResponse)
                        CollisionSystem::Separate(a, b, c, removeApproachVelocity);
                }
            }
        }
    }

    void RecordTrails()
    {
        const size_t maxTrail = 60;
        for (PhysicsBody& b : bodies)
        {
            if (!b.IsSphere() || b.isFixed || !b.active) continue;
            b.trail.push_back(b.position);
            if (b.trail.size() > maxTrail) b.trail.erase(b.trail.begin());
        }
    }
};

// ---------------- Scene setup ----------------
const Color sphereColors[] = { DARKBLUE, ORANGE, DARKGREEN, PURPLE, BROWN, SKYBLUE, GOLD, LIME, VIOLET, DARKGRAY };
const int sphereColorCount = sizeof(sphereColors) / sizeof(sphereColors[0]);

void AddHalfspace(PhysicsSimulation& sim, Vector2 point, float angleDeg, Color c)
{
    PhysicsBody body(point, { 0.0f, 0.0f }, std::make_shared<Halfspace>(angleDeg), c);
    body.isFixed = true;
    sim.AddBody(body);
}

void AddFixedSphere(PhysicsSimulation& sim, Vector2 pos, float radius, Color c)
{
    PhysicsBody body(pos, { 0.0f, 0.0f }, std::make_shared<Sphere>(radius), c);
    body.isFixed = true;
    sim.AddBody(body);
}

void AddDynamicSphere(PhysicsSimulation& sim, Vector2 pos, Vector2 vel, float radius, Color c)
{
    sim.AddBody(PhysicsBody(pos, vel, std::make_shared<Sphere>(radius), c));
}

// A "valley" of static halfspaces at different angles + two fixed spheres
void BuildScene(PhysicsSimulation& sim)
{
    sim.bodies.clear();

    AddHalfspace(sim, { 9.5f, 0.0f },   0.0f, DARKGRAY);    // flat floor
    AddHalfspace(sim, { 8.0f, 0.0f }, -25.0f, DARKBROWN);   // left slope (rises to the left)
    AddHalfspace(sim, { 11.0f, 0.0f }, 40.0f, DARKBLUE);    // right slope (rises to the right)

    AddFixedSphere(sim, { 6.5f, 6.0f }, 0.7f, DARKGREEN);
    AddFixedSphere(sim, { 13.0f, 5.5f }, 0.9f, PURPLE);
}

// B: several spheres fall from the top
void DropSpheres(PhysicsSimulation& sim, int count, int& colorIndex)
{
    for (int i = 0; i < count; i++)
    {
        Vector2 pos = { GetRandomValue(400, 1500) / 100.0f, GetRandomValue(800, 1000) / 100.0f };
        float radius = GetRandomValue(30, 60) / 100.0f;
        AddDynamicSphere(sim, pos, { 0.0f, 0.0f }, radius, sphereColors[colorIndex % sphereColorCount]);
        colorIndex++;
    }
}

// raylib only fills triangles with one winding order (back-face culling),
// so draw both orders - exactly one of them is visible.
void DrawTriangleAnyOrder(Vector2 a, Vector2 b, Vector2 c, Color col)
{
    DrawTriangle(a, b, c, col);
    DrawTriangle(a, c, b, col);
}

void DrawHalfspace(const PhysicsBody& b)
{
    const Halfspace* h = b.AsHalfspace();
    Vector2 n = h->GetNormal();
    Vector2 t = h->GetTangent();
    const float L = 100.0f;   // "infinite" for this screen

    // Solid side: a huge quad behind the plane
    Vector2 p1 = WorldToScreen(b.position - t * L);
    Vector2 p2 = WorldToScreen(b.position + t * L);
    Vector2 p3 = WorldToScreen(b.position + t * L - n * L);
    Vector2 p4 = WorldToScreen(b.position - t * L - n * L);
    Color fill = Fade(b.GetDrawColor(), 0.12f);
    DrawTriangleAnyOrder(p1, p2, p3, fill);
    DrawTriangleAnyOrder(p1, p3, p4, fill);

    // The plane itself (a line in 2D)
    DrawLineEx(p1, p2, 3.0f, b.GetDrawColor());
}

// Handle point + normal arrow (drawn last, so spheres never hide it)
void DrawHalfspaceHandle(const PhysicsBody& b, bool highlighted)
{
    const Halfspace* h = b.AsHalfspace();
    Vector2 n = h->GetNormal();
    Vector2 handle = WorldToScreen(b.position);
    DrawLineEx(handle, WorldToScreen(b.position + n * 0.8f), 2.0f, b.GetDrawColor());
    DrawCircleV(handle, 7.0f, highlighted ? YELLOW : b.GetDrawColor());
    DrawCircleLinesV(handle, 7.0f, BLACK);
    if (highlighted)
        DrawText(TextFormat("angle %.0f deg", h->angleDeg), (int)handle.x - 40, (int)handle.y + 12, 16, BLACK);
}

int main()
{
    InitWindow(SCREEN_W, SCREEN_H, "GAME2005 Lab 5 - Collision Response - Myroslav Pochynok");
    SetTargetFPS(60);

    PhysicsSimulation sim;
    BuildScene(sim);

    // Launch settings
    Vector2 launchPosition = { 1.0f, 5.0f };  // units
    float launchAngle = 30.0f;                // degrees
    float launchSpeed = 8.0f;                 // units per second
    float launchRadius = 0.35f;               // units

    // Gravity settings (turned into sim.gravity every frame)
    float gravityMagnitude = 9.81f;           // units per second^2
    float gravityAngle = -90.0f;              // degrees, -90 = straight down

    int colorIndex = 0;
    int heldIndex = -1;                       // body held by the mouse

    while (!WindowShouldClose())
    {
        // Cap dt so a lag spike (e.g. dragging the window) can't make bodies jump through things
        float dt = fminf(GetFrameTime(), 1.0f / 30.0f);
        Vector2 mouseWorld = ScreenToWorld(GetMousePosition());

        // ================= INPUT =================
        // Launch speed / angle / position / radius
        if (IsKeyDown(KEY_UP))    launchSpeed += 5.0f * dt;
        if (IsKeyDown(KEY_DOWN))  launchSpeed -= 5.0f * dt;
        if (IsKeyDown(KEY_LEFT))  launchAngle += 30.0f * dt;
        if (IsKeyDown(KEY_RIGHT)) launchAngle -= 30.0f * dt;
        if (IsKeyDown(KEY_J)) launchPosition.x -= 3.0f * dt;
        if (IsKeyDown(KEY_L)) launchPosition.x += 3.0f * dt;
        if (IsKeyDown(KEY_I)) launchPosition.y += 3.0f * dt;
        if (IsKeyDown(KEY_K)) launchPosition.y -= 3.0f * dt;
        if (IsKeyDown(KEY_Q)) launchRadius -= 0.5f * dt;
        if (IsKeyDown(KEY_E)) launchRadius += 0.5f * dt;

        // Gravity magnitude / direction
        if (IsKeyDown(KEY_W)) gravityMagnitude += 5.0f * dt;
        if (IsKeyDown(KEY_S)) gravityMagnitude -= 5.0f * dt;
        if (IsKeyDown(KEY_A)) gravityAngle -= 45.0f * dt;
        if (IsKeyDown(KEY_D)) gravityAngle += 45.0f * dt;
        if (IsKeyPressed(KEY_G)) { gravityMagnitude = 9.81f; gravityAngle = -90.0f; }

        launchSpeed = Clamp(launchSpeed, 1.0f, 20.0f);
        launchAngle = Clamp(launchAngle, -90.0f, 180.0f);
        launchPosition.x = Clamp(launchPosition.x, 0.0f, 19.0f);
        launchPosition.y = Clamp(launchPosition.y, 0.0f, 9.5f);
        launchRadius = Clamp(launchRadius, 0.1f, 1.5f);
        gravityMagnitude = Clamp(gravityMagnitude, 0.0f, 30.0f);

        sim.gravity = VectorFromAngle(gravityAngle, gravityMagnitude);

        // Toggles
        if (IsKeyPressed(KEY_T)) sim.collisionResponse = !sim.collisionResponse;
        if (IsKeyPressed(KEY_V)) sim.removeApproachVelocity = !sim.removeApproachVelocity;

        // SPACE: launch one sphere, B: drop several spheres
        if (IsKeyPressed(KEY_SPACE))
        {
            AddDynamicSphere(sim, launchPosition, VectorFromAngle(launchAngle, launchSpeed),
                launchRadius, sphereColors[colorIndex % sphereColorCount]);
            colorIndex++;
        }
        if (IsKeyPressed(KEY_B)) DropSpheres(sim, 6, colorIndex);

        // C: clear moving spheres, R: reset the whole scene
        if (IsKeyPressed(KEY_C)) { sim.ClearDynamic(); heldIndex = -1; }
        if (IsKeyPressed(KEY_R)) { BuildScene(sim); heldIndex = -1; colorIndex = 0; }

        // Mouse: what is under the cursor? (halfspace handles first, then spheres)
        int hoveredIndex = -1;
        for (int i = (int)sim.bodies.size() - 1; i >= 0 && hoveredIndex == -1; i--)
        {
            const PhysicsBody& b = sim.bodies[i];
            if (b.IsHalfspace() && Vector2Distance(mouseWorld, b.position) < 0.25f)
                hoveredIndex = i;
        }
        for (int i = (int)sim.bodies.size() - 1; i >= 0 && hoveredIndex == -1; i--)
        {
            const PhysicsBody& b = sim.bodies[i];
            if (b.IsSphere() && Vector2Distance(mouseWorld, b.position) < b.AsSphere()->radius)
                hoveredIndex = i;
        }

        // LEFT MOUSE: drag a body. While held it acts like a fixed body and pushes others.
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hoveredIndex != -1)
        {
            heldIndex = hoveredIndex;
            sim.bodies[heldIndex].isHeld = true;
            sim.bodies[heldIndex].velocity = { 0.0f, 0.0f };
            sim.bodies[heldIndex].active = true;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && heldIndex != -1)
        {
            sim.bodies[heldIndex].isHeld = false;
            sim.bodies[heldIndex].velocity = { 0.0f, 0.0f };
            heldIndex = -1;
        }
        if (heldIndex != -1)
            sim.bodies[heldIndex].position = mouseWorld;

        // F: fix / unfix the sphere under the cursor
        if (IsKeyPressed(KEY_F) && hoveredIndex != -1 && sim.bodies[hoveredIndex].IsSphere())
        {
            PhysicsBody& b = sim.bodies[hoveredIndex];
            b.isFixed = !b.isFixed;
            b.velocity = { 0.0f, 0.0f };
            b.trail.clear();
        }

        // MOUSE WHEEL: sphere -> radius, halfspace -> angle
        int wheelIndex = (heldIndex != -1) ? heldIndex : hoveredIndex;
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f && wheelIndex != -1)
        {
            PhysicsBody& b = sim.bodies[wheelIndex];
            if (b.IsSphere())
                b.AsSphere()->radius = Clamp(b.AsSphere()->radius + wheel * 0.1f, 0.1f, 3.0f);
            else if (b.IsHalfspace())
                b.AsHalfspace()->angleDeg = Clamp(b.AsHalfspace()->angleDeg + wheel * 5.0f, -80.0f, 80.0f);
        }

        // ================= UPDATE =================
        sim.Update(dt);   // move -> detect -> separate

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
        }

        // Halfspaces
        for (int i = 0; i < (int)sim.bodies.size(); i++)
            if (sim.bodies[i].IsHalfspace())
                DrawHalfspace(sim.bodies[i]);

        // Green curve: predicted path with p = p0 + v0*t + 1/2*g*t^2 (ignores collisions)
        {
            Vector2 v0 = VectorFromAngle(launchAngle, launchSpeed);
            Vector2 prev = WorldToScreen(launchPosition);
            for (float t = 0.02f; t < 4.0f; t += 0.02f)
            {
                Vector2 p = launchPosition + v0 * t + sim.gravity * (0.5f * t * t);
                if (p.y < -1.0f || p.x < -2.0f || p.x > 22.0f || p.y > 12.0f) break;

                Vector2 cur = WorldToScreen(p);
                DrawLineEx(prev, cur, 2.0f, Fade(GREEN, 0.5f));
                prev = cur;
            }
        }

        // Trails
        for (const PhysicsBody& b : sim.bodies)
            for (const Vector2& p : b.trail)
                DrawCircleV(WorldToScreen(p), 2.0f, Fade(b.color, 0.35f));

        // Spheres: RED when overlapping, original color when not
        for (int i = 0; i < (int)sim.bodies.size(); i++)
        {
            const PhysicsBody& b = sim.bodies[i];
            if (!b.IsSphere()) continue;

            Vector2 center = WorldToScreen(b.position);
            float radiusPx = b.AsSphere()->radius * PX_PER_UNIT;

            DrawCircleV(center, radiusPx, Fade(b.GetDrawColor(), 0.75f));
            DrawCircleLinesV(center, radiusPx, BLACK);

            if (b.isFixed)
            {
                // Fixed spheres get a thick ring + "F"
                DrawRing(center, radiusPx - 3.0f, radiusPx, 0.0f, 360.0f, 36, BLACK);
                DrawText("F", (int)center.x - 5, (int)center.y - 9, 20, WHITE);
            }
            else
            {
                DrawCircleV(center, 3.0f, BLACK);
                if (b.active)
                    DrawLineEx(center, WorldToScreen(b.position + b.velocity * 0.15f), 2.0f, BLUE);
            }

            if (i == hoveredIndex || i == heldIndex)
            {
                DrawCircleLinesV(center, radiusPx + 3.0f, YELLOW);
                DrawText(TextFormat("r = %.2f", b.AsSphere()->radius),
                    (int)center.x - 25, (int)(center.y - radiusPx - 22), 16, BLACK);
            }
        }

        // Halfspace handles
        for (int i = 0; i < (int)sim.bodies.size(); i++)
            if (sim.bodies[i].IsHalfspace())
                DrawHalfspaceHandle(sim.bodies[i], i == hoveredIndex || i == heldIndex);

        // Launch point + RED initial velocity vector + size preview
        Vector2 launchScreen = WorldToScreen(launchPosition);
        DrawCircleLinesV(launchScreen, launchRadius * PX_PER_UNIT, Fade(BLACK, 0.4f));
        DrawCircleV(launchScreen, 5.0f, BLACK);
        DrawLineEx(launchScreen, WorldToScreen(launchPosition + VectorFromAngle(launchAngle, launchSpeed) * 0.2f), 4.0f, RED);

        // Gravity arrow (top right)
        {
            Vector2 center = { 1180.0f, 300.0f };
            Vector2 g = sim.gravity * 4.0f;
            Vector2 tip = { center.x + g.x, center.y - g.y };
            DrawCircleLinesV(center, 45.0f, LIGHTGRAY);
            DrawLineEx(center, tip, 3.0f, PURPLE);
            DrawCircleV(tip, 5.0f, PURPLE);
            DrawText("Gravity", 1148, 352, 18, PURPLE);
        }

        // HUD - values
        int dynamicCount = 0;
        for (const PhysicsBody& b : sim.bodies) if (b.IsSphere() && !b.isFixed) dynamicCount++;

        DrawRectangle(10, 8, 760, 196, Fade(RAYWHITE, 0.85f));
        DrawText("GAME2005 Lab 5 - Collision Response (Separation)", 20, 12, 24, DARKGRAY);
        DrawText(TextFormat("Collision response: %s   (T)", sim.collisionResponse ? "ON  - separate" : "OFF - overlap only (Lab 4)"),
            20, 44, 20, sim.collisionResponse ? DARKGREEN : RED);
        DrawText(TextFormat("Remove velocity into surface: %s   (V)", sim.removeApproachVelocity ? "ON" : "OFF"),
            20, 68, 20, BLACK);
        DrawText(TextFormat("launch: pos (%.1f, %.1f)  angle %.0f  speed %.1f  radius %.2f",
            launchPosition.x, launchPosition.y, launchAngle, launchSpeed, launchRadius), 20, 92, 20, BLACK);
        DrawText(TextFormat("gravity: %.2f u/s^2 at %.0f deg   (W/S, A/D, G reset)", gravityMagnitude, gravityAngle), 20, 116, 20, BLACK);
        DrawText(TextFormat("time %.1f s  dt %.4f s  moving %d  contacts %d  still overlapping %d",
            sim.time, sim.deltaTime, dynamicCount, sim.contactCount, sim.overlapCount), 20, 140, 20, BLACK);
        DrawText("SPACE launch  B drop 6  F fix/unfix  Mouse drag  Wheel radius/angle", 20, 164, 18, DARKGRAY);
        DrawText("Arrows/JLIK/QE launcher   C clear   R reset", 20, 184, 18, DARKGRAY);

        // HUD - legend
        DrawText("RED = overlapping", 960, 14, 18, RED);
        DrawText("Shaded side = solid halfspace", 960, 36, 18, DARKGRAY);
        DrawText("F = fixed (static) sphere", 960, 58, 18, BLACK);
        DrawText("Blue = velocity", 960, 80, 18, BLUE);
        DrawText("Response: push apart", 960, 108, 18, DARKGRAY);
        DrawText("along normal by depth", 960, 128, 18, DARKGRAY);

        // Name - bottom left
        DrawText("Myroslav Pochynok", 20, SCREEN_H - 30, 20, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

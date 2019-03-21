#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>

#include <vector>
#include <time.h>
#include <iostream>

//Box coordinates
glm::vec3 boxCoordinatesInit{ -5, 0, -5 };
glm::vec3 boxCoordinatesFinal = { 5, 10, 5 };

//Straw coordinates
glm::vec3 strawCoordinatesInit{ -5, -1, -5 };
glm::vec3 strawCoordinatesFinal = { 5, -1, 5 };

//Straw settings
int numberOfStraws = 100;
float strawHeight = 5.0f;

//Spring parameters
glm::vec2 k_stretch{ 10, 5 };
glm::vec2 k_bend{ 0, 0 };
float particleLinkD = 0.8f;


namespace Box {
	void drawCube();
}
namespace Axis {
	void drawAxis();
}
namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
	extern void cleanupSphere();
	bool collider;
	float mass;
	glm::vec3 initPosition{ 0.0f, 1.0f, 0.0f };
	glm::vec3 position { 0.0f, 1.0f, 0.0f };
	float speed {0.0f};
	float turnRadius;
	extern float radius;
	void UniformCircularMotion(float dt)
	{
		static float alpha = 0;
		alpha += dt * speed;
		if (alpha > glm::two_pi<float>())
			alpha = 0;

		position.x = initPosition.x + turnRadius * glm::sin(alpha);
		position.z = initPosition.z + turnRadius * glm::cos(alpha);
	}
}
namespace Capsule {
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius = 1.f);
	extern void drawCapsule();
}
namespace Particles {
	extern const int maxParticles;
	extern void setupParticles(int numTotalParticles, float radius);
	extern void updateParticles(int startIdx, int count, float* array_data);
	extern void drawParticles(int startIdx, int count);
}
namespace Mesh {
	extern const int numCols;
	extern const int numRows;
	extern void updateMesh(float* array_data);
	extern void drawMesh();
}
namespace Fiber {
	extern const int numVerts;
	extern void updateFiber(float* array_data);
	extern void drawFiber();
}
namespace Cube {
	extern void updateCube(const glm::mat4& transform);
	extern void drawCube();
}

//ADDITIONAL FUNCTIONS
//Function that generates a random float between two numbers
float RandomFloat(float a, float b)
{
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = b - a;
	float r = random * diff;
	return a + r;
}

//Function that returns the module of a given vector
float vectorModule(const glm::vec3& A)
{
	return (glm::sqrt((glm::pow(A.x, 2)) + (glm::pow(A.y, 2)) + (glm::pow(A.z, 2))));
}

struct Spring
{
	bool stretch;
	int P1, P2;
	float L0;

	Spring(bool isStretch, int p1, int p2, float l0) : stretch(isStretch), P1(p1), P2(p2), L0(l0) {}
};

glm::vec3 springforce(const glm::vec3& P1, const glm::vec3& V1, const glm::vec3& P2, const glm::vec3& V2, float L0, float ke, float kd){	return -(ke*(vectorModule(P1 - P2) - L0) + kd * (V1 - V2)*((P1 - P2) / (vectorModule(P1 - P2))))*((P1 - P2) / (vectorModule(P1 - P2)));}

struct FiberStraw
{
public: 
	float *pos, *prevPos, *auxPos, *vel;
	glm::vec3 initPos;

	std::vector<Spring> springs;

	FiberStraw(float strawHeight)
	{
		pos = new float[Fiber::numVerts * 3];
		prevPos = new float[Fiber::numVerts * 3];
		auxPos = new float[Fiber::numVerts * 3];

		vel = new float[Fiber::numVerts * 3];

		InitStraw(strawHeight);
	}

	void InitStraw(float strawHeight)
	{
		initPos = { RandomFloat(strawCoordinatesInit.x, strawCoordinatesFinal.x), RandomFloat(strawCoordinatesInit.y, strawCoordinatesFinal.y), RandomFloat(strawCoordinatesInit.z, strawCoordinatesFinal.z) };

		pos[0] = initPos.x;
		pos[1] = initPos.y;
		pos[2] = initPos.z;

		int y = 1; 
		bool bend = false;
		for (int i = 1; i < Fiber::numVerts; i++)
		{
			pos[i * 3] = initPos.x;
			pos[i * 3 + 1] = initPos.y + y * (strawHeight / Fiber::numVerts);
			pos[i * 3 + 2] = initPos.z;

			springs.push_back({ true, ((i - 1) * 3), i * 3, (strawHeight / Fiber::numVerts)});

			if (bend)
				springs.push_back({ false, ((i - 2) * 3), i * 3, (strawHeight / Fiber::numVerts) * 2 });
			else 
				bend = true;
			y++;
		}	

		for (int i = 0; i < Fiber::numVerts * 3; i++)
		{
			prevPos[i] = pos[i];
			vel[i] = 0;
		}
	}

	void ResetStraw(float strawHeight)
	{
		InitStraw(strawHeight);
	}

	void DrawFiberStraw()
	{
		Fiber::updateFiber(pos);
		Fiber::drawFiber();
	}
};

glm::vec3 springForcePos(FiberStraw& fiber, int j)
{
	glm::vec3 p1 = { fiber.pos[fiber.springs[j].P1], fiber.pos[fiber.springs[j].P1 + 1],fiber.pos[fiber.springs[j].P1 + 2] };
	glm::vec3 p2 = { fiber.pos[fiber.springs[j].P2], fiber.pos[fiber.springs[j].P2 + 1], fiber.pos[fiber.springs[j].P2 + 2] };
	glm::vec3 v1 = { fiber.vel[fiber.springs[j].P1], fiber.vel[fiber.springs[j].P1 + 1], fiber.vel[fiber.springs[j].P1 + 2] };
	glm::vec3 v2 = { fiber.vel[fiber.springs[j].P2], fiber.vel[fiber.springs[j].P2 + 1], fiber.vel[fiber.springs[j].P2 + 2] };

	return springforce(p1, v1, p2, v2, fiber.springs[j].L0, fiber.springs[j].stretch ? k_stretch.x : k_bend.x, fiber.springs[j].stretch ? k_stretch.y : k_bend.y);
}

glm::vec3 springForcePrev(FiberStraw& fiber, int j)
{
	glm::vec3 p1 = { fiber.prevPos[fiber.springs[j].P1], fiber.prevPos[fiber.springs[j].P1 + 1],fiber.prevPos[fiber.springs[j].P1 + 2] };
	glm::vec3 p2 = { fiber.pos[fiber.springs[j].P2], fiber.pos[fiber.springs[j].P2 + 1], fiber.pos[fiber.springs[j].P2 + 2] };
	glm::vec3 v1 = { fiber.vel[fiber.springs[j].P1], fiber.vel[fiber.springs[j].P1 + 1], fiber.vel[fiber.springs[j].P1 + 2] };
	glm::vec3 v2 = { fiber.vel[fiber.springs[j].P2], fiber.vel[fiber.springs[j].P2 + 1], fiber.vel[fiber.springs[j].P2 + 2] };

	return springforce(p1, v1, p2, v2, fiber.springs[j].L0, fiber.springs[j].stretch ? k_stretch.x : k_bend.x, fiber.springs[j].stretch ? k_stretch.y : k_bend.y);
}

std::vector<FiberStraw> fiberStraws;

struct ForceActuator
{
	virtual glm::vec3 computeForce(float mass, const glm::vec3& position) = 0;
};

struct GravityForce : ForceActuator
{
	glm::vec3 force;
	bool useForce;

	GravityForce(const glm::vec3& f) : force(f) 
	{
		useForce = true;
	};

	glm::vec3 computeForce(float mass, const glm::vec3& position)
	{
		if (useForce)
		{
			return force;
		}
		return { 0,0,0 };
	}
};

glm::vec3 computeForces(FiberStraw& fiber, int idx, const std::vector<ForceActuator*>& force_acts)
{
	glm::vec3 force;

	for (int i = 0; i < force_acts.size(); i++)
		force += force_acts[i]->computeForce(1, {fiber.pos[idx * 3], fiber.pos[idx * 3 + 1], fiber.pos[idx * 3 + 2] });

	return force;
}

std::vector<ForceActuator*> forceActuators;

GravityForce* gravity = new GravityForce({ 0,-0.1f,0 });
GravityForce* wind = new GravityForce({ 0.1f, 0, 0 });

struct Collider
{
	virtual bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) = 0;
	virtual void getPlane(glm::vec3& normal, float& d) = 0;
	void computeCollision(glm::vec3& old_pos, glm::vec3& new_pos)
	{

	}
};

struct PlaneCol : Collider 
{ 

};

struct SphereCol : Collider 
{ 

};

std::vector<Collider*> colliders;

void verlet(float dt, FiberStraw& fiber, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts)
{
	glm::vec3 force;
	for (int i = 1; i < Fiber::numVerts; i++)
	{
		force = computeForces(fiber, i, force_acts);

		for (int j = 0; j < fiber.springs.size(); j++)
		{
			if (fiber.springs[j].P1 == i * 3)
			{
				glm::vec3 s = springForcePos(fiber, j);
				force += s;
			}
			else if (fiber.springs[j].P2 == i * 3)
			{
				glm::vec3 s = springForcePrev(fiber, j);
				force += -s;
			}
		}

		glm::vec3 pos = { fiber.pos[i * 3], fiber.pos[i * 3 + 1], fiber.pos[i * 3 + 2] };
		glm::vec3 prevPos = { fiber.prevPos[i * 3], fiber.prevPos[i * 3 + 1], fiber.prevPos[i * 3 + 2] };

		glm::vec3 nextPos = pos + (pos - prevPos) + (force / 1.0f) * dt * dt;

		fiber.prevPos[i * 3] = pos.x;
		fiber.prevPos[i * 3 + 1] = pos.y;
		fiber.prevPos[i * 3 + 2] = pos.z;

		fiber.pos[i * 3] = nextPos.x;
		fiber.pos[i * 3 + 1] = nextPos.y;
		fiber.pos[i * 3 + 2] = nextPos.z;
	}
}


// Boolean variables allow to show/hide the primitives
bool renderSphere = true;
bool renderCapsule = false;
bool renderParticles = true;
bool renderMesh = false;
bool renderFiber = true;
bool renderCube = false;

//You may have to change this code
void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();

	if (renderSphere)
		Sphere::drawSphere();
	if (renderCapsule)
		Capsule::drawCapsule();
	if (renderParticles) 
	{
		int startDrawingFromParticle = 0;
		int numParticlesToDraw = Particles::maxParticles;
		Particles::drawParticles(startDrawingFromParticle, numParticlesToDraw);
	}
	if (renderMesh)
		Mesh::drawMesh();
	if (renderFiber)
		for (int i = 0; i < fiberStraws.size(); i++)
			fiberStraws[i].DrawFiberStraw();
	if (renderCube)
		Cube::drawCube();
}


void Reset()
{
	for (int i = 0; i < fiberStraws.size(); i++)
		fiberStraws[i].InitStraw(strawHeight);
}

float dragPrecision = 0.01f;

bool play;

//Elasticity & friction
float elasticCoefficient = 0.009f;
float frictionCoefficient = 0.9f;

//Sphere
float sphereTurnRadius = 1.0f;
float sphereTurnSpeed = 1.0f;


void GUI() {
	bool show = true;

	ImGui::Begin("Physics Parameters", &show, 0);
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::Checkbox("Play simulation", &play); //

		if (ImGui::Button("Reset")) //
			Reset();

		if (ImGui::TreeNode("Spring parameters"))
		{
			ImGui::DragFloat2("k_strecth", &k_stretch.x, dragPrecision); //
			ImGui::DragFloat2("k_bend", &k_bend.x, dragPrecision); //
			ImGui::DragFloat("Particle Link D", &particleLinkD, dragPrecision); //
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Elasticity & friction"))
		{
			ImGui::DragFloat("Elastic Coefficient", &elasticCoefficient, dragPrecision); //
			ImGui::DragFloat("Friction Coefficient", &frictionCoefficient, dragPrecision); //
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Sphere"))
		{
			ImGui::Checkbox("Show Sphere", &renderSphere);
			ImGui::Checkbox("Use Sphere Collider", &Sphere::collider);
			ImGui::DragFloat("Sphere Y", &Sphere::position.y, dragPrecision);
			ImGui::DragFloat("Sphere Turn Radius", &Sphere::turnRadius, dragPrecision); 
			ImGui::DragFloat("Sphere Turn Speed", &Sphere::speed, dragPrecision); 
			ImGui::DragFloat("Sphere Radius", &Sphere::radius, dragPrecision);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Forces"))
		{
			ImGui::Checkbox("Use gravity", &gravity->useForce); 
			ImGui::DragFloat3("Gravity Accel", &gravity->force.x, dragPrecision); 
			ImGui::Checkbox("Use wind", &wind->useForce); 
			ImGui::DragFloat3("Wind Accel", &wind->force.x, dragPrecision); 
			ImGui::TreePop();
		}
	}
	ImGui::End();
}

void PhysicsInit() 
{
	srand(time(NULL));

	for (int i = 0; i < numberOfStraws; i++)
		fiberStraws.push_back({ strawHeight });

	forceActuators.push_back(gravity);
	forceActuators.push_back(wind);
}

void PhysicsUpdate(float dt) 
{
	Sphere::UniformCircularMotion(dt);
	Sphere::updateSphere(Sphere::position, Sphere::radius);

	for (int j = 0; j < 5; j++)
		for (int i = 0; i < fiberStraws.size(); i++)
			verlet(dt / 5, fiberStraws[i], colliders, forceActuators);
}

void PhysicsCleanup() {
	Sphere::cleanupSphere();
}
#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>

#include <vector>
#include <time.h>
#include <iostream>

double G = 0.00000000006674F;

glm::vec3 boxCoordinatesInit = { -5, 5, -5 };
glm::vec3 boxCoordinatesFinal = { 5, 10, 5 };

float dragPrecision = 0.01f;

float elasticCoefficient = 0.0f;
float frictionCoefficient = 0.0f;

bool useGravity;

namespace Box {
	void drawCube();
}
namespace Axis {
	void drawAxis();
}

namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();

	bool collider;

	float mass;
	glm::vec3 position {0.0f, 1.0f, 0.0f};
	extern float radius;
}
namespace Capsule {
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius = 1.f);
	extern void drawCapsule();

	bool collider;
	
	glm::vec3 positionA{ -3.f, 2.f, -2.f };
	glm::vec3 positionB{ -4.f, 2.f, 2.f };
	extern float radius;
}
namespace Particles {
	extern const int maxParticles;
	extern void setupParticles(int numTotalParticles, float radius);
	extern void updateParticles(int startIdx, int count, float* array_data);
	extern void drawParticles(int startIdx, int count);
	extern void cleanupParticles();
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

//Useful Functions
//Function that generates a random float between two numbers
float RandomFloat(float a, float b)
{
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = b - a;
	float r = random * diff;
	return a + r;
}

//Function that returns the distance between two points
float distance(const glm::vec3& A, const glm::vec3& B)
{
	return glm::sqrt(glm::pow((B.x - A.x), 2) + glm::pow((B.y - A.y), 2) + glm::pow((B.z - A.z),2));
}

//Function that returns the normalized vector between two points
glm::vec3 normVector(const glm::vec3& A, const glm::vec3& B)
{
	glm::vec3 v { (B.x - A.x), (B.y - A.y), (B.z - A.z) };
	v = glm::normalize(v);
	return v;
}

//Force Actuators
struct ForceActuator
{
	virtual glm::vec3 computeForce(float mass, const glm::vec3& position) = 0;
};

struct GravityForce : ForceActuator
{
	glm::vec3 force{ 0.0f, -9.81f, 0.0f };

	glm::vec3 computeForce(float mass, const glm::vec3& position)
	{
		if (useGravity)
			return force;
		return { 0,0,0 };
	}
};

struct PositionalGravityForce : ForceActuator
{
	glm::vec3 computeForce(float mass, const glm::vec3& position)
	{
		float sphereMass = Sphere::mass;
		glm::vec3 spherePos = Sphere::position;

		float d = distance(position, spherePos);

		float forceMagnitude = (mass * sphereMass) / (d * d);

		glm::vec3 v = normVector(position, spherePos);

		return v * forceMagnitude;
	}
};

glm::vec3 computeForces(float mass, const glm::vec3& position, const std::vector<ForceActuator*>& forceActs)
{
	glm::vec3 totalForce{ 0,0,0 };

	for (int i = 0; i < forceActs.size(); i++)
	{
		totalForce += forceActs[i]->computeForce(mass, position);
	}

	return totalForce;
}

//Collisions
struct Collider
{
	glm::vec3 normal; 
	float d;

	virtual bool checkCollision(const glm::vec3& prevPos, const glm::vec3& nextPos) = 0;
	virtual void getPlane(glm::vec3& normal, float& d) = 0;
	void computeCollision(const glm::vec3& oldPos, const glm::vec3& oldVel, glm::vec3& newPos, glm::vec3& newVel)
	{
		getPlane(normal, d);
		std::cout << normal.x << std::endl;
		std::cout << normal.y << std::endl;
		std::cout << normal.z << std::endl;
		std::cout << d << std::endl;
	}
};

struct PlaneCol : Collider
{
	bool checkCollision(const glm::vec3& prevPos, const glm::vec3& nextPos)
	{

	}

	void getPlane(glm::vec3& normal, float& d)
	{
		normal = { 0,2,4 };
		d = 200.45f;
	}
};

struct SphereCol : Collider
{
	bool checkCollision(const glm::vec3& prevPos, const glm::vec3& nextPos)
	{
		float prevD = distance(prevPos, Sphere::position);
		float nextD = distance(nextPos, Sphere::position);

		return (prevD > nextD && nextD <= Sphere::radius);
	}

	void getPlane(glm::vec3& normal, float& d)
	{
		
	}
};

struct CapsuleCol : Collider
{

};

//Particle System struct
struct ParticleSystem {
public:
	int numparticles;

	float* pos;
	float* vel;

	float mass;

	bool play;

	ParticleSystem(int num, float r, glm::vec3 initSquarePos, glm::vec3 finalSquarePos) : numparticles(num), radius(r)
	{
		srand(time(NULL));

		Particles::setupParticles(num, r);

		pos = new float[num * 3];
		vel = new float[num * 3];

		InitParticlesPosition(initSquarePos, finalSquarePos);
		InitParticlesVelocity();

		mass = 1.0f;

		play = false;
		useGravity = true;
	}

	void UpdateParticles()
	{
		Particles::updateParticles(0, numparticles, pos);
	}

	void CleanUpParticles()
	{
		Particles::cleanupParticles();
	}

	void ResetParticleSystem(glm::vec3 initSquarePos, glm::vec3 finalSquarePos)
	{
		Particles::setupParticles(numparticles, radius);

		pos = new float[numparticles * 3];
		vel = new float[numparticles * 3];

		InitParticlesPosition(initSquarePos, finalSquarePos);
		InitParticlesVelocity();
	}

private:
	float radius;

	void InitParticlesPosition(glm::vec3 initSquarePos, glm::vec3 finalSquarePos)
	{
		int init[3] = { initSquarePos.x, initSquarePos.y, initSquarePos.z};
		int final[3] = { finalSquarePos.x, finalSquarePos.y, finalSquarePos.z};

		for (int i = 0; i < numparticles; i++)
		{
			pos[i * 3] = RandomFloat(init[0], final[0]);
			pos[i * 3 + 1] = RandomFloat(init[1], final[1]);
			pos[i * 3 + 2] = RandomFloat(init[2], final[2]);
		}
	}

	void InitParticlesVelocity()
	{
		for (int i = 0; i < numparticles; i++)
			vel[i * 3] = vel[i * 3 + 1] = vel[i * 3 + 2] = 0;
	}
};

ParticleSystem* ps;

std::vector<ForceActuator*> forceActuators;

GravityForce* gravity = new GravityForce;
PositionalGravityForce* positionalGravityForce = new PositionalGravityForce;

std::vector<Collider*> colliders;
SphereCol* sphereCollider = new SphereCol;

bool renderSphere = false;
bool renderCapsule = false;
bool renderParticles = true;
bool renderMesh = false;
bool renderFiber = false;
bool renderCube = false;

//You may have to change this code
void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();

	if (renderSphere)
		Sphere::drawSphere();
	if (renderCapsule)
		Capsule::drawCapsule();

	if (renderParticles) {
		int startDrawingFromParticle = 0;
		int numParticlesToDraw = Particles::maxParticles;
		Particles::drawParticles(startDrawingFromParticle, numParticlesToDraw);
	}

	if (renderMesh)
		Mesh::drawMesh();
	if (renderFiber)
		Fiber::drawFiber();

	if (renderCube)
		Cube::drawCube();
}


void GUI() {
	bool show = true;
	ImGui::Begin("Simulation Parameters", &show, 0);
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::Checkbox("Play simulation", &ps->play);

		if (ImGui::Button("Reset Simulation"))
		{
			ps->ResetParticleSystem(boxCoordinatesInit, boxCoordinatesFinal);
		}
	}

	ImGui::End();

	ImGui::Begin("Physics Parameters", &show, 0);
	{	
		ImGui::DragFloat("Particles mass", &ps->mass, dragPrecision, 0.0f, FLT_MAX);

		ImGui::Text("Elasticity & Friction");
		ImGui::DragFloat("Elastic Coefficient", &elasticCoefficient, dragPrecision, 0.0f, 1.f);
		ImGui::DragFloat("Friction Coefficient", &frictionCoefficient, dragPrecision, 0.0f, 1.f);

		ImGui::Spacing();

		ImGui::Text("Colliders");
		ImGui::Checkbox("Show Sphere", &renderSphere);
		ImGui::Checkbox("Use Sphere Collider", &Sphere::collider);
		ImGui::DragFloat("Sphere Mass", &Sphere::mass, dragPrecision, 0.0f, FLT_MAX);
		ImGui::DragFloat3("Sphere Position", &Sphere::position.x, dragPrecision);
		ImGui::DragFloat("Sphere Radius", &Sphere::radius, dragPrecision, 0.0f, FLT_MAX);

		ImGui::Checkbox("Show Capsule", &renderCapsule);
		ImGui::Checkbox("Use Capsule Collider", &Capsule::collider);
		ImGui::DragFloat3("Capsule Position A", &Capsule::positionA.x, dragPrecision);
		ImGui::DragFloat3("Capsule Position B", &Capsule::positionB.x, dragPrecision);
		ImGui::DragFloat("Capsule Radius", &Capsule::radius, dragPrecision, 0.0f, FLT_MAX);

		ImGui::Spacing();

		ImGui::Text("Forces");
		ImGui::Checkbox("Use gravity", &useGravity);
		ImGui::DragFloat3("Gravity Acceleration", &gravity->force.x, dragPrecision);
	}
	
	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = false;
	if(show_test_window) 
	{
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void euler(float dt, ParticleSystem& particles, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& forceActs)
{
	if (particles.play)
	{
		for (int i = 0; i < particles.numparticles; i++)
		{
			glm::vec3 position{ particles.pos[i * 3], particles.pos[i * 3 + 1], particles.pos[i * 3 + 2] };
			glm::vec3 velocity{ particles.vel[i * 3], particles.vel[i * 3 + 1], particles.vel[i * 3 + 2] };

			glm::vec3 forces = computeForces(particles.mass, position, forceActs);

			particles.pos[i * 3] = position.x + velocity.x * dt + 0.5f * forces.x * dt * dt;
			particles.pos[i * 3 + 1] = position.y + velocity.y * dt + 0.5f * forces.y * dt * dt;
			particles.pos[i * 3 + 2] = position.z + velocity.z * dt + 0.5f * forces.z * dt * dt;	

			particles.vel[i * 3] = velocity.x + forces.x * dt;
			particles.vel[i * 3 + 1] = velocity.y + forces.y * dt;
			particles.vel[i * 3 + 2] = velocity.z + forces.z * dt;

			for (int j = 0; j < colliders.size(); j++)
				if (colliders[j]->checkCollision(position, { particles.pos[i * 3], particles.pos[i * 3 + 1] , particles.pos[i * 3 + 2] }))
				{
					glm::vec3 newPos; 
					glm::vec3 newVel;
					colliders[j]->computeCollision(position, velocity, newPos, newVel);
				}
		}
	}
}

void PhysicsInit() 
{
	ps = new ParticleSystem(1, 0.05f, boxCoordinatesInit, boxCoordinatesFinal);

	forceActuators.push_back(gravity);
	forceActuators.push_back(positionalGravityForce);

	colliders.push_back(sphereCollider);
}

void PhysicsUpdate(float dt) 
{
	Sphere::updateSphere(Sphere::position, Sphere::radius);
	Capsule::updateCapsule(Capsule::positionA, Capsule::positionB, Capsule::radius);

	euler(dt, *ps, colliders, forceActuators);
	ps->UpdateParticles();
}

void PhysicsCleanup() {
	ps->CleanUpParticles();
}
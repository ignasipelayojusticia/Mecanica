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
glm::vec2 k_stretch{ 1000, 5 };
glm::vec2 k_bend{ 1000, 5 };
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
	int numPart;
	float* pos;
	float* vel;
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
	int P1, P2;
	float L0;
	glm::vec2 k;

	Spring(int p1, int p2, float l0, glm::vec2 _k) : P1(p1), P2(p2), L0(l0), k(_k) {}
};

glm::vec3 springforce(const glm::vec3& P1, const glm::vec3& V1, const glm::vec3& P2, const glm::vec3& V2, float L0, float ke, float kd){	return -(ke*(vectorModule(P1 - P2) - L0) + kd * (V1 - V2)*((P1 - P2) / (vectorModule(P1 - P2))))*((P1 - P2) / (vectorModule(P1 - P2)));}

struct FiberStraw
{
public: 
	int firstParticle;

	glm::vec3 initPos;

	std::vector<Spring> springs;

	FiberStraw(int firstPart, int partPerStraw, float strawHeight) : firstParticle(firstPart)
	{
		InitStraw(strawHeight);
	}

	void InitStraw(float strawHeight)
	{
		initPos = { RandomFloat(strawCoordinatesInit.x, strawCoordinatesFinal.x), RandomFloat(strawCoordinatesInit.y, strawCoordinatesFinal.y), RandomFloat(strawCoordinatesInit.z, strawCoordinatesFinal.z) };

		Particles::pos[firstParticle * 3] = initPos.x;
		Particles::pos[firstParticle * 3 + 1] = initPos.y;
		Particles::pos[firstParticle * 3 + 2] = initPos.z;

		int y = 1; 
		bool si = false;
		for (int i = firstParticle + 1; i < firstParticle + Fiber::numVerts; i++)
		{
			Particles::pos[i * 3] = initPos.x;
			Particles::pos[i * 3 + 1] = initPos.y + y * (strawHeight / Fiber::numVerts);
			Particles::pos[i * 3 + 2] = initPos.z;

			springs.push_back({ ((i - 1) * 3), i * 3, (strawHeight / Fiber::numVerts), k_stretch });
			if (si)
				springs.push_back({ ((i - 2) * 3), i * 3, (strawHeight / Fiber::numVerts) * 2, k_bend });
			else 
				si = true;
			y++;
		}	

		for (int i = 0; i < springs.size(); i++)
		{
			glm::vec3 p1 = { Particles::pos[springs[i].P1], Particles::pos[springs[i].P1 + 1], Particles::pos[springs[i].P1 + 2] };
			glm::vec3 p2 = { Particles::pos[springs[i].P2], Particles::pos[springs[i].P2 + 1], Particles::pos[springs[i].P2 + 2] };
			glm::vec3 v1 = { Particles::vel[springs[i].P1], Particles::vel[springs[i].P1 + 1], Particles::vel[springs[i].P1 + 2] };
			glm::vec3 v2 = { Particles::vel[springs[i].P2], Particles::vel[springs[i].P2 + 1], Particles::vel[springs[i].P2 + 2] };
			glm::vec3 s = springforce(p1, v1, p2, v2, springs[i].L0, springs[i].k.x, springs[i].k.y);

			std::cout << s.x << "  " << s.y << "  " << s.z << std::endl;
		}
	}

	void ResetStraw(float strawHeight)
	{
		InitStraw(strawHeight);
	}

	void DrawFiberStraw()
	{
		float* p = Particles::pos;
		p += firstParticle * 3;
		Fiber::updateFiber(p);
		Fiber::drawFiber();
	}
};

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
	return glm::vec3{0,0,0};
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

	Particles::numPart = numberOfStraws * Fiber::numVerts;
	Particles::setupParticles(Particles::numPart, 0.05f);
	Particles::pos = new float[Particles::numPart * 3];
	Particles::vel = new float[Particles::numPart * 3];
	for (int i = 0; i < Particles::numPart * 3; i++)
		Particles::vel[i] = 0;

	for (int i = 0; i < numberOfStraws; i++)
		fiberStraws.push_back({ i * Fiber::numVerts, Fiber::numVerts, strawHeight });

	forceActuators.push_back(gravity);
	forceActuators.push_back(wind);
}

void PhysicsUpdate(float dt) 
{
	Particles::updateParticles(0, Particles::numPart, Particles::pos);

	Sphere::UniformCircularMotion(dt);
	Sphere::updateSphere(Sphere::position, Sphere::radius);
}

void PhysicsCleanup() {
	Sphere::cleanupSphere();
}
#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>

#include <vector>
#include <time.h>
#include <iostream>

//Physics 
double G = 0.6674f;
float elasticCoefficient = 1.0f;
float frictionCoefficient = 0.0f;
bool useGravity;

//Box coordinates
glm::vec3 boxCoordinatesInit{ -5, 0, -5 };
glm::vec3 boxCoordinatesFinal = { 5, 10, 5 };

//Particles spawn down position
glm::vec3 particlesSpawnInit = { -5, 5, -5 };

//Particles init speed range values
glm::vec2 xSpeed = {-0.3, 0.3};
glm::vec2 ySpeed = { -0.3, 0.3 };
glm::vec2 zSpeed = { -0.3, 0.3 };

//GUI 
float dragPrecision = 0.01f;


//NAMESPACES
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

//ADDITIONAL FUNCTIONS
//Function that generates a random float between two numbers
float RandomFloat(float a, float b)
{
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = b - a;
	float r = random * diff;
	return a + r;
}

float vectorModule(const glm::vec3& A)
{
	return (glm::sqrt((glm::pow(A.x, 2)) + (glm::pow(A.y, 2)) + (glm::pow(A.z, 2))));
}

//Function that returns the distance between two points
float distance(const glm::vec3& A, const glm::vec3& B)
{
	return glm::sqrt(glm::pow((B.x - A.x), 2) + glm::pow((B.y - A.y), 2) + glm::pow((B.z - A.z),2));
}

//Function that returns the distance between a plane and a point
float distancePlanePoint(const glm::vec3& n, const float& d, const glm::vec3& p)
{
	float up = n.x * p.x + n.y * p.y + n.z * p.z + d;
	float down = glm::sqrt(pow(n.x,2) + pow(n.y,2) + pow(n.z,2));
	return up / down;
}

float distanceRectPoint(const glm::vec3& q, const glm::vec3& v, const glm::vec3& p)
{
	return vectorModule((q + (glm::dot((q - p), v))*v) - p);
}

//Function that returns the normalized vector between two points
glm::vec3 normalizeVector(const glm::vec3& A, const glm::vec3& B)
{
	glm::vec3 v { (B.x - A.x), (B.y - A.y), (B.z - A.z) };
	v = glm::normalize(v);
	return v;
}

//Function that returns the dotProduct between two vectors
float multVector(const glm::vec3& A, const glm::vec3& B)
{
	return (B.x * A.x) + (B.y * A.y) + (B.z * A.z);
}

//Function that returns the vector between two points
glm::vec3 makeVector(const glm::vec3& A, const glm::vec3& B)
{
	return {(B.x - A.x), (B.y - A.y), (B.z - A.z)};
}

//Function that returns the crossProduct between two vectors
glm::vec3 crossPorduct(const glm::vec3& A, const glm::vec3& B)
{
	return {((B.z * A.y) - (B.y * A.z)), ((A.z * B.x) - (A.x * B.z)), ((A.x * B.y) - (B.x * A.y))};
}



glm::vec3 closestPoint(const glm::vec3& p, const glm::vec3& linePoint, const glm::vec3& lineVector)
{
	glm::vec3 vectorLPP = makeVector(linePoint, p);

	glm::vec3 proj = (multVector(lineVector, vectorLPP) / glm::pow(vectorModule(lineVector), 2)) * lineVector;

	return linePoint + proj;
}


glm::vec3 binaryS(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& q, const glm::vec3& v)
{
	glm::vec3 middle = p1 + (makeVector(p1, p2) / 2.0f);
	if (distanceRectPoint(middle, v, q) < Capsule::radius + 0.000000005 || distanceRectPoint(middle, v, q) > Capsule::radius - 0.000000005) {
		return middle;
	}
	bool mayor = distanceRectPoint(middle, v, q) > Capsule::radius;
	glm::vec3 p3;
	if (mayor) p3 = (distanceRectPoint(p1, v, q) > Capsule::radius) ? p2 : p1;
	else p3 = (distanceRectPoint(p1, v, q) < Capsule::radius) ? p2 : p1;
	
	binaryS(middle, p3, q, v);
}



//PARTICLE SYSTEM
struct ParticleSystem {
public:
	int numparticles;

	float* pos;
	float* vel;

	float mass;
	bool play;

	ParticleSystem(int num, float r, glm::vec3 initSquarePos, glm::vec3 finalSquarePos, glm::vec2 xS, glm::vec2 yS, glm::vec2 zS) : numparticles(num), radius(r)
	{
		Particles::setupParticles(num, r);

		pos = new float[num * 3];
		vel = new float[num * 3];

		InitParticlesPosition(initSquarePos, finalSquarePos);
		InitParticlesVelocity(xS, yS, zS);

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

	void ResetParticleSystem(glm::vec3 initSquarePos, glm::vec3 finalSquarePos, glm::vec2 xS, glm::vec2 yS, glm::vec2 zS)
	{
		Particles::setupParticles(numparticles, radius);

		pos = new float[numparticles * 3];
		vel = new float[numparticles * 3];

		InitParticlesPosition(initSquarePos, finalSquarePos);
		InitParticlesVelocity(xS, yS, zS);
	}

private:
	float radius;

	void InitParticlesPosition(glm::vec3 initSquarePos, glm::vec3 finalSquarePos)
	{
		int init[3] = { initSquarePos.x, initSquarePos.y, initSquarePos.z };
		int final[3] = { finalSquarePos.x, finalSquarePos.y, finalSquarePos.z };

		for (int i = 0; i < numparticles; i++)
		{
			pos[i * 3] = RandomFloat(init[0], final[0]);
			pos[i * 3 + 1] = RandomFloat(init[1], final[1]);
			pos[i * 3 + 2] = RandomFloat(init[2], final[2]);
		}
	}

	void InitParticlesVelocity(glm::vec2 xS, glm::vec2 yS, glm::vec2 zS)
	{
		for (int i = 0; i < numparticles; i++)
			vel[i * 3] = vel[i * 3 + 1] = vel[i * 3 + 2] = 0;

		for (int i = 0; i < numparticles; i++)
		{
			vel[i * 3] = RandomFloat(xS.x, xS.y);
			vel[i * 3 + 1] = RandomFloat(yS.x, yS.y);
			vel[i * 3 + 2] = RandomFloat(zS.x, zS.y);
		}
	}
};

//ParticleSystem
ParticleSystem* ps;





//FORCE ACTUATORS
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

		float forceMagnitude = G * (mass * sphereMass) / (d * d);

		glm::vec3 v = normalizeVector(position, spherePos);

		return v * forceMagnitude;
	}
};

glm::vec3 computeForces(float mass, const glm::vec3& position, const std::vector<ForceActuator*>& forceActs)
{
	glm::vec3 totalForce{ 0,0,0 };

	for (int i = 0; i < forceActs.size(); i++)
		totalForce += forceActs[i]->computeForce(mass, position);

	return totalForce;
}



//Forces
std::vector<ForceActuator*> forceActuators;
GravityForce* gravity = new GravityForce;
PositionalGravityForce* positionalGravityForce = new PositionalGravityForce;





//COLLIDERS
struct Collider
{
	glm::vec3 normal; 
	float d;

	glm::vec3 previousPosition;
	glm::vec3 newPosition;

	virtual bool checkCollision(const glm::vec3& prevPos, const glm::vec3& nextPos) = 0;
	virtual void getPlane(glm::vec3& normal, float& d) = 0;
	void computeCollision(const glm::vec3& oldPos, const glm::vec3& oldVel, glm::vec3& newPos, glm::vec3& newVel)
	{
		previousPosition = oldPos;
		newPosition = newPos;

		getPlane(normal, d);

		glm::vec3 finalPos = newPos - (1 + elasticCoefficient) * (multVector(normal, newPos) + d) * normal;
		newPos = finalPos;

		glm::vec3 dir = glm::normalize((multVector(normal, oldVel) / glm::pow(vectorModule(normal), 2)) * normal);

		glm::vec3 proj = (multVector(-normal, (gravity->force * ps->mass)) / glm::pow(vectorModule(-normal), 2)) * -normal;

		float Ff = vectorModule(proj)*frictionCoefficient;

		glm::vec3 Fr = dir * Ff;

		glm::vec3 finalVel = newVel - (1 + elasticCoefficient) * (multVector(normal, newVel)) * normal;
		finalVel += Fr;
		//Tenemos que calcular la fuerza normal contra la superfície que colisiona (si está en plano la fuerza normal == masa * g)

		newVel = finalVel;
	}
};

struct PlaneCol : Collider
{
	glm::vec3 p1, p2, p3, p4;

	PlaneCol(glm::vec3 _p1, glm::vec3 _p2, glm::vec3 _p3, glm::vec3 _p4) : p1(_p1), p2(_p2), p3(_p3), p4(_p4)
	{
		glm::vec3 v1 = makeVector(p1, p4);
		glm::vec3 v2 = makeVector(p2, p3);

		normal = glm::normalize(crossPorduct(v1, v2));
		d = -(normal.x * p1.x + normal.y * p1.y + normal.z * p1.z);
	}

	bool checkCollision(const glm::vec3& prevPos, const glm::vec3& nextPos)
	{
		return distancePlanePoint(normal, d, nextPos) < 0;
	}

	void getPlane(glm::vec3& normal, float& d) {}
};

struct SphereCol : Collider
{
	bool checkCollision(const glm::vec3& prevPos, const glm::vec3& nextPos)
	{
		if (!Sphere::collider)
			return false;

		float prevD = distance(prevPos, Sphere::position);
		float nextD = distance(nextPos, Sphere::position);

		return (prevD > nextD && nextD <= Sphere::radius);
	}

	void getPlane(glm::vec3& normal, float& d)
	{
		glm::vec3 vec = newPosition - previousPosition;

		float a = ((pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2)));
		float b = -(2 * (Sphere::position.x*vec.x + Sphere::position.y*vec.y + Sphere::position.z*vec.z - previousPosition.x*vec.x - previousPosition.y*vec.y - previousPosition.z*vec.z));		
		float c = ((pow(Sphere::position.x, 2) + pow(Sphere::position.y, 2) + pow(Sphere::position.z, 2) - 2 * (Sphere::position.x * previousPosition.x + Sphere::position.y * previousPosition.y 
			+ Sphere::position.z * previousPosition.z) + pow(previousPosition.x,2) + pow(previousPosition.y, 2) + pow(previousPosition.z, 2) - glm::pow(Sphere::radius,2)));

		float lambda1 = (-b + glm::sqrt(glm::pow(b, 2) - (4 * a * c))) / (2 * a);
		float lambda2 = (-b - glm::sqrt(glm::pow(b, 2) - (4 * a * c))) / (2 * a);

		glm::vec3 colPoint1 = { previousPosition.x + lambda1 * vec.x, previousPosition.y + lambda1 * vec.y, previousPosition.z + lambda1 * vec.z, };
		glm::vec3 colPoint2 = { previousPosition.x + lambda2 * vec.x, previousPosition.y + lambda2 * vec.y, previousPosition.z + lambda2 * vec.z, };

		glm::vec3 colPoint = (distance(previousPosition, colPoint1) < distance(previousPosition, colPoint2)) ? colPoint1 : colPoint2;

		normal = normalizeVector(Sphere::position, colPoint);

		d = -(normal.x * colPoint.x + normal.y * colPoint.y + normal.z * colPoint.z);
	}
};

struct CapsuleCol : Collider
{
	bool cilindre = false;
	glm::vec3 pAux;
	bool checkCollision(const glm::vec3& prevPos, const glm::vec3& nextPos)
	{
		if (!Capsule::collider)
			return false;
		cilindre = false;
		glm::vec3 p = closestPoint(nextPos, Capsule::positionA, glm::normalize(makeVector(Capsule::positionB, Capsule::positionA)));

		p.x = glm::clamp(p.x, glm::min(Capsule::positionA.x, Capsule::positionB.x), glm::max(Capsule::positionA.x, Capsule::positionB.x));
		p.y = glm::clamp(p.y, glm::min(Capsule::positionA.y, Capsule::positionB.y), glm::max(Capsule::positionA.y, Capsule::positionB.y));
		p.z = glm::clamp(p.z, glm::min(Capsule::positionA.z, Capsule::positionB.z), glm::max(Capsule::positionA.z, Capsule::positionB.z));

		if (distance(nextPos, p) < Capsule::radius)
		{
			if (p != Capsule::positionA && p != Capsule::positionB) cilindre = true;
			else pAux = p;
			return true;
		}

		return false;
	}

	void getPlane(glm::vec3& normal, float& d)
	{
		glm::vec3 p1 = previousPosition;
		glm::vec3 p2 = newPosition;
		glm::vec3 colPoint1, colPoint2, colPoint;

		if (cilindre)
		{
			//Prueba con distancia para punto exacto, error
			/*std::cout << "Parte cilindro" << std::endl;
			glm::vec3 q = Capsule::positionA;
			glm::vec3 v = (makeVector(Capsule::positionA, Capsule::positionB));
			std::cout << "Vector A-B: (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;

			float X1 = (q.x + glm::dot(q, v)*v.x - (glm::dot(p1, v)*v.x) - p1.x);
			float X2 = ((glm::dot(vr, v)*v.x) - vr.x);
			std::cout << "X1: " << X1 << " X2: " << X2 << std::endl;

			float Y1 = (q.y + glm::dot(q, v)*v.y - (glm::dot(p1, v)*v.y) - p1.y);
			float Y2 = ((glm::dot(vr, v)*v.y) - vr.y);
			std::cout << "Y1: " << Y1 << " Y2: " << Y2 << std::endl;

			float Z1 = (q.z + glm::dot(q, v)*v.z - (glm::dot(p1, v)*v.z) - p1.z);
			float Z2 = ((glm::dot(vr, v)*v.z) - vr.z);
			std::cout << "Z1: " << Z1 << " Z2: " << Z2 << std::endl;

			a = (pow(X2, 2) + pow(Y2, 2) + pow(Z2, 2));
			b = -2 * (X1*X2 + Y1 * Y2 + Z1 * Z2);
			c = (pow(X1, 2) + pow(Y1, 2) + pow(Z1, 2) - pow(Capsule::radius, 2));*/

			//Con binary
			glm::vec3 v = (makeVector(Capsule::positionA, Capsule::positionB));
			colPoint = binaryS(p1,p2,Capsule::positionA,v);
			pAux = closestPoint(colPoint, Capsule::positionA, glm::normalize(makeVector(Capsule::positionB, Capsule::positionA)));
		}
		else
		{
			float a, b, c;
			float lambda1, lambda2;
			glm::vec3 vr = (makeVector(p1, p2));

			a = ((pow(vr.x, 2) + pow(vr.y, 2) + pow(vr.z, 2)));
			b = -(2 * (pAux.x*vr.x + pAux.y*vr.y + pAux.z*vr.z - p1.x*vr.x - p1.y*vr.y - p1.z*vr.z));
			c = ((pow(pAux.x, 2) + pow(pAux.y, 2) + pow(pAux.z, 2) - 2 * (pAux.x * p1.x + pAux.y * p1.y
				+ pAux.z * p1.z) + pow(p1.x, 2) + pow(p1.y, 2) + pow(p1.z, 2) - glm::pow(Capsule::radius, 2)));

			lambda1 = (-b + glm::sqrt(glm::pow(b, 2) - (4 * a * c))) / (2 * a);
			lambda2 = (-b - glm::sqrt(glm::pow(b, 2) - (4 * a * c))) / (2 * a);

			colPoint1 = { p1.x + lambda1 * vr.x, p1.y + lambda1 * vr.y, p1.z + lambda1 * vr.z, };
			colPoint2 = { p1.x + lambda2 * vr.x, p1.y + lambda2 * vr.y, p1.z + lambda2 * vr.z, };

			colPoint = (distance(p1, colPoint1) < distance(p1, colPoint2)) ? colPoint1 : colPoint2;
		}
		normal = normalizeVector(pAux, colPoint);
		d = -(normal.x * colPoint.x + normal.y * colPoint.y + normal.z * colPoint.z);
	}
};



//Colliders
std::vector<Collider*> colliders;
SphereCol* sphereCollider = new SphereCol;
CapsuleCol* capsuleCollider = new CapsuleCol;
PlaneCol* downPlaneCollider = new PlaneCol({ -5, 0,5 }, { 5,0,5 }, { -5,0,-5 }, { 5,0,-5 });
PlaneCol* rightPlaneCollider = new PlaneCol({ 5,0,-5 }, { 5,0,5 }, { 5,10,-5 }, { 5,10,5 });
PlaneCol* leftPlaneCollider = new PlaneCol({ -5,10,5 }, { -5,0,5 }, { -5,10,-5 }, { -5,0,-5 });
PlaneCol* upPlaneCollider = new PlaneCol({ 5,10,5 }, { -5,10,5 }, { 5,10,-5 }, { -5,10,-5 });
PlaneCol* frontPlaneCollider = new PlaneCol({ -5,0,-5 }, { 5,0,-5 }, { -5,10,-5 }, { 5,10,-5 });
PlaneCol* backPlaneCollider = new PlaneCol({ -5,10,5 }, { 5,10,5 }, { -5,0,5 }, { 5,0,5 });



//RENDER SETTINGS
bool renderSphere = false;
bool renderCapsule = false;
bool renderParticles = true;
bool renderMesh = false;
bool renderFiber = false;
bool renderCube = false;

void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();

	if (renderSphere)
		Sphere::drawSphere();
	if (renderCapsule)
		Capsule::drawCapsule();

	if (renderParticles) {
		int startDrawingFromParticle = 0;
		Particles::drawParticles(startDrawingFromParticle, ps->numparticles);
	}

	if (renderMesh)
		Mesh::drawMesh();

	if (renderFiber)
		Fiber::drawFiber();

	if (renderCube)
		Cube::drawCube();
}

//ImGUI 
void GUI() {
	bool show = true;

	ImGui::Begin("Simulation Parameters", &show, 0);
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::Checkbox("Play simulation", &ps->play);

		if (ImGui::Button("Reset Simulation"))
		{
			ps->ResetParticleSystem(particlesSpawnInit, boxCoordinatesFinal, xSpeed, ySpeed, zSpeed);
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
					glm::vec3 newPos = { particles.pos[i * 3], particles.pos[i * 3 + 1] , particles.pos[i * 3 + 2] };
					glm::vec3 newVel = { particles.vel[i * 3], particles.vel[i * 3 + 1] , particles.vel[i * 3 + 2] };

					colliders[j]->computeCollision(position, velocity, newPos, newVel);

					particles.pos[i * 3] = newPos.x;
					particles.pos[i * 3 + 1] = newPos.y;
					particles.pos[i * 3 + 2] = newPos.z;

					particles.vel[i * 3] = newVel.x;
					particles.vel[i * 3 + 1] = newVel.y;
					particles.vel[i * 3 + 2] = newVel.z;
				}
		}
	}
}

void PhysicsInit() 
{
	srand(time(NULL));

	ps = new ParticleSystem(100, 0.05f, particlesSpawnInit, boxCoordinatesFinal, xSpeed, ySpeed, zSpeed);

	forceActuators.push_back(gravity);
	forceActuators.push_back(positionalGravityForce);

	colliders.push_back(sphereCollider);
	colliders.push_back(capsuleCollider);
	colliders.push_back(downPlaneCollider);
	colliders.push_back(rightPlaneCollider);
	colliders.push_back(leftPlaneCollider);
	colliders.push_back(upPlaneCollider);
	colliders.push_back(frontPlaneCollider);
	colliders.push_back(backPlaneCollider);
}

void PhysicsUpdate(float dt) 
{
	Sphere::updateSphere(Sphere::position, Sphere::radius);
	Capsule::updateCapsule(Capsule::positionA, Capsule::positionB, Capsule::radius);

	euler(dt, *ps, colliders, forceActuators);
	ps->UpdateParticles();
}

void PhysicsCleanup() 
{
	ps->CleanUpParticles();
}
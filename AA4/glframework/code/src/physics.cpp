#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>

#include <vector>
#include <iostream>
#include <string>
#include <sstream>

float dragPrecision = 0.01f;

bool pause;

glm::vec3 gravity{ 0,-5,0 };
float elasticCoefficient = 0.5f;

//Box coordinates
glm::vec3 boxCoordinatesInit{ -5, 0, -5 };
glm::vec3 boxCoordinatesFinal = { 5, 10, 5 };

int numberOfSpheres = 1;

namespace Box {
	void drawCube();
}
namespace Axis {
	void drawAxis();
}

namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
}
namespace Capsule {
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius = 1.f);
	extern void drawCapsule();
}
namespace Particles {
	extern const int maxParticles;
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

//Function that returns the distance between a plane and a point
float distancePlanePoint(const glm::vec3& n, const float& d, const glm::vec3& p)
{
	float up = n.x * p.x + n.y * p.y + n.z * p.z + d;
	float down = glm::sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2));
	return up / down;
}

//Function that returns the module of a given vector
float vectorModule(const glm::vec3& A)
{
	return (glm::sqrt((glm::pow(A.x, 2)) + (glm::pow(A.y, 2)) + (glm::pow(A.z, 2))));
}

//Function that returns the vector between two points
glm::vec3 makeVector(const glm::vec3& A, const glm::vec3& B)
{
	return { (B.x - A.x), (B.y - A.y), (B.z - A.z) };
}

//Function that returns the crossProduct between two vectors
glm::vec3 crossProduct(const glm::vec3& A, const glm::vec3& B)
{
	return { ((B.z * A.y) - (B.y * A.z)), ((A.z * B.x) - (A.x * B.z)), ((A.x * B.y) - (B.x * A.y)) };
}

//Function that returns the normalized vector between two points
glm::vec3 normalizeVector(const glm::vec3& A, const glm::vec3& B)
{
	glm::vec3 v{ (B.x - A.x), (B.y - A.y), (B.z - A.z) };
	v = glm::normalize(v);
	return v;
}

//Function that returns the dotProduct between two vectors
float dotProd(const glm::vec3& A, const glm::vec3& B)
{
	return (B.x * A.x) + (B.y * A.y) + (B.z * A.z);
}

void printVec3OnConsole(const glm::vec3& vec)
{
	std::cout << vec.x << " " << vec.y << " " << vec.z << std::endl;
}

struct Collider 
{
	virtual bool checkCollision(const glm::vec3& next_pos, float radius) = 0;
};

struct PlaneCol : Collider 
{
	glm::vec3 p1, p2, p3, p4;
	glm::vec3 normal;
	float d;

	PlaneCol(glm::vec3 _p1, glm::vec3 _p2, glm::vec3 _p3, glm::vec3 _p4) : p1(_p1), p2(_p2), p3(_p3), p4(_p4)
	{
		glm::vec3 v1 = makeVector(p1, p4);
		glm::vec3 v2 = makeVector(p2, p3);

		normal = glm::normalize(crossProduct(v1, v2));
		d = -(normal.x * p1.x + normal.y * p1.y + normal.z * p1.z);
	}

	bool checkCollision(const glm::vec3& next_pos, float radius) override
	{
		return distancePlanePoint(normal, d, next_pos) <= radius;
	}
};

std::vector<PlaneCol*> planeColliders;

PlaneCol* downPlaneCollider = new PlaneCol({ -5, 0,5 }, { 5,0,5 }, { -5,0,-5 }, { 5,0,-5 });
PlaneCol* rightPlaneCollider = new PlaneCol({ 5,0,-5 }, { 5,0,5 }, { 5,10,-5 }, { 5,10,5 });
PlaneCol* leftPlaneCollider = new PlaneCol({ -5,10,5 }, { -5,0,5 }, { -5,10,-5 }, { -5,0,-5 });
PlaneCol* upPlaneCollider = new PlaneCol({ 5,10,5 }, { -5,10,5 }, { 5,10,-5 }, { -5,10,-5 });
PlaneCol* frontPlaneCollider = new PlaneCol({ -5,0,-5 }, { 5,0,-5 }, { -5,10,-5 }, { 5,10,-5 });
PlaneCol* backPlaneCollider = new PlaneCol({ -5,10,5 }, { 5,10,5 }, { -5,0,5 }, { 5,0,5 });


struct RigidSphere :Collider
{
	glm::vec3 x, v, P, L, w;
	glm::mat3 I, Ibody, R;
	float radius;
	float mass;
	glm::vec3 F;

	RigidSphere()
	{
		initSphere();
	}

	void initSphere()
	{
		radius = RandomFloat(0.1f, 1.5f);
		mass = RandomFloat(0.5f, 3.0f);

		float init[3] = {boxCoordinatesInit.x + radius, boxCoordinatesInit.y + radius, boxCoordinatesInit.z + radius};
		float final[3] = { boxCoordinatesFinal.x - radius, boxCoordinatesFinal.y - radius, boxCoordinatesFinal.z - radius };
		x = { RandomFloat(init[0], final[0]), RandomFloat(init[1], final[1]), RandomFloat(init[2], final[2]) };


		v = glm::vec3(RandomFloat(-2.f, 2.f), RandomFloat(-2.f, 2.f), RandomFloat(-2.f, 2.f));
		P = mass * v;
		w = glm::vec3(0, 0, 0);
		R = glm::mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);
		Ibody = glm::mat3(2 / 5 * mass * radius * radius, 0, 0, 0, 2 / 5 * mass * radius * radius, 0, 0, 0, 2 / 5 * mass * radius * radius);
		I = R * Ibody * R;
		L = I * w;
		F = glm::vec3(0);
	}

	void drawSphere()
	{
		Sphere::updateSphere(x, radius);
		Sphere::drawSphere();
	}

	bool checkCollision(const glm::vec3& next_pos, float r) 
	{
		return vectorModule(next_pos - x) <= r + radius;
	}
};

std::vector<RigidSphere> spheres;

bool renderSphere = true;
bool renderCapsule = false;
bool renderParticles = false;
bool renderMesh = false;
bool renderFiber = false;
bool renderCube = false;

float computeImpulseCorrection(float massA, glm::vec3 ra, glm::mat3 invIa, float massB, glm::vec3 rb, glm::mat3 invIb, float vrel, float epsilon, glm::vec3 normal)
{
	float up = -(1 + elasticCoefficient) * vrel;
	float down = (1 / massA) + (1 / massB) + dotProd(normal, crossProduct(invIa * (crossProduct(ra, normal)), ra)) + dotProd(normal, crossProduct(invIb * crossProduct(rb, normal), rb));
	return up / down;
}

void updateColliders(Collider* A, Collider* B)
{

}

void euler(float dt, RigidSphere& sph) 
{
	glm::vec3 totalForces = sph.F + gravity;
	glm::vec3 tor = glm::vec3(0, 0, 0); //Torque de la gravedad

	glm::vec3 prevX = sph.x;

	sph.P += dt * totalForces;	//Sin colisión (solo afecta gravedad)
	sph.v = sph.P / sph.mass;
	sph.x = sph.x + sph.v * dt;

	sph.L += dt * tor; //no se modifica el momento angular porque tor = 0 ya que la fuerza de la gravedad se aplica en el centro de massas del objeto
	glm::mat3 Ii = sph.R * glm::inverse(sph.Ibody) * glm::transpose(sph.R);
	sph.w = Ii * sph.L;
	sph.R = sph.R + dt * (glm::mat3(0, -sph.w.z, sph.w.y, sph.w.z, 0, -sph.w.x, -sph.w.y, sph.w.x, 0)* sph.R);

	//collisiones
	for (auto plane : planeColliders)
	{
		if (plane->checkCollision(sph.x, sph.radius))
		{
			glm::vec3 J = glm::vec3(0);
			glm::vec3 F0 = sph.F;
			glm::vec3 iTor;

			float newDt = dt;
			while (distancePlanePoint(plane->normal, plane->d, prevX) >= sph.radius + 0.001f)
			{
				prevX += sph.v * (dt / 1000.0f);
				newDt -= dt / 1000.0f;
			}
			glm::vec3 collisionPoint = sph.x + (- plane->normal * sph.radius);		
			
			glm::vec3 collisionForce = plane->normal * sph.mass;
			//tor += crossProduct((collisionPoint - sph.x), collisionForce);
			sph.F += collisionForce;
			J = sph.F - F0 + gravity;
			iTor = crossProduct((collisionPoint - sph.x), J);
			sph.w = glm::transpose(sph.I) * iTor;
			sph.L = sph.I * sph.w;
			sph.P = J - sph.P;
			sph.v = sph.P / sph.mass;
		}
	}

	printVec3OnConsole(sph.P);
	for (int i = 0; i < spheres.size(); i++) 
	{
		if (spheres[i].x != sph.x)
		{
			if (sph.checkCollision(spheres[i].x, spheres[i].radius)) 
			{
				std::cout << "Colisiona" << std::endl;
				//todo: calcular nueva fuerza y añadirla a totalForces
				//tor += crossPorduct((/*punto en el que se produce la colision*/ -sph.x), /*fuerza de esta colision*/);
			}
		}
	}
}

void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();


	if (renderSphere)
		for (auto sphere : spheres)
			sphere.drawSphere();

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

void Reset()
{
	for (auto& sphere : spheres)
		sphere.initSphere();
}

void ResetCountDown(float dt)
{
	static float accumTime = 0.0f;
	accumTime += dt;

	if (accumTime >= 15.0f)
	{
		accumTime = 0.0f;
		Reset();
	}
}

void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::Checkbox("Play simulation", &pause);
		if (ImGui::Button("Reset"))
			Reset();

		ImGui::DragFloat3("Gravity Acceleration", &gravity.x, dragPrecision);
		ImGui::DragFloat("Elastic Coefficient", &elasticCoefficient, dragPrecision, 0.0f, 1.0f);

		if (ImGui::TreeNode("Spheres"))
		{
			for (int i = 0; i < spheres.size(); i++)
			{
				std::stringstream ss;
				ss << i;
				std::string mass = "Mass_" + ss.str();
				std::string radius = "Radius_" + ss.str();

				ImGui::Spacing();
				ImGui::Spacing();
				ImGui::Spacing();

				ImGui::DragFloat(mass.c_str(), &spheres[i].mass, dragPrecision, 0.0f, FLT_MAX);
				ImGui::DragFloat(radius.c_str(), &spheres[i].radius, dragPrecision, 0.0f, FLT_MAX);
			}
			ImGui::TreePop();
		}
	}

	ImGui::End();
}

void PhysicsInit()
{
	planeColliders.push_back(downPlaneCollider);
	planeColliders.push_back(rightPlaneCollider);
	planeColliders.push_back(leftPlaneCollider);
	planeColliders.push_back(upPlaneCollider);
	planeColliders.push_back(frontPlaneCollider);
	planeColliders.push_back(backPlaneCollider);

	for(int i = 0; i < numberOfSpheres; i++)
		spheres.push_back(RigidSphere());
	Reset();
}

void PhysicsUpdate(float dt)
{
	if (pause)
		return;

	ResetCountDown(dt);

	for (auto& sphere : spheres)
		euler(dt, sphere);
}

void PhysicsCleanup()
{}
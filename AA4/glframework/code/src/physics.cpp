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

struct RigidSphere {
	glm::vec3 x, v, P, L;
	glm::mat3 w, I, Ibody, R;
	float radius;
	float mass;

	RigidSphere(const glm::vec3& initPos, const float& r, const float& m) : x(initPos), radius(r), mass(m)
	{}

	void drawSphere()
	{
		Sphere::updateSphere(x, radius);
		Sphere::drawSphere();
	}
};

std::vector<RigidSphere> spheres {
	{{1,1,1}, 2, 3},
	{{5,2,1}, 1, 3},
	{{-2,0.5,2}, 0.2, 3}
};

bool renderSphere = true;
bool renderCapsule = false;
bool renderParticles = false;
bool renderMesh = false;
bool renderFiber = false;
bool renderCube = false;

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
	std::cout << "Reset" << std::endl;
}

void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		
		ImGui::Checkbox("Play simulation", &pause);
		if(ImGui::Button("Reset"))
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

}

void PhysicsUpdate(float dt) 
{

}

void PhysicsCleanup() 
{

}
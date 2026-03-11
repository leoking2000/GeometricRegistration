#pragma once
#include <random>
#include <glm/glm.hpp>

namespace geo
{
	/// <summary>
	/// uses the std::mt19937 engine
	/// </summary>
	class Random final
	{
	public:
		Random(); // random_device seed
		explicit Random(unsigned int seed);
	public:
		// Sets the seed for reproducible random number generation
		void SetSeed(unsigned int seed);

		// Returns a random signed int in the range [min, max]
		int Int(int min, int max);

		// Returns a random unsigned int in the range [min, max]
		unsigned int UInt(unsigned int min, unsigned int max);

		// Returns a random float in the range [min, max)
		float Float(float min = 0.0f, float max = 1.0f);

		// Returns a random glm::vec2 where each component is in the range [min, max)
		glm::vec2 Float2(float min = 0.0f, float max = 1.0f);

		// Returns a random glm::vec3 where each component is in the range [min, max)
		glm::vec3 Float3(float min = 0.0f, float max = 1.0f);

		// Returns a random 2D direction vector with length provided
		glm::vec2 Dir2D(float length = 1.0f);

		// Returns a random 3D direction vector with length provided
		glm::vec3 Dir3D(float length = 1.0f);
	private:
		std::mt19937 m_Rng;
	};
}

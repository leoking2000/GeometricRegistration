#pragma once
#include <random>
#include <glm/glm.hpp>
#include "GeoTypes.h"

namespace geo
{
	/// <summary>
	/// uses the std::mt19937 engine
	/// </summary>
	class Random final
	{
	public:
		Random(); // random_device seed
		explicit Random(u32 seed);
	public:
		// Sets the seed for reproducible random number generation
		void SetSeed(u32 seed);

		// Returns a random signed int in the range [min, max]
		i32 Int(i32 min, i32 max);

		// Returns a random unsigned int in the range [min, max]
		u32 UInt(u32 min, u32 max);

		// Returns a random float in the range [min, max)
		f32 Float(f32 min = 0.0f, f32 max = 1.0f);

		// Returns a random glm::vec2 where each component is in the range [min, max)
		glm::vec2 Float2(f32 min = 0.0f, f32 max = 1.0f);

		// Returns a random glm::vec3 where each component is in the range [min, max)
		glm::vec3 Float3(f32 min = 0.0f, f32 max = 1.0f);

		// Returns a random 2D direction vector with length provided
		glm::vec2 Dir2D(f32 length = 1.0f);

		// Returns a random 3D direction vector with length provided
		glm::vec3 Dir3D(f32 length = 1.0f);
	private:
		std::mt19937 m_Rng;
	};
}

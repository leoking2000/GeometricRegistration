#pragma once
#include <string>
#include <filesystem>

namespace geo::io
{
	std::string ReadFile(const std::filesystem::path& filePath);

	// Checks whether a file exists at the given path.
	bool FileExists(const std::filesystem::path& filePath);

	// Extracts the filename portion from a full filesystem path.
	std::string GetFileName(const std::filesystem::path& filePath);

	// Returns the parent directory of a given file path.
	std::filesystem::path GetParentFolder(const std::filesystem::path& filePath);
}

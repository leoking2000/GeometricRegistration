#include <glad/glad.h>
#include <geo/utils/logging/LogMacros.h>
#include <geo/io/IOUtils.h>
#include "Shader.h"


namespace gl
{
	geo::u32 CompileShader(const char* source, geo::u32 type);
	geo::u32 CreateShaderProgramFromSource(const char* vertSrc, const char* geoSrc, const char* fragSrc);

	ShaderProgram::ShaderProgram(const std::string& filepath)
		:
		m_program_id(0)
	{
		std::string vert_source = geo::io::ReadFile(filepath + ".vert");
		std::string geom_source = geo::io::ReadFile(filepath + ".geom");
		std::string frag_source = geo::io::ReadFile(filepath + ".frag");

		if (vert_source == "")
		{
			GEOLOGFATAL("Vertex shader missing!");
			std::exit(-1);
		}

		if (frag_source == "")
		{
			GEOLOGFATAL("Fragment shader missing!");
			std::exit(-1);
		}

		Reload(vert_source.c_str(), ((geom_source == "") ? nullptr : geom_source.c_str()), frag_source.c_str());
	}

	ShaderProgram::ShaderProgram(const char* vertSrc, const char* fragSrc)
		:
		ShaderProgram(vertSrc, nullptr, fragSrc)
	{
	}

	ShaderProgram::ShaderProgram(const char* vertSrc, const char* geoSrc, const char* fragSrc)
		:
		m_program_id(0)
	{
		Reload(vertSrc, geoSrc, fragSrc);
	}

	ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept
		:
		m_program_id(other.m_program_id),
		m_uniforms(std::move(other.m_uniforms))
	{
		other.m_program_id = 0;
	}

	ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept
	{
		glDeleteProgram(m_program_id);

		m_program_id = other.m_program_id;
		m_uniforms = std::move(other.m_uniforms);

		other.m_program_id = 0;

		return *this;
	}

	ShaderProgram::~ShaderProgram()
	{
		glDeleteProgram(m_program_id);
	}

	bool ShaderProgram::Reload(const char* vertexSrc, const char* geoSrc, const char* fragSrc)
	{
		geo::u32 new_program = CreateShaderProgramFromSource(vertexSrc, geoSrc, fragSrc);

		if (new_program == 0) // if new program failed to Compiled
		{
			return false; // we do nothing and return false
		}

		// delete old 
		glDeleteProgram(m_program_id);
		m_uniforms.clear();

		// replace
		m_program_id = new_program;

		return true;
	}

	bool ShaderProgram::Reload(const char* vertexSrc, const char* fragSrc)
	{
		return Reload(vertexSrc, nullptr, fragSrc);
	}

	void ShaderProgram::Bind() const
	{
		glUseProgram(m_program_id);
	}

	void ShaderProgram::UnBind() const
	{
		glUseProgram(0);
	}

	bool ShaderProgram::SetUniform(const std::string& name, float num) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform1f(location, num);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, glm::vec2 a) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform2f(location, a.x, a.y);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, glm::vec3 a) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform3f(location, a.x, a.y, a.z);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, glm::vec4 a) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform4f(location, a.x, a.y, a.z, a.w);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, int i) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform1i(location, i);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, glm::ivec3 a) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform3i(location, a.x, a.y, a.z);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, glm::uvec3 a) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform3ui(location, a.x, a.y, a.z);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, unsigned int i) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniform1ui(location, i);
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, const glm::mat4& mat) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(mat));
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, const glm::mat3& mat) const
	{
		geo::i32 location = GetLocation(name);

		if (location != -1)
		{
			glUniformMatrix3fv(location, 1, GL_FALSE, glm::value_ptr(mat));
			return true;
		}

		return false;
	}

	bool ShaderProgram::SetUniform(const std::string& name, const std::vector<glm::vec2>& vec_arr, geo::u32 size)
	{
		for (geo::u32 i = 0; i < size; i++)
		{
			geo::i32 location = GetLocation(name + "[" + std::to_string(i) + "]");

			if (location == -1) {
				return false;
			}

			glUniform2f(location, vec_arr[i].x, vec_arr[i].y);
		}

		return true;
	}

	bool ShaderProgram::SetUniform(const std::string& name, const std::vector<glm::vec3>& vec_arr, geo::u32 size)
	{
		for (geo::u32 i = 0; i < size; i++)
		{
			geo::i32 location = GetLocation(name + "[" + std::to_string(i) + "]");

			if (location == -1) {
				return false;
			}

			glUniform3f(location, vec_arr[i].x, vec_arr[i].y, vec_arr[i].z);
		}

		return true;
	}

	geo::i32 ShaderProgram::GetLocation(const std::string& name) const
	{
		if (m_uniforms.find(name) != m_uniforms.end())
		{
			return m_uniforms[name];
		}

		geo::i32 loc;
		loc = glGetUniformLocation(m_program_id, name.c_str());

		if (loc != -1)
		{
			m_uniforms[name] = loc;
		}

		return loc;
	}

	bool ShaderProgram::IsValid() const
	{
		return m_program_id != 0;
	}


	geo::u32 CompileShader(const char* source, geo::u32 type)
	{
		if (source == nullptr) return 0;

		geo::u32 id = glCreateShader(type);

		glShaderSource(id, 1, &source, nullptr);
		glCompileShader(id);

		int resoult;
		glGetShaderiv(id, GL_COMPILE_STATUS, &resoult);
		if (resoult == GL_FALSE)
		{
			//if (type == GL_VERTEX_SHADER)
			//{
			//	LOGERROR("<VERTEX SHADER COMPILE ERROR>");
			//}
			//else if (type == GL_FRAGMENT_SHADER)
			//{
			//	LOGERROR("<FRAGMENT SHADER COMPILE ERROR>");
			//}
			//else if (type == GL_GEOMETRY_SHADER)
			//{
			//	LOGERROR("<GEOMETRY SHADER COMPILE ERROR>");
			//}

			//char msg[512];
			//glGetShaderInfoLog(id, 512, NULL, msg);
			//LOGERROR("error").Note(msg);

			glDeleteShader(id);
			return 0;
		}

		return id;
	}

	geo::u32 CreateShaderProgramFromSource(const char* vertSrc, const char* geoSrc, const char* fragSrc)
	{
		geo::u32 vs = CompileShader(vertSrc, GL_VERTEX_SHADER);
		geo::u32 gs = CompileShader(geoSrc, GL_GEOMETRY_SHADER);
		geo::u32 fs = CompileShader(fragSrc, GL_FRAGMENT_SHADER);

		geo::u32 programid = glCreateProgram();

		if (vs == 0 || fs == 0) return 0;
		if (geoSrc != nullptr && gs == 0) return 0;


		glAttachShader(programid, vs);
		if (gs != 0)
		{
			glAttachShader(programid, gs);
		}
		glAttachShader(programid, fs);

		glLinkProgram(programid);
		glValidateProgram(programid);

		glDeleteShader(vs);
		if (gs != 0)
		{
			glDeleteShader(gs);
		}
		glDeleteShader(fs);

		return programid;
	}
}


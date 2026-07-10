#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assert.h>
#include <core/logging/Log.h>
#include "LeoGraphics.h"

namespace gl
{
	static bool g_innitglad = false;

    static std::string GenarateOpenGLErrorMessage(
        GLenum source,
        GLenum type,
        GLuint id,
        GLenum severity,
        GLsizei length,
        const GLchar* message,
        const void* userParam);

	static void GLAPIENTRY OpenGL_DebugMessageCallback(
			GLenum source,
			GLenum type,
			GLuint id,
			GLenum severity,
			GLsizei length,
			const GLchar* message,
			const void* userParam)
	{
        // ignore non-significant error/warning codes
        if (id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

		LOGDEBUG(GenarateOpenGLErrorMessage(source, type, id, severity, length, message, userParam));
	}

	void GraphicsInitialization()
	{
		assert(g_innitglad == false && "GLAD is already Initialized.");

		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
            assert(false && "Failed to initialize GLAD");
		}

#if OPENGL_DEBUG
        int flags; 
        glGetIntegerv(GL_CONTEXT_FLAGS, &flags);

        if (!(flags & GL_CONTEXT_FLAG_DEBUG_BIT))
        {
            LOGERROR("OpenGL Debug output is not enable on debug.");
        }

        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
		glDebugMessageCallback(OpenGL_DebugMessageCallback, 0);
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
#endif

		// print OpenGL version
		LOGINFO("Using OpenGL " << (const char*)glGetString(GL_VERSION));
		g_innitglad = true;
	}

    static std::string GenarateOpenGLErrorMessage(
        GLenum source,
        GLenum type,
        GLuint id,
        GLenum severity,
        GLsizei length,
        const GLchar* message,
        const void* userParam)
    {
        std::ostringstream message_stream;

        message_stream << "OpenGL ERROR " << id << "\n";
        message_stream << message << "\n";

        switch (source)
        {
        case GL_DEBUG_SOURCE_API:             message_stream << "Source: API"; break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   message_stream << "Source: Window System"; break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER: message_stream << "Source: Shader Compiler"; break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:     message_stream << "Source: Third Party"; break;
        case GL_DEBUG_SOURCE_APPLICATION:     message_stream << "Source: Application"; break;
        case GL_DEBUG_SOURCE_OTHER:           message_stream << "Source: Other"; break;
        } 
        message_stream << "\n";

        switch (type)
        {
        case GL_DEBUG_TYPE_ERROR:               message_stream << "Type: Error"; break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: message_stream << "Type: Deprecated Behaviour"; break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  message_stream << "Type: Undefined Behaviour"; break;
        case GL_DEBUG_TYPE_PORTABILITY:         message_stream << "Type: Portability"; break;
        case GL_DEBUG_TYPE_PERFORMANCE:         message_stream << "Type: Performance"; break;
        case GL_DEBUG_TYPE_MARKER:              message_stream << "Type: Marker"; break;
        case GL_DEBUG_TYPE_PUSH_GROUP:          message_stream << "Type: Push Group"; break;
        case GL_DEBUG_TYPE_POP_GROUP:           message_stream << "Type: Pop Group"; break;
        case GL_DEBUG_TYPE_OTHER:               message_stream << "Type: Other"; break;
        } 
        message_stream << "\n";

        switch (severity)
        {
        case GL_DEBUG_SEVERITY_HIGH:         message_stream << "Severity: high"; break;
        case GL_DEBUG_SEVERITY_MEDIUM:       message_stream << "Severity: medium"; break;
        case GL_DEBUG_SEVERITY_LOW:          message_stream << "Severity: low"; break;
        case GL_DEBUG_SEVERITY_NOTIFICATION: message_stream << "Severity: notification"; break;
        } 
        message_stream << "\n";
        return message_stream.str();
    }
}
#include <GL/glew.h>
#include <cstdio>
#include <Windows.h>
#include <iostream>

#include <string>
#include <fstream>
#include <streambuf>


namespace shader
{
	extern
		GLuint load(const char * filename, GLenum shader_type, bool check_errors)
	{
		GLuint result = 0;
		size_t filesize;

		std::ifstream t(filename);
		std::string str((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());

		result = glCreateShader(shader_type);

		if (!result)
			goto fail_shader_alloc;
		const char *data = str.c_str();
		glShaderSource(result, 1, &data, NULL);

		glCompileShader(result);

		if (check_errors)
		{
			GLint status = 0;
			glGetShaderiv(result, GL_COMPILE_STATUS, &status);

			if (!status)
			{
				char buffer[4096];
				glGetShaderInfoLog(result, 4096, NULL, buffer);
#ifdef _WIN32
				std::cout << filename << ":" << buffer << std::endl;
#else
				fprintf(stderr, "%s: %s\n", filename, buffer);
#endif
				goto fail_compile_shader;
			}
		}

		return result;

	fail_compile_shader:
		glDeleteShader(result);

	fail_shader_alloc:;
	fail_data_alloc:
		return result;
	}

	GLuint from_string(const char * source,
		GLenum shader_type,
		bool check_errors)
	{
		GLuint sh;

		sh = glCreateShader(shader_type);

		const char * strings[] = { source };
		glShaderSource(sh, 1, strings, nullptr);

		glCompileShader(sh);

		if (check_errors)
		{
			GLint status = 0;
			glGetShaderiv(sh, GL_COMPILE_STATUS, &status);

			if (!status)
			{
				char buffer[4096];
				glGetShaderInfoLog(sh, 4096, NULL, buffer);
#ifdef _WIN32
				std::cout << buffer << std::endl;
#else
				fprintf(stderr, "%s\n", buffer);
#endif
				goto fail_compile_shader;
			}
		}

		return sh;

	fail_compile_shader:
		glDeleteShader(sh);

		return 0;
	}

}

namespace program
{

	GLuint link_from_shaders(const GLuint * shaders,
		int shader_count,
		bool delete_shaders,
		bool check_errors)
	{
		int i;

		GLuint program;

		program = glCreateProgram();

		for (i = 0; i < shader_count; i++)
		{
			glAttachShader(program, shaders[i]);
		}

		glLinkProgram(program);

		if (check_errors)
		{
			GLint status;
			glGetProgramiv(program, GL_LINK_STATUS, &status);

			if (!status)
			{
				char buffer[4096];
				glGetProgramInfoLog(program, 4096, NULL, buffer);
#ifdef _WIN32
				std::cout << buffer << std::endl;
#endif
				glDeleteProgram(program);
				return 0;
			}
		}

		if (delete_shaders)
		{
			for (i = 0; i < shader_count; i++)
			{
				glDeleteShader(shaders[i]);
			}
		}

		return program;
	}

}
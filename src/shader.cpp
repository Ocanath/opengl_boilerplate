#include "shader.h"
#include <glad/gl.h>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdio>
#include <string>

static std::string readFile(const char* path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error(std::string("Shader: cannot open ") + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

unsigned int Shader::compile(const char* path, unsigned int type)
{
    std::string src = readFile(path);
    const char* csrc = src.c_str();

    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &csrc, nullptr);
    glCompileShader(shader);

    int ok;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        fprintf(stderr, "Shader compile error (%s):\n%s\n", path, log);
        glDeleteShader(shader);
        throw std::runtime_error("Shader compilation failed");
    }
    return shader;
}

void Shader::checkLinkErrors(unsigned int program)
{
    int ok;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        fprintf(stderr, "Shader link error:\n%s\n", log);
        throw std::runtime_error("Shader link failed");
    }
}

Shader::Shader(const char* vertPath, const char* fragPath)
{
    unsigned int vert = compile(vertPath, GL_VERTEX_SHADER);
    unsigned int frag = compile(fragPath, GL_FRAGMENT_SHADER);

    id = glCreateProgram();
    glAttachShader(id, vert);
    glAttachShader(id, frag);
    glLinkProgram(id);
    checkLinkErrors(id);

    glDeleteShader(vert);
    glDeleteShader(frag);
}

Shader::~Shader()
{
    if (id) glDeleteProgram(id);
}

void Shader::use() const { glUseProgram(id); }

void Shader::setInt(const char* name, int v) const {
    glUniform1i(glGetUniformLocation(id, name), v);
}
void Shader::setFloat(const char* name, float v) const {
    glUniform1f(glGetUniformLocation(id, name), v);
}
void Shader::setVec3(const char* name, const glm::vec3& v) const {
    glUniform3fv(glGetUniformLocation(id, name), 1, glm::value_ptr(v));
}
void Shader::setMat4(const char* name, const glm::mat4& v) const {
    glUniformMatrix4fv(glGetUniformLocation(id, name), 1, GL_FALSE, glm::value_ptr(v));
}

// Array element helpers using snprintf to build "name[idx]"
void Shader::setVec3i(const char* name, int idx, const glm::vec3& v) const {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s[%d]", name, idx);
    glUniform3fv(glGetUniformLocation(id, buf), 1, glm::value_ptr(v));
}
void Shader::setFloati(const char* name, int idx, float v) const {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s[%d]", name, idx);
    glUniform1f(glGetUniformLocation(id, buf), v);
}

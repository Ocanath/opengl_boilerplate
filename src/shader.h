#pragma once
#include <string>
#include <glm/glm.hpp>

class Shader
{
public:
    unsigned int id;

    Shader(const char* vertPath, const char* fragPath);
    ~Shader();

    void use() const;

    void setInt  (const char* name, int value)               const;
    void setFloat(const char* name, float value)             const;
    void setVec3 (const char* name, const glm::vec3& value)  const;
    void setMat4 (const char* name, const glm::mat4& value)  const;

    // Array element setters (index variant)
    void setVec3i (const char* name, int idx, const glm::vec3& value) const;
    void setFloati(const char* name, int idx, float value)            const;

private:
    static unsigned int compile(const char* path, unsigned int type);
    static void checkLinkErrors(unsigned int program);
};

#pragma once
#include <vector>
#include <glad/glad.h>

class IndexBuffer
{
private:
	unsigned int mID;
	unsigned int mCount;

public:
	IndexBuffer(const unsigned int * data, unsigned int count, GLuint usage = GL_STATIC_DRAW);
	~IndexBuffer();

	void Bind() const;
	void UpdateData(const unsigned int* data, unsigned int count, GLuint usage = GL_STATIC_DRAW);
	void UpdateSubData(const unsigned int* data, unsigned int count_offset, unsigned int count);
	std::vector<unsigned int> GetSubData(unsigned int count_offset, unsigned int count);
	bool IsBinding() const;
	void Unbind() const;
	inline GLuint GetID() const { return mID; }
	inline unsigned int GetCount() const {	return mCount;	}

	static GLuint CurrentBinding();
};
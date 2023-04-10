#include "IndexBuffer.h"

IndexBuffer::IndexBuffer(const unsigned int * data, unsigned int count, GLuint usage)
	: mCount(count)
{
	glGenBuffers(1, &mID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), data, usage);
}

IndexBuffer::~IndexBuffer()
{
	glDeleteBuffers(1, &mID);
}

void IndexBuffer::Bind() const
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mID);
}

void IndexBuffer::UpdateData(const unsigned int* data, unsigned int count, GLuint usage)
{
	Bind();
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), data, usage);
	mCount = count;
}

void IndexBuffer::UpdateSubData(const unsigned int* data, unsigned int count_offset, unsigned int count)
{
	Bind();
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, count_offset * sizeof(unsigned int), count * sizeof(unsigned int), data);
}

std::vector<unsigned int> IndexBuffer::GetSubData(unsigned int count_offset, unsigned int count)
{
	std::vector<unsigned int> buf;
	buf.resize(count);
	glGetBufferSubData(
		GL_ELEMENT_ARRAY_BUFFER,
		count_offset * sizeof(unsigned int), 
		count * sizeof(unsigned int),
		const_cast<void*>(static_cast<const void*>(buf.data())));
	return buf;
}

bool IndexBuffer::IsBinding() const
{
	return CurrentBinding() == mID;
}

void IndexBuffer::Unbind() const
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

GLuint IndexBuffer::CurrentBinding()
{
	GLint result = 0;
	glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &result);
	return result;
}

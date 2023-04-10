#pragma once
#include <glm/glm.hpp>

class Triangle
{
public:
    Triangle() = default;
    Triangle( unsigned int a, unsigned int b, unsigned int c )
        : a( a ), b( b ), c( c ) {}

    unsigned int a;
    unsigned int b;
    unsigned int c;

    unsigned int& operator[]( int i ) noexcept
    {
        switch (i)
        {
        case 0:
            return a;
        case 1:
            return b;
        case 2:
            return c;
        }
    }
    const unsigned int& operator[]( int i ) const noexcept
    {
        switch (i)
        {
        case 0:
            return a;
        case 1:
            return b;
        case 2:
            return c;
        }
    }
};

class TriPoint
{
public:
    TriPoint() = default;
    TriPoint( glm::vec3 a, glm::vec3 b, glm::vec3 c ) : a( a ), b( b ), c( c )
    {}

    glm::vec3& operator[]( int i ) noexcept
    {
        switch (i)
        {
        case 0:
            return a;
        case 1:
            return b;
        case 2:
            return c;
        }
        assert( false );
    }
    const glm::vec3& operator[]( int i ) const noexcept
    {
        switch (i)
        {
        case 0:
            return a;
        case 1:
            return b;
        case 2:
            return c;
        }
        assert( false );
    }

    glm::vec3 a;
    glm::vec3 b;
    glm::vec3 c;

};

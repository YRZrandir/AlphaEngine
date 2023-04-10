#include "Image.h"
#include "stb_image.h"
#include <exception>
#include <span>

Image<unsigned char> LoadImageFile( const std::string& path )
{
    stbi_set_flip_vertically_on_load( true );
    int width = 0;
    int height = 0;
    int nchannel = 0;
    unsigned char* data = stbi_load( path.c_str(), &width, &height, &nchannel, 0 );
    if (!data)
    {
        std::cout << "Failed to load image " + path << std::endl;
        throw std::runtime_error( "Failed to load image " + path );
    }

    Image<unsigned char> img( width, height, nchannel );
    std::copy( data, data + (static_cast<size_t>(width) * height * nchannel), img.Data() );
    stbi_image_free( data );

    return img;
}

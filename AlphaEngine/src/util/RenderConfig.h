#pragma once
class RenderConfig
{
public:
    enum class MatType { Texture, VtxColor, PlainColor };
    bool _draw_face = true;
    bool _draw_line = false;
    bool _draw_points = false;
    MatType _face_mattype = MatType::Texture;
    MatType _line_mattype = MatType::PlainColor;
    MatType _point_mattype = MatType::PlainColor;
};
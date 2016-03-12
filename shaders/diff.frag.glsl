#version 130
// ^ Change this to version 130 if you have compatibility issues

#define amb 0.3

//these are the interpolated values out of the rasterizer, so you can't know
//their specific values without knowing the vertices that contributed to them
in vec4 fs_Normal;
in vec4 fs_Color;

out vec4 out_Color;

void main()
{
    vec3 up = vec3(0.0, 0.0, 1.0); // we live in a z-up world
    float sun_mag = max(dot(up, fs_Normal.xyz), amb);

    out_Color = vec4(vec3(sun_mag * fs_Color), 1.0);
}

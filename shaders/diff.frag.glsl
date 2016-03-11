#version 130
// ^ Change this to version 130 if you have compatibility issues

//these are the interpolated values out of the rasterizer, so you can't know
//their specific values without knowing the vertices that contributed to them
in vec4 fs_Normal;
in vec4 fs_LightVector;
in vec4 fs_Color;
in vec3 fs_lightColor;
in vec3 fs_eyeVector;

out vec4 out_Color;

void main()
{
    // Material base color (before shading)
    vec3 surfaceColor = fs_Color.rgb;

    // Calculate the diffuse term
    // Id = Ii (C / D) = Ii cos(theta) = Ii (L dot N)
    float diffuseTerm = dot(normalize(fs_LightVector), normalize(fs_Normal));
    // Avoid negative lighting values
    diffuseTerm = clamp(diffuseTerm, 0, 1);

    // Calculate the specular term using blinn-phong
    vec3 N = normalize(fs_Normal.xyz);
    vec3 D = normalize(fs_LightVector.xyz * -1.0f); // light to point
    vec3 R = D - 2 * N * dot(normalize(D), normalize(N));

    float specularTerm = dot(normalize(R), normalize(fs_eyeVector));
    specularTerm = pow(specularTerm, 180);
    specularTerm = clamp(specularTerm, 0, 1);

    float ambientTerm = 0.2;

    float diffuse = diffuseTerm + ambientTerm;

    // Compute final shaded color
    out_Color = vec4(fs_lightColor * (surfaceColor * diffuse + specularTerm), fs_Color.a);
}

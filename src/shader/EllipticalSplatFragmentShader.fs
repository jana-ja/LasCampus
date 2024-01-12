#version 330 core

uniform vec4 vp;
uniform vec3 zb;

in vec3 v2f_color;
in vec3 v2f_center;
//in float v2f_radius;
in vec3  v2f_tangent0;
in vec3  v2f_tangent1;
in vec3 v2f_light_dir;

out vec4 f_color;

vec3 phong_lighting(const vec3  normal, const vec3  color, const vec3 view_dir, const vec3 light_dir)
{
    const float  ambient   = 0.1; // 0.1
    const float  diffuse   = 0.8;
    const float  specular  = 0.6;
    const float  shininess = 100.0;
    //const vec3   light_dir = vec3( 1.0, 1.0, 1.0);
    //const vec3   V         = vec3(0,0,-1); // view direction
    vec3   V         = view_dir;
    vec3 L, R, N = normalize(normal);
    float NL, RV;

    vec3 result = ambient *  color; // 0.1 weg?

    L = normalize(light_dir);
    NL = dot(N, -L);
    if (NL > 0.0)
    {
        result += diffuse * NL * color;

        R  = normalize(reflect(L, N));
        RV = dot(R, V);
        if (RV > 0.0) result += vec3( specular * pow(RV, shininess) );
    }

    result = clamp(result, 0.0, 1.0);

    return result;
}

void main()
{
    // point on near plane
    vec3 qn = vec3(gl_FragCoord.xy * vp.xy + vp.zw, zb.z); // z = -z_near

    // normal in eye space
    vec3 u = v2f_tangent0;
    vec3 v = v2f_tangent1;
    vec3 n = normalize(cross(u,v)); // testen TODO problem: t1 t2 sind nicht immer gleiche reihenfolge (uhrzeigersinn), dadurch kann normale hier geflippt sein
    // wahrscheinlich einfacher die auch noch als daten rüber zu schieben, ist auch eh praktisch für f1 modus. sonst müsste ich die iwie switchen. vllt ist das aber auch noch für was anderes wichtig?

    // point in eye space
    vec3 q = qn * (dot(v2f_center, n) / dot(qn, n));

    // clamp based on radius
    vec3 qc = q - v2f_center;
    if (pow(dot(qc,u), 2.0) + pow(dot(qc,v), 2.0) > 1.0) discard;
    //if (distance(q, v2f_center) > v2f_radius) discard;

    // lighting
    f_color = vec4(phong_lighting(n, v2f_color, normalize(v2f_center), normalize(v2f_light_dir)), 1.0);

    // depth correction
	gl_FragDepth = zb.y * q.z + zb.x;
}

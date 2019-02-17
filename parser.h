#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>

namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        float x, y, z;
    public:
        Vec3f operator=(Vec3f a){
            this->x = a.x;
            this->y = a.y;
            this->z = a.z;
            return (*this);
        }
        Vec3f operator+(const Vec3f a) const{
            Vec3f z;
            z.x = a.x + this->x;
            z.y = a.y + this->y;
            z.z = a.z + this->z;
            return z;
        }
        Vec3f operator-(Vec3f a){
            Vec3f temp;
            temp.x = this->x - a.x;
            temp.y = this->y - a.y;
            temp.z = this->z - a.z;
            return temp;
        }
        Vec3f operator*(const float d) const{
            Vec3f z;
            z.x = this->x * d;
            z.y = this->y * d;
            z.z = this->z * d;
            return z;
        }
        Vec3f operator/(const float d) const{
            Vec3f z;
            z.x = this->x / d;
            z.y = this->y / d;
            z.z = this->z / d;
            return z;
        }
        Vec3f operator*=(const Vec3f a) const{ //cross product
            Vec3f z;
            z.x = this->y * a.z - this->z * a.y;
            z.y = this->z * a.x - this->x * a.z;
            z.z = this->x * a.y - this->y * a.x;
            return z; 
        }
        float operator+=(const Vec3f a) const{ //dot product
            float z = 0;
            z = this->x * a.x;
            z = z + this->y * a.y;
            z = z + this->z * a.z;
            return z; 
        }
        Vec3f operator%(const Vec3f a) const{
            Vec3f z;
            z.x = this->x * a.x;
            z.y = this->y * a.y;
            z.z = this->z * a.z;
            return z;
        }

    };

    struct Vec3i
    {
        int x, y, z;
    public:
        Vec3i operator=(Vec3i a){
            this->x = a.x;
            this->y = a.y;
            this->z = a.z;
            return (*this);
        }
        Vec3i operator+(const Vec3i a) const{
            Vec3i z;
            z.x = a.x + this->x;
            z.y = a.y + this->y;
            z.z = a.z + this->z;
            return z;
        }
        Vec3i operator-(Vec3i a){
            Vec3i temp;
            temp.x = this->x - a.x;
            temp.y = this->y - a.y;
            temp.z = this->z - a.z;
            return temp;
        }
        Vec3i operator*(const float d) const{
            Vec3i z;
            z.x = this->x * d;
            z.y = this->y * d;
            z.z = this->z * d;
            return z;
        }
        Vec3i operator*=(const Vec3i a) const{ //cross product
            Vec3i z;
            z.x = this->y * a.z - this->z * a.y;
            z.y = this->z * a.x - this->x * a.z;
            z.z = this->x * a.y - this->y * a.x;
            return z; 
        }
        int operator+=(const Vec3i a) const{ //dot product
            int z = 0;
            z = this->x * a.x;
            z = z + this->y * a.y;
            z = z + this->z * a.z;
            return z; 
        }

    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
        public:
        Face operator=(Face a){
            this->v0_id = a.v0_id;
            this->v1_id = a.v1_id;
            this->v2_id = a.v2_id;
            return (*this);
        }
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
    public:
        Triangle operator=(Triangle a){
            this->material_id = a.material_id;
            this->indices = a.indices;
            return (*this);
        }
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    public:
        Sphere operator=(Sphere a){
            this->material_id = a.material_id;
            this->center_vertex_id = a.center_vertex_id;
            this->radius = a.radius;
            return (*this);
        }
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        //Functions
        void loadFromXml(const std::string& filepath);
    };
}

#endif

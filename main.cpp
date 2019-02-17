#include <iostream>
#include <math.h>
#include "parser.h"
#include "ppm.h"
#include <algorithm>
#include <limits>

typedef unsigned char RGB[3];
using namespace parser;
const float epsilon = 1e-5;

void normalize(Vec3f & a){
    a = a / sqrt( a += a);
}

class Ray{
    public:
        Vec3f dir, pos;
        Ray(Vec3f direction, Vec3f position){
            normalize(direction);
            this->dir = direction;
            this->pos = position;
        }
        void intersects_sphere(Sphere & sp, const Scene & scene, double &t_min, Sphere* & objsphere) {
            Vec3f center = scene.vertex_data[sp.center_vertex_id-1];
            double a = dir += dir;            
            double b = ( (pos - center) += dir) * 2 ;            
            double c = ((pos - center) += (pos - center)) - (sp.radius) * (sp.radius);            
            double disc = (b*b) - 4*a*c;
            if(disc > epsilon) {
                double t0 = ( (-1)*b + sqrt(disc)) / 2*a;
                double t1 = ( (-1)*b - sqrt(disc)) / 2*a;
                float t = std::numeric_limits<float>::infinity();

                if(t0 > epsilon && t1 < epsilon){                    
                    t = t0; 
                }    
                else if(t0 < epsilon && t1 > epsilon){                    
                    t = t1;
                }    
                else if(t0 > epsilon && t1 > epsilon){                    

                    if(t1 >= t0){
                        t = t0;
                    }
                    else{
                        t = t1;
                    }
                }                                  
                if(t < t_min) {
                    t_min = t ;                                        
                    objsphere = &sp ;
                }
            }
        }

        bool intersects(const Scene & scene, float max_t, Vec3f camera_pos) {
            for (int ot = 0; ot < scene.spheres.size() ; ++ot) {
                Vec3f center = scene.vertex_data[scene.spheres[ot].center_vertex_id-1];
                double a = dir += dir;            
                double b = ( (pos - center) += dir) * 2 ;            
                double c = ((pos - center) += (pos - center)) - (scene.spheres[ot].radius) * (scene.spheres[ot].radius);            
                double disc = (b*b) - 4*a*c;
                double t = 0;
                float uzunluk = 0;
                if(disc > epsilon) {
                    double t0 = ( (-1)*b + sqrt(disc)) / 2*a;
                    double t1 = ( (-1)*b - sqrt(disc)) / 2*a;
                    if(t0 > epsilon && t1 < epsilon){                    
                        t = t0;     
                        Vec3f vec = camera_pos - (dir * t + pos);
                        uzunluk = sqrt(vec += vec);
                        if( (uzunluk < max_t) && uzunluk > epsilon ){
                            return true;
                        }

                    }    
                    else if(t0 < epsilon && t1 > epsilon){                    
                        t = t1;
                        Vec3f vec = camera_pos - (dir * t + pos);
                        uzunluk = sqrt(vec += vec);
                        if( (uzunluk < max_t) && uzunluk > epsilon ){
                            return true;
                        }                        
                    }    
                    else if(t0 > epsilon && t1 > epsilon){                    

                        if(t1 >= t0){
                            t = t0;
                            Vec3f vec = camera_pos - (dir * t + pos);
                            uzunluk = sqrt(vec += vec);
                            if( (uzunluk < max_t) && uzunluk > epsilon ){
                                return true;
                            }
                        }
                        else{
                            t = t1;
                            Vec3f vec = camera_pos - (dir * t + pos);
                            uzunluk = sqrt(vec += vec);
                            if( (uzunluk < max_t) && uzunluk > epsilon ){
                                return true;
                            }

                        }
                    }   
                }
            }

            for (int ot = 0; ot < scene.triangles.size() ; ++ot){
                float a_x = scene.vertex_data[scene.triangles[ot].indices.v0_id-1].x ;
                float a_y = scene.vertex_data[scene.triangles[ot].indices.v0_id-1].y ;
                float a_z = scene.vertex_data[scene.triangles[ot].indices.v0_id-1].z ;
                float b_x = scene.vertex_data[scene.triangles[ot].indices.v1_id-1].x ;
                float b_y = scene.vertex_data[scene.triangles[ot].indices.v1_id-1].y ;
                float b_z = scene.vertex_data[scene.triangles[ot].indices.v1_id-1].z ;
                float c_x = scene.vertex_data[scene.triangles[ot].indices.v2_id-1].x ;
                float c_y = scene.vertex_data[scene.triangles[ot].indices.v2_id-1].y ;
                float c_z = scene.vertex_data[scene.triangles[ot].indices.v2_id-1].z ;

                float a = a_x - b_x ;
                float b = a_y - b_y ;
                float c = a_z - b_z ;
                float d = a_x - c_x ;
                float e = a_y - c_y ;
                float f = a_z - c_z ;

                float j = a_x - pos.x ;
                float k = a_y - pos.y ;
                float l = a_z - pos.z ;

                float ak_minus_jb = a * k - j * b ;
                float jc_minus_al = j * c - a * l ;
                float bl_minus_kc = b * l - k * c ;


                float ei_minus_hf = e * dir.z - dir.y * f ;
                float gf_minus_di = dir.x * f - d * dir.z ;
                float dh_minus_eg = d * dir.y - e * dir.x ;

                float M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg ;

                float t = (-1) * (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc)/M ;
                Vec3f vec = camera_pos - (dir * t + pos);
                float uzunluk = sqrt(vec += vec);                
                if(t >= epsilon && uzunluk <= max_t ) {  //t karsilastirmasi degil uzunluk karsilastirmasi yapilacak 
                    float gamma = (dir.z * ak_minus_jb + dir.y * jc_minus_al + dir.x * bl_minus_kc)/M ;
                    if(gamma >= 0 && gamma <= 1) {
                        float beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg)/M ;
                        if(beta >= 0 && beta + gamma <= 1 ) {
                            return true;
                        }
                    }
                }            
            }

            for (int ot = 0; ot < scene.meshes.size() ; ++ot){
                for(int mot = 0; mot < scene.meshes[ot].faces.size() ; ++mot){
                    float a_x = scene.vertex_data[scene.meshes[ot].faces[mot].v0_id-1].x ;
                    float a_y = scene.vertex_data[scene.meshes[ot].faces[mot].v0_id-1].y ;
                    float a_z = scene.vertex_data[scene.meshes[ot].faces[mot].v0_id-1].z ;
                    float b_x = scene.vertex_data[scene.meshes[ot].faces[mot].v1_id-1].x ;
                    float b_y = scene.vertex_data[scene.meshes[ot].faces[mot].v1_id-1].y ;
                    float b_z = scene.vertex_data[scene.meshes[ot].faces[mot].v1_id-1].z ;
                    float c_x = scene.vertex_data[scene.meshes[ot].faces[mot].v2_id-1].x ;
                    float c_y = scene.vertex_data[scene.meshes[ot].faces[mot].v2_id-1].y ;
                    float c_z = scene.vertex_data[scene.meshes[ot].faces[mot].v2_id-1].z ;

                    float a = a_x - b_x ;
                    float b = a_y - b_y ;
                    float c = a_z - b_z ;
                    float d = a_x - c_x ;
                    float e = a_y - c_y ;
                    float f = a_z - c_z ;

                    float j = a_x - pos.x ;
                    float k = a_y - pos.y ;
                    float l = a_z - pos.z ;

                    float ak_minus_jb = a * k - j * b ;
                    float jc_minus_al = j * c - a * l ;
                    float bl_minus_kc = b * l - k * c ;


                    float ei_minus_hf = e * dir.z - dir.y * f ;
                    float gf_minus_di = dir.x * f - d * dir.z ;
                    float dh_minus_eg = d * dir.y - e * dir.x ;

                    float M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg ;

                    float t = (-1) * (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc)/M ;
                    Vec3f vec = (dir * t + pos) - pos;
                    float uzunluk = sqrt(vec += vec);                    
                    if(t >= epsilon && uzunluk <= max_t) {  //t karsilastirmasi degil uzunluk karsilastirmasi yapilacak
                        float gamma = (dir.z * ak_minus_jb + dir.y * jc_minus_al + dir.x * bl_minus_kc)/M ;
                        if(gamma >= epsilon && gamma <= 1) {
                            float beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg)/M ;
                            if(beta >= epsilon && beta + gamma <= 1 ) {
                                return true;
                            }
                        }
                       
                    }
                }
            }

            return false;
        }    

        void intersects_triangle(Triangle & tr, const Scene & scene, double & t_min, Sphere* & objsphere, Triangle* & objtriangle) {
            float a_x = scene.vertex_data[tr.indices.v0_id-1].x ;
            float a_y = scene.vertex_data[tr.indices.v0_id-1].y ;
            float a_z = scene.vertex_data[tr.indices.v0_id-1].z ;
            float b_x = scene.vertex_data[tr.indices.v1_id-1].x ;
            float b_y = scene.vertex_data[tr.indices.v1_id-1].y ;
            float b_z = scene.vertex_data[tr.indices.v1_id-1].z ;
            float c_x = scene.vertex_data[tr.indices.v2_id-1].x ;
            float c_y = scene.vertex_data[tr.indices.v2_id-1].y ;
            float c_z = scene.vertex_data[tr.indices.v2_id-1].z ;

            float a = a_x - b_x ;
            float b = a_y - b_y ;
            float c = a_z - b_z ;
            float d = a_x - c_x ;
            float e = a_y - c_y ;
            float f = a_z - c_z ;

            float j = a_x - pos.x ;
            float k = a_y - pos.y ;
            float l = a_z - pos.z ;

            float ak_minus_jb = a * k - j * b ;
            float jc_minus_al = j * c - a * l ;
            float bl_minus_kc = b * l - k * c ;


            float ei_minus_hf = e * dir.z - dir.y * f ;
            float gf_minus_di = dir.x * f - d * dir.z ;
            float dh_minus_eg = d * dir.y - e * dir.x ;

            float M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg ;

            float t = (-1) * (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc)/M ;
            if(t >= epsilon && t <= t_min ) {
                float gamma = (dir.z * ak_minus_jb + dir.y * jc_minus_al + dir.x * bl_minus_kc)/M ;
                if(gamma >= epsilon && gamma <= 1) {
                    float beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg)/M ;
                    if(beta >= epsilon && beta + gamma <= 1 ) {
                        t_min = t ;
                        objsphere = NULL ;
                        objtriangle = &tr ;
                    }
                }
            }
        }

        void intersects_mesh(Mesh & me, const Scene & scene, double & t_min, int & face_no, Sphere* & objsphere, Triangle* & objtriangle, Mesh* & objmesh) {
            for(int mot = 0; mot < me.faces.size() ; ++mot){
                float a_x = scene.vertex_data[me.faces[mot].v0_id-1].x ;
                float a_y = scene.vertex_data[me.faces[mot].v0_id-1].y ;
                float a_z = scene.vertex_data[me.faces[mot].v0_id-1].z ;
                float b_x = scene.vertex_data[me.faces[mot].v1_id-1].x ;
                float b_y = scene.vertex_data[me.faces[mot].v1_id-1].y ;
                float b_z = scene.vertex_data[me.faces[mot].v1_id-1].z ;
                float c_x = scene.vertex_data[me.faces[mot].v2_id-1].x ;
                float c_y = scene.vertex_data[me.faces[mot].v2_id-1].y ;
                float c_z = scene.vertex_data[me.faces[mot].v2_id-1].z ;

                float a = a_x - b_x ;
                float b = a_y - b_y ;
                float c = a_z - b_z ;
                float d = a_x - c_x ;
                float e = a_y - c_y ;
                float f = a_z - c_z ;

                float j = a_x - pos.x ;
                float k = a_y - pos.y ;
                float l = a_z - pos.z ;

                float ak_minus_jb = a * k - j * b ;
                float jc_minus_al = j * c - a * l ;
                float bl_minus_kc = b * l - k * c ;


                float ei_minus_hf = e * dir.z - dir.y * f ;
                float gf_minus_di = dir.x * f - d * dir.z ;
                float dh_minus_eg = d * dir.y - e * dir.x ;

                float M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg ;

                float t = (-1) * (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc)/M ;
                if(t >= epsilon && t <= t_min ) {
                    float gamma = (dir.z * ak_minus_jb + dir.y * jc_minus_al + dir.x * bl_minus_kc)/M ;
                    if(gamma >= 0 && gamma <= 1) {
                        float beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg)/M ;
                        if(beta >= 0 && beta + gamma <= 1 ) {
                            face_no = mot ;
                            t_min = t ;
                            objsphere = NULL ;
                            objtriangle = NULL ;
                            objmesh = &me ;
                        }
                    }
                }   
            }
        }

};

Vec3f send_ray(Ray & rt,Scene & scene, int depth)  {
	if(depth > scene.max_recursion_depth) {
		Vec3f pixelcolor;
		pixelcolor.x = 0 ;
		pixelcolor.y = 0 ;
		pixelcolor.z = 0 ;
		return pixelcolor ;
	}
	double t_min = std::numeric_limits<float>::infinity();
    Sphere * objsphere = NULL;
    Triangle * objtriangle = NULL;
    Mesh * objmesh = NULL;
    int face_id = 0;
    for (int ot = 0; ot < scene.spheres.size() ; ++ot) {
        rt.intersects_sphere(scene.spheres[ot], scene, t_min, objsphere) ;
    }

    for (int ot = 0; ot < scene.triangles.size() ; ++ot){
        rt.intersects_triangle(scene.triangles[ot], scene, t_min, objsphere, objtriangle) ;

    }

    for (int ot = 0; ot < scene.meshes.size() ; ++ot){
        rt.intersects_mesh(scene.meshes[ot], scene, t_min, face_id, objsphere, objtriangle, objmesh) ; 
    }

    if(objsphere != NULL){
        Vec3f pixelcolor = scene.materials[objsphere->material_id-1].ambient % scene.ambient_light;
        float phong = scene.materials[objsphere->material_id-1].phong_exponent;
        Vec3f kd = scene.materials[objsphere->material_id-1].diffuse;
        Vec3f ks = scene.materials[objsphere->material_id-1].specular;
        Vec3f km = scene.materials[objsphere->material_id-1].mirror;

        Vec3f hitpoint = ( (rt.dir * t_min) + rt.pos ) ;
        Vec3f normal =   hitpoint - scene.vertex_data[objsphere->center_vertex_id-1] ;
        normalize(normal);
        for(int liter = 0; liter < scene.point_lights.size(); liter++){
            bool intersected = false;                    
            float shadowmax = 0;
            Vec3f shadow_ray_dir = scene.point_lights[liter].position - hitpoint;
            normalize(shadow_ray_dir);
            Vec3f shadowpos = (shadow_ray_dir * scene.shadow_ray_epsilon) + hitpoint;
            
            shadowmax = sqrt( (scene.point_lights[liter].position - shadowpos) += (scene.point_lights[liter].position - shadowpos) );

            Ray shadow_ray = Ray(shadow_ray_dir, shadowpos);

            intersected = shadow_ray.intersects(scene, shadowmax, scene.point_lights[liter].position);
            if(intersected) continue;

			Vec3f l = scene.point_lights[liter].position - hitpoint ;
            float lboy = sqrt( l += l );
            normalize(l);

            Vec3f lightintensity = scene.point_lights[liter].intensity / (lboy * lboy) ;

            Vec3f h = ( l - rt.dir );
            normalize(h);

            pixelcolor = pixelcolor + ( ( kd % lightintensity) * std::max(l += normal,(float)0.0) );                        
            pixelcolor = pixelcolor + ( ( ks % lightintensity) * pow(std::max(normal += h,(float)0.0), phong) );  
        }
        if(km.x != 0 || km.y != 0 || km.z != 0) {
            Vec3f dir_new = rt.dir + normal * (normal += (rt.dir * (-1))) * 2 ;
            normalize(dir_new) ;
            hitpoint = dir_new * scene.shadow_ray_epsilon + hitpoint ;
            Ray r_new = Ray(dir_new,hitpoint) ;
            pixelcolor = pixelcolor + (km % send_ray(r_new,scene,++depth));
        }
        return pixelcolor ;
    } 

    else if(objtriangle != NULL){
        Vec3f pixelcolor = scene.materials[objtriangle->material_id-1].ambient % scene.ambient_light;
        Vec3f a = scene.vertex_data[objtriangle->indices.v0_id-1] ;
        Vec3f b = scene.vertex_data[objtriangle->indices.v1_id-1] ;
        Vec3f c = scene.vertex_data[objtriangle->indices.v2_id-1] ;
        Vec3f normal = (b-a) *= (c-a);
        normalize(normal);
        float phong = scene.materials[objtriangle->material_id-1].phong_exponent;
        Vec3f kd = scene.materials[objtriangle->material_id-1].diffuse;
        Vec3f ks = scene.materials[objtriangle->material_id-1].specular;
        Vec3f km = scene.materials[objtriangle->material_id-1].mirror;
        Vec3f hitpoint = ( (rt.dir * t_min) + rt.pos ) ;
        for(int liter = 0; liter < scene.point_lights.size(); liter++){
            bool intersected = false;                    
            float shadowmax = 0;
            Vec3f hitpoint = ( (rt.dir * t_min) + rt.pos ) ;
            Vec3f shadow_ray_dir = scene.point_lights[liter].position - hitpoint;
            normalize(shadow_ray_dir);
            Vec3f shadowpos = (shadow_ray_dir * scene.shadow_ray_epsilon) + hitpoint;
            
            shadowmax = sqrt( (scene.point_lights[liter].position - shadowpos) += (scene.point_lights[liter].position - shadowpos) );

            Ray shadow_ray = Ray(shadow_ray_dir, shadowpos);

            intersected = shadow_ray.intersects(scene, shadowmax, scene.point_lights[liter].position);
            if(intersected) continue;



            Vec3f l = scene.point_lights[liter].position - hitpoint ;
            float lboy = sqrt( l += l );
            l = l / lboy ;

            Vec3f lightintensity = scene.point_lights[liter].intensity / (lboy * lboy) ;

            Vec3f vplusl = ( l - (rt.dir / sqrt( rt.dir += rt.dir ) ) );
            Vec3f h = vplusl / sqrt( vplusl += vplusl );

            pixelcolor = pixelcolor + ( ( kd % lightintensity) * std::max(l += normal,(float)0.0) );

            pixelcolor = pixelcolor + ( ( ks % lightintensity) * pow(std::max(normal += h,(float)0.0), phong) );
        }
        if(km.x != 0 || km.y != 0 || km.z != 0) {
            Vec3f dir_new = rt.dir + normal * (normal += (rt.dir * (-1))) * 2 ;
            normalize(dir_new) ;
            hitpoint = dir_new * scene.shadow_ray_epsilon + hitpoint ;
            Ray r_new = Ray(dir_new,hitpoint) ;
            pixelcolor = pixelcolor + ( km % send_ray(r_new,scene,++depth));
        }
        return pixelcolor ;
    }

    else if(objmesh != NULL){
        Vec3f pixelcolor = scene.materials[objmesh->material_id-1].ambient % scene.ambient_light;
        Vec3f a = scene.vertex_data[objmesh->faces[face_id].v0_id-1] ;
        Vec3f b = scene.vertex_data[objmesh->faces[face_id].v1_id-1] ;
        Vec3f c = scene.vertex_data[objmesh->faces[face_id].v2_id-1] ;
        Vec3f normal = (b-a) *= (c-a);
        normalize(normal);
        float phong = scene.materials[objmesh->material_id-1].phong_exponent;
        Vec3f kd = scene.materials[objmesh->material_id-1].diffuse;
        Vec3f ks = scene.materials[objmesh->material_id-1].specular;
        Vec3f km = scene.materials[objmesh->material_id-1].mirror;
        Vec3f hitpoint = ( (rt.dir * t_min) + rt.pos ) ;
        for(int liter = 0; liter < scene.point_lights.size(); liter++){
            bool intersected = false;                    
            float shadowmax = 0;
            Vec3f shadow_ray_dir = scene.point_lights[liter].position - hitpoint;
            normalize(shadow_ray_dir);
            Vec3f shadowpos = (shadow_ray_dir * scene.shadow_ray_epsilon) + hitpoint;

            shadowmax = sqrt( (scene.point_lights[liter].position - shadowpos) += (scene.point_lights[liter].position - shadowpos) );

            Ray shadow_ray = Ray(shadow_ray_dir, shadowpos);

            intersected = shadow_ray.intersects(scene, shadowmax, scene.point_lights[liter].position);

            if(intersected)
                continue;
        

            

            Vec3f l = scene.point_lights[liter].position - hitpoint ;
            float lboy = sqrt( l += l );
            l = l / lboy ;

            Vec3f lightintensity = scene.point_lights[liter].intensity / (lboy * lboy) ;
            Vec3f vplusl = ( l - rt.dir);
            Vec3f h = vplusl / sqrt( vplusl += vplusl );

            pixelcolor = pixelcolor + ( ( kd % lightintensity) * std::max(l += normal,(float)0.0) );
            pixelcolor = pixelcolor + ( ( ks % lightintensity) * pow(std::max(normal += h,(float)0.0), phong) );
            
        }
        if(km.x != 0 || km.y != 0 || km.z != 0) {
            Vec3f dir_new = rt.dir + normal * (normal += (rt.dir * (-1))) * 2 ;
            normalize(dir_new) ;
            hitpoint = dir_new * scene.shadow_ray_epsilon + hitpoint ;
            Ray r_new = Ray(dir_new,hitpoint) ;
            pixelcolor = pixelcolor + ( km % send_ray(r_new,scene,++depth));
        }

        return pixelcolor;
    }

    else if(t_min == std::numeric_limits<float>::infinity()) {
        Vec3f pixelcolor;
		pixelcolor.x = 0 ;
		pixelcolor.y = 0 ;
		pixelcolor.z = 0 ;
		return pixelcolor ;
    }                
}

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    Scene scene;
    scene.loadFromXml(argv[1]);

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.
    
    for (int it = 0; it < scene.cameras.size() ; it++){
        unsigned char* image = new unsigned char [scene.cameras[it].image_width * scene.cameras[it].image_height * 3];
        int pixeliter = 0;
        float left = scene.cameras[it].near_plane.x;
        float right = scene.cameras[it].near_plane.y;
        float bottom = scene.cameras[it].near_plane.z;
        float top = scene.cameras[it].near_plane.w;

        for(int i = 0; i < scene.cameras[it].image_height; i++){
            for(int j = 0; j < scene.cameras[it].image_width; j++){
            	int depth = 0 ;

                Vec3f m = scene.cameras[it].position + (scene.cameras[it].gaze * scene.cameras[it].near_distance );
                Vec3f u = ( scene.cameras[it].up *= (scene.cameras[it].gaze * (-1)) );

                Vec3f lu = u * left;
                Vec3f tv = ( scene.cameras[it].up ) * top;

                Vec3f q = m + lu + tv;

                float su = (right - left) * (j + 0.5) / scene.cameras[it].image_width;
                float sv = (top - bottom) * (i + 0.5) / scene.cameras[it].image_height;

                Vec3f d = ( q + (u * su) - (scene.cameras[it].up * sv) ) - scene.cameras[it].position;
                Ray rt = Ray(d, scene.cameras[it].position);

                Vec3f pixelcolor = send_ray(rt,scene,depth) ;

                if(pixelcolor.x > 255) pixelcolor.x = 255;
                if(pixelcolor.y > 255) pixelcolor.y = 255;
                if(pixelcolor.z > 255) pixelcolor.z = 255;

                image[pixeliter++] = round(pixelcolor.x);
                image[pixeliter++] = round(pixelcolor.y);
                image[pixeliter++] = round(pixelcolor.z);
            }
        }
        write_ppm(scene.cameras[it].image_name.c_str(), image, scene.cameras[it].image_width, scene.cameras[it].image_height);
    }







}

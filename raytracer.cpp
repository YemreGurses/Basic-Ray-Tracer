#include <iostream>
#include "parser.h"
#include "ppm.h"

using namespace parser;

class ray{
	public:
		Vec3f d, e;
		ray(Vec3f dir, Vec3f pos){
			this->d = dir;
			this->e = pos;
		}
};
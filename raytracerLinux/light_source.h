
#ifndef _LIGHT_
#define _LIGHT_

#include "util.h"

// Base class for a light source.  You could define different types
// of lights here, but point light is sufficient for most scenes you
// might want to render.  Different light sources shade the ray 
// differently.
class LightSource {
public:
	virtual void shade( Ray3D& ) = 0;
	virtual Point3D get_position() const = 0; 
	static RenderType RENDER_TYPE;
};

// A point light is defined by its position in world space and its specular and diffuse colours
class PointLight : public LightSource {
public:
	PointLight( Point3D pos, Colour col ) : _pos(pos), _col_diffuse(col), _col_specular(col), _col_ambient(col) {}
	PointLight( Point3D pos, Colour diffuse, Colour specular, Colour ambient ) 
	: _pos(pos), _col_diffuse(diffuse), _col_specular(specular), _col_ambient(ambient) {}
	
	Point3D get_position() const { return _pos; }
	void setPosition(Point3D pos) {
		_pos = pos;
	}
	void shade( Ray3D& ray );
	
private:
	Point3D _pos;
	Colour _col_diffuse; 
	Colour _col_specular; 
	Colour _col_ambient;
};

#endif

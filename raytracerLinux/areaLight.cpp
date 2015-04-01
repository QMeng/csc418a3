#include "areaLight.h"
#include <stdlib.h>

void AreaLight::shade( Ray3D& ray ) {
    PointLight pointLight = PointLight(Point3D(), color);

    for (unsigned int i = 0; i < NUM_SHADOW_RAYS; i++) {

        double us = (double)rand() / RAND_MAX;
        double vs = (double)rand() / RAND_MAX;
        
        Point3D pol = pos + us * uLength * u + vs * vLength * v;  
        Vector3D lto = ray.intersection.point - pol;
        lto.normalize();
        Ray3D lightToObjWorldSpace = Ray3D(pol, lto);
        r.traverseScene(r._root, lightToObjWorldSpace);

        if (lightToObjWorldSpace.intersection.point == ray.intersection.point) {
            pointLight.setPosition(pol);
            pointLight.shade(ray);
        }

    }

}
#ifndef CAMERA_H
#define CAMERA_H

#include <ode/ode.h>

/*
  Simple implementatino of a camera that orbits the origin
 */


struct Camera {

    float fov, aspect,
        znear, zfar;

    float x, y, z; // eye pos
    float tx, ty, tz; // target
    dVector3 up;
    dVector3 right;

    int width, height, window_x, window_y;


    Camera();
    void transform_proj() const;
    void transform_model() const;

    void reshape(int w, int h);

    void press(int x, int y);
    void arcball(int x, int y);

    double dist() const;

    void zoom(float d); // >1 to zoom out, <1 to zoom in

private:
    float old_x, old_y;

    void reset();
    float normx(int x) const;
    float normy(int y) const;

    float to_sphere_z(float r, float x, float y);
};

#endif

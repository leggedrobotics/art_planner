#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

#include <GL/gl.h>
#include <GL/glu.h>

#include "camera.hpp"


using std::clog;
using std::endl;

Camera::Camera() :
    fov(50),
    aspect(1),
    znear(.01),
    zfar(50),
    width(0),
    height(0),
    window_x(0),
    window_y(0)
{
    reset();
}


void
Camera::transform_proj() const
{
    glLoadIdentity();
    gluPerspective(fov, aspect, znear, zfar);
}

void
Camera::transform_model() const
{
    glLoadIdentity();
    gluLookAt(x, y, z, tx, ty, tz, up[0], up[1], up[2]);
}



void
Camera::reset()
{		
    x = 0;
    y = 0;
    z = 0;
    tx = 0;
    ty = 0;
    tz = 0;
    up[0] = 0;
    up[1] = 1;
    up[2] = 0;
    right[0] = 1;
    right[1] = 0;
    right[2] = 0;
}



void
Camera::reshape(int w, int h)
{
    aspect = float(w)/h;
    width = w;
    height = h;
}


float
Camera::normx(int x) const
{
    return (2.0*(x-window_x)) / width - 1.0;
}

float
Camera::normy(int y) const
{
    return -((2.0*(y-window_y)) / height - 1.0);
}


void
Camera::press(int x, int y)
{
    old_x = normx(x);
    old_y = normy(y);
}



float
Camera::to_sphere_z(float r, float x, float y)
{
    float delta = x*x + y*y;
    if (delta > r*r)
        return 0;
    else
        return std::sqrt(r*r - delta);
}


void showm(dMatrix3 m)
{
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j)
            clog << " " << std::setw(12) << m[i*4 + j];
        clog << endl;
    }
    clog << endl;
}

void showv(dVector3 v)
{
    for (int i=0; i<3; ++i)
        clog << " " << std::setw(12) << v[i];
    clog << endl << endl;
}



void
Camera::arcball(int mx, int my)
{
    const float radius = 2;
    
    float new_x, new_y;
    new_x = normx(mx);
    new_y = normy(my);

    float old_z, new_z;
    old_z = to_sphere_z(radius, old_x, old_y);
    new_z = to_sphere_z(radius, new_x, new_y);

    dVector3 m = { old_x, old_y, old_z };
    dVector3 n = { new_x, new_y, new_z };

    //std::clog << "old: " << old_x << " , " << old_y << std::endl;
    //std::clog << "new: " << new_x << " , " << new_y << std::endl;
    old_x = new_x;
    old_y = new_y;


    dNormalize3(m);
    dNormalize3(n);

    // axis of rotation r is along m cross n
    dVector3 r;
    dCROSS(r, =, m, n);
    float s = dLENGTH(r);
    s = std::min(1.f, std::max(-1.f, s));
    float angle = std::asin(s);

    // build the local matrix
    dVector3 f = {x-tx, y-ty, z-tz};
    dNormalize3(f);
    //showv(f);
    dCROSS(right, =, up, f);
    dNormalize3(right);
    dCROSS(up, =, f, right);
    dNormalize3(up);
    dMatrix3 localR = {
        right[0], right[1], right[2], 0,
        up[0], up[1], up[2], 0,
        f[0],  f[1],  f[2]
    };
    //showm(localR);

    // bring the axis to global coordinates
    dVector3 global_r;
    dMultiply1_331(global_r, localR, r);
    dMatrix3 R;
    dRFromAxisAndAngle(R, global_r[0], global_r[1], global_r[2], -2*angle);

    // we re-position the camera, maintaining the distance
    dVector3 p = { x, y, z };
    dVector3 t = { tx, ty, tz };
    dVector3 d;
    dSubtractVectors3(d, p, t); // d = p - t
    dVector3 nd;
    dMultiply0_331(nd, R, d);
    x = nd[0] + tx;
    y = nd[1] + ty;
    z = nd[2] + tz;    

    dVector3 nup;
    dMultiply0_331(nup, R, up);
    dCopyVector3(up, nup);
    dNormalize3(up);

    dVector3 nright;
    dMultiply0_331(nright, R, right);
    dCopyVector3(right, nright);
    dNormalize3(right);
}


double
Camera::dist() const
{
    double dx = tx - x;
    double dy = ty - y;
    double dz = tz - z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void
Camera::zoom(float d)
{
    double dx = x - tx;
    double dy = y - ty;
    double dz = z - tz;
    dx *= d;
    dy *= d;
    dz *= d;
    x = tx + dx;
    y = ty + dy;
    z = tz + dz;
}

#include <cstdlib>
#include <iostream>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include <ode/ode.h>

#include <AntTweakBar.h>

#if USE_GLFW
#include <GL/glfw.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>


#include "camera.hpp"


using std::cout;
using std::cerr;
using std::clog;
using std::endl;



TwBar *left_bar = 0;
TwBar *right_bar = 0;


float left_x=-1, left_y=0, left_z=0;
float left_quat[4] = {0, 0, 0, 1}; // x y z w

float right_x=1, right_y=0, right_z=0;
float right_quat[4] = {0, 0, 0, 1}; // x y z w

int left_geom_type = 0;
int right_geom_type = 0;

// left
float left_sphere_radius = 1;
float left_box_width = 1;
float left_box_height = 1;
float left_box_depth = 1;
float left_capsule_radius = 1;
float left_capsule_length = 1;

float right_sphere_radius = 1;
float right_box_width = 1;
float right_box_height = 1;
float right_box_depth = 1;
float right_capsule_radius = 1;
float right_capsule_length = 1;


struct Color {
    float r, g, b, a;
    Color() :
        r(0), g(0), b(0), a(0)
    {}

    Color(float x, float y, float z, float w=1.0) :
        r(x), g(y), b(z), a(w)
    {}
};

Color left_fill,
    left_wire,
    right_fill,
    right_wire,
    contact_p,
    contact_d,
    contact_n;


dGeomID left_geom=0, right_geom=0;

Camera cam;

enum GeomType { Sphere, Box, Capsule };



void draw();


void left_update_position();
void right_update_position();


#if USE_GLFW
void GLFWCALL mouse_moved(int x, int y);
void GLFWCALL mouse_button(int button, int action);
void GLFWCALL mouse_wheel(int pos);

void GLFWCALL resize(int width, int height)
{
    // Set OpenGL viewport and camera
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40, (double)width/height, 1, 10);
    gluLookAt(-1,0,3, 0,0,0, 0,1,0);    


    cam.reshape(width, height);
    
    // Send the new window size to AntTweakBar
    TwWindowSize(width, height);
}

void init_gl()
{
    if (!glfwInit()) {
        cerr << "Failed to initialize GLFW" << endl;
        exit(EXIT_FAILURE);
    }
    std::atexit(glfwTerminate);

    if (!glfwOpenWindow(1024, 768,
                        0,0,0,0,
                        16, // depth
                        0, // stencil
                        GLFW_WINDOW)) {
        exit(EXIT_FAILURE);
    }
    
    glfwEnable(GLFW_MOUSE_CURSOR);
    glfwEnable(GLFW_KEY_REPEAT);
    glfwSetWindowTitle("ODE collision test environment");

    // Initialize AntTweakBar
    TwInit(TW_OPENGL, NULL);

    // Set GLFW event callbacks
    // - Redirect window size changes to the callback function WindowSizeCB
    glfwSetWindowSizeCallback(resize);

    glfwSetMouseButtonCallback(mouse_button);

    glfwSetMousePosCallback(mouse_moved);

    glfwSetMouseWheelCallback((GLFWmousewheelfun)mouse_wheel);
    // - Directly redirect GLFW key events to AntTweakBar
    glfwSetKeyCallback((GLFWkeyfun)TwEventKeyGLFW);
    // - Directly redirect GLFW char events to AntTweakBar
    glfwSetCharCallback((GLFWcharfun)TwEventCharGLFW);


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void loop()
{
    bool running = true;
    
    while (running) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        draw();
        
        TwDraw();
        
        glfwSwapBuffers();

        running = glfwGetWindowParam(GLFW_OPENED) && !glfwGetKey(GLFW_KEY_ESC);
    }
}

static bool left_dragging = false;
static bool right_dragging = false;
static bool left_dragging_geom = false;
static bool right_dragging_geom = false;

static int prev_x = 0;
static int prev_y = 0;

void GLFWCALL mouse_moved(int x, int y)
{
    int dx = x - prev_x;
    int dy = y - prev_y;

    // don't let Tw eat the events if we are doing something
    if (!right_dragging && !left_dragging && !left_dragging_geom && !right_dragging_geom)
        TwEventMousePosGLFW(x, y);
    else {
        if (right_dragging) {
            cam.arcball(x, y);
        }
        if (left_dragging) {
            float s = 0.001*cam.dist();
            cam.tx += s*(-dx * cam.right[0] + dy*cam.up[0]);
            cam.ty += s*(-dx * cam.right[1] + dy*cam.up[1]);
            cam.tz += s*(-dx * cam.right[2] + dy*cam.up[2]);
        }
        if (left_dragging_geom) {
            float s = 0.001*cam.dist();
            left_x += s*(dx * cam.right[0] - dy*cam.up[0]);
            left_y += s*(dx * cam.right[1] - dy*cam.up[1]);
            left_z += s*(dx * cam.right[2] - dy*cam.up[2]);
            left_update_position();
        }
        if (right_dragging_geom) {
            float s = 0.001*cam.dist();
            right_x += s*(dx * cam.right[0] - dy*cam.up[0]);
            right_y += s*(dx * cam.right[1] - dy*cam.up[1]);
            right_z += s*(dx * cam.right[2] - dy*cam.up[2]);
            right_update_position();
        }
    }
    prev_x = x;
    prev_y = y;
}

void GLFWCALL mouse_button(int button, int action)
{
    int x, y;
    glfwGetMousePos(&x, &y);

    int handled = TwEventMouseButtonGLFW(button, action);
    if (!handled) {
        if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
            cam.press(x, y);
            right_dragging = true;
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
            right_dragging = false;
        }

        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            
            GLdouble model[16];
            GLdouble proj[16];
            GLint viewport[4];
            glGetDoublev(GL_MODELVIEW_MATRIX, model);
            glGetDoublev(GL_PROJECTION_MATRIX, proj);
            glGetIntegerv(GL_VIEWPORT, viewport);
            GLdouble ox, oy, oz;

            if (gluUnProject(x, viewport[3] - y, 0, model, proj, viewport, &ox, &oy, &oz) == GLU_TRUE) {
                double dx, dy, dz;
                dx = ox - cam.x;
                dy = oy - cam.y;
                dz = oz - cam.z;

                dGeomID ray = dCreateRay(0, 100);
                dGeomRaySet(ray, cam.x, cam.y, cam.z, dx, dy, dz);
                dContactGeom left_contact, right_contact;
                int leftn, rightn;
                leftn = dCollide(ray, left_geom, 1, &left_contact, sizeof left_contact);
                rightn = dCollide(ray, right_geom, 1, &right_contact, sizeof right_contact);
                if (leftn && !rightn)
                    left_dragging_geom = true;
                if (rightn && !leftn)
                    right_dragging_geom = true;
                if (leftn && rightn) {
                    if (left_contact.depth < right_contact.depth)
                        left_dragging_geom = true;
                    else 
                        right_dragging_geom = true;
                }
                
                dGeomDestroy(ray);

            }
            if (!left_dragging_geom && !right_dragging_geom)
                left_dragging = true;
        }
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
            left_dragging = false;
            left_dragging_geom = false;
            right_dragging_geom = false;
        }
    }
}

static int prev_wheel = 0;
void GLFWCALL mouse_wheel(int pos)
{
    if (!TwEventMouseWheelGLFW(pos)) {
        int delta = pos - prev_wheel;
        
        // set the camera zoom
        if (delta > 0)
            cam.zoom(1.1);
        if (delta < 0)
            cam.zoom(1/1.1);
    }
    prev_wheel = pos;
}



#else
void init_gl()
{}

void loop()
{}
#endif




// generic getters
void TW_CALL get_float(void *value, void *data)
{
    float *v = reinterpret_cast<float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *v = *d;
}

void TW_CALL get_int(void *value, void *data)
{
    int *v = reinterpret_cast<int*>(value);
    int *d = reinterpret_cast<int*>(data);
    *v = *d;
}


void enable_sphere(TwBar *bar)
{
    TwSetParam(bar, "SphereRadius", "visible", TW_PARAM_CSTRING, 1, "true");
}

void disable_sphere(TwBar *bar)
{
    TwSetParam(bar, "SphereRadius", "visible", TW_PARAM_CSTRING, 1, "false");
}

void enable_box(TwBar *bar)
{
    TwSetParam(bar, "BoxWidth", "visible", TW_PARAM_CSTRING, 1, "true");
    TwSetParam(bar, "BoxHeight", "visible", TW_PARAM_CSTRING, 1, "true");
    TwSetParam(bar, "BoxDepth", "visible", TW_PARAM_CSTRING, 1, "true");
}

void disable_box(TwBar *bar)
{
    TwSetParam(bar, "BoxWidth", "visible", TW_PARAM_CSTRING, 1, "false");
    TwSetParam(bar, "BoxHeight", "visible", TW_PARAM_CSTRING, 1, "false");
    TwSetParam(bar, "BoxDepth", "visible", TW_PARAM_CSTRING, 1, "false");
}

void enable_capsule(TwBar *bar)
{
    TwSetParam(bar, "CapsuleRadius", "visible", TW_PARAM_CSTRING, 1, "true");
    TwSetParam(bar, "CapsuleLength", "visible", TW_PARAM_CSTRING, 1, "true");
}

void disable_capsule(TwBar *bar)
{
    TwSetParam(bar, "CapsuleRadius", "visible", TW_PARAM_CSTRING, 1, "false");
    TwSetParam(bar, "CapsuleLength", "visible", TW_PARAM_CSTRING, 1, "false");
}


void left_update_position()
{
    if (left_geom)
        dGeomSetPosition(left_geom, left_x, left_y, left_z);
}

void right_update_position()
{
    if (right_geom)
        dGeomSetPosition(right_geom, right_x, right_y, right_z);
}

void left_update_quat()
{
    if (left_geom) {
        dQuaternion q = {left_quat[3], left_quat[0], left_quat[1], left_quat[2]};
        dGeomSetQuaternion(left_geom, q);
    }
}

void right_update_quat()
{
    if (right_geom) {
        dQuaternion q = {right_quat[3], right_quat[0], right_quat[1], right_quat[2]};
        dGeomSetQuaternion(right_geom, q);
    }
}

void choose_left_geom(int type);
void choose_right_geom(int type);


void TW_CALL left_set_geom_type(const void *value, void *data)
{
    const int *v = reinterpret_cast<const int*>(value);
    int *d = reinterpret_cast<int*>(data);
    *d = *v;

    choose_left_geom(*d);
}

void TW_CALL right_set_geom_type(const void *value, void *data)
{
    const int *v = reinterpret_cast<const int*>(value);
    int *d = reinterpret_cast<int*>(data);
    *d = *v;

    choose_right_geom(*d);
}


void TW_CALL left_set_val(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    left_update_position();
}

void TW_CALL right_set_val(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    right_update_position();
}

void TW_CALL left_set_quat(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d++ = *v++;
    *d++ = *v++;
    *d++ = *v++;
    *d++ = *v++;

    left_update_quat();
}

void TW_CALL right_set_quat(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d++ = *v++;
    *d++ = *v++;
    *d++ = *v++;
    *d++ = *v++;

    right_update_quat();
}

void TW_CALL get_quat(void *value, void *data)
{
    float *v = reinterpret_cast<float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *v++ = *d++;
    *v++ = *d++;
    *v++ = *d++;
    *v++ = *d++;
}





void TW_CALL left_sphere_set_radius(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomSphereSetRadius(left_geom, left_sphere_radius);
}

void TW_CALL right_sphere_set_radius(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomSphereSetRadius(right_geom, right_sphere_radius);
}



void TW_CALL left_box_set_size(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomBoxSetLengths(left_geom, left_box_width, left_box_height, left_box_depth);
}

void TW_CALL right_box_set_size(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomBoxSetLengths(right_geom, right_box_width, right_box_height, right_box_depth);
}


void TW_CALL left_capsule_set_radius(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomCapsuleSetParams(left_geom, left_capsule_radius, left_capsule_length);
}

void TW_CALL left_capsule_set_length(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomCapsuleSetParams(left_geom, left_capsule_radius, left_capsule_length);
}

void TW_CALL right_capsule_set_radius(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomCapsuleSetParams(right_geom, right_capsule_radius, right_capsule_length);
}

void TW_CALL right_capsule_set_length(const void *value, void *data)
{
    const float *v = reinterpret_cast<const float*>(value);
    float *d = reinterpret_cast<float*>(data);
    *d = *v;

    dGeomCapsuleSetParams(right_geom, right_capsule_radius, right_capsule_length);
}




void choose_left_geom(int g)
{
    if (left_geom) {
        dGeomDestroy(left_geom);
        left_geom = 0;
    }

    // TODO: update ODE geom
    if (g == Sphere) {
        enable_sphere(left_bar);
        left_geom = dCreateSphere(0, left_sphere_radius);
    } else
        disable_sphere(left_bar);

    if (g == Box) {
        enable_box(left_bar);
        left_geom = dCreateBox(0, left_box_width, left_box_height, left_box_depth);
    } else
        disable_box(left_bar);

    if (g == Capsule) {
        enable_capsule(left_bar);
        left_geom = dCreateCapsule(0, left_capsule_radius, left_capsule_length);
    } else
        disable_capsule(left_bar);


    left_update_position();
    left_update_quat();
}

void choose_right_geom(int g)
{
    if (right_geom) {
        dGeomDestroy(right_geom);
        right_geom = 0;
    }
    
    // TODO: update ODE geom
    if (g == Sphere) {
        enable_sphere(right_bar);
        right_geom = dCreateSphere(0, right_sphere_radius);
    } else
        disable_sphere(right_bar);

    if (g == Box) {
        enable_box(right_bar);
        right_geom = dCreateBox(0, right_box_width, right_box_height, right_box_depth);
    } else
        disable_box(right_bar);

    if (g == Capsule) {
        enable_capsule(right_bar);
        right_geom = dCreateCapsule(0, right_capsule_radius, right_capsule_length);
    } else
        disable_capsule(right_bar);


    right_update_position();
    right_update_quat();
}



void TW_CALL reset_camera(void*)
{
    cam.x = 0;
    cam.y = 0;
    cam.z = 8;
    cam.tx = 0;
    cam.ty = 0;
    cam.tz = 0;

    cam.up[0] = 0;
    cam.up[1] = 1;
    cam.up[2] = 0;

    cam.right[0] = 1;
    cam.right[1] = 0;
    cam.right[2] = 0;
}

void TW_CALL reset_geoms(void*)
{
    choose_left_geom(Sphere);
    choose_right_geom(Sphere);
    left_x = -1;
    left_y = 0;
    left_z = 0;
    right_x = 1;
    right_y = 0;
    right_z = 0;
    left_quat[0] = 1;
    left_quat[1] = 0;
    left_quat[2] = 0;
    left_quat[3] = 0;
    right_quat[0] = 1;
    right_quat[1] = 0;
    right_quat[2] = 0;
    right_quat[3] = 0;

    left_update_position();
    left_update_quat();
    right_update_position();
    right_update_quat();
}





void init_ui()
{
    TwBar *bar = TwNewBar("Controls");

    TwAddButton(bar, "ResetCamera", reset_camera, 0, "label='Reset camera'");
    TwAddButton(bar, "ResetGeoms", reset_geoms, 0, "label='Reset geoms'");

    TwType gtype = TwDefineEnumFromString("GeomType", "Sphere,Box,Capsule");

    left_bar = TwNewBar("Left");

    TwAddVarCB(left_bar, "Geom", gtype, left_set_geom_type, get_int, &left_geom_type, NULL);

    TwSetParam(left_bar, NULL, "position", TW_PARAM_CSTRING, 1, "50 50");

    TwAddVarCB(left_bar, "X", TW_TYPE_FLOAT, left_set_val, get_float, &left_x, "min=-100 max=100 step=0.01");
    TwAddVarCB(left_bar, "Y", TW_TYPE_FLOAT, left_set_val, get_float, &left_y, "min=-100 max=100 step=0.01");
    TwAddVarCB(left_bar, "Z", TW_TYPE_FLOAT, left_set_val, get_float, &left_z, "min=-100 max=100 step=0.01");
    TwAddVarCB(left_bar, "Quat", TW_TYPE_QUAT4F, left_set_quat, get_quat, left_quat, "showval=true opened=true");

    TwAddVarCB(left_bar, "SphereRadius", TW_TYPE_FLOAT, left_sphere_set_radius, get_float, &left_sphere_radius, " min=0.001 max=100 step=0.01 visible=false");

    TwAddVarCB(left_bar, "BoxWidth", TW_TYPE_FLOAT, left_box_set_size, get_float, &left_box_width, " min=0.001 max=100 step=0.01 visible=false");
    TwAddVarCB(left_bar, "BoxHeight", TW_TYPE_FLOAT, left_box_set_size, get_float, &left_box_height, " min=0.001 max=100 step=0.01 visible=false");
    TwAddVarCB(left_bar, "BoxDepth", TW_TYPE_FLOAT, left_box_set_size, get_float, &left_box_depth, " min=0.001 max=100 step=0.01 visible=false");

    TwAddVarCB(left_bar, "CapsuleRadius", TW_TYPE_FLOAT, left_capsule_set_radius, get_float, &left_capsule_radius, " min=0.001 max=100 step=0.01 visible=false");
    TwAddVarCB(left_bar, "CapsuleLength", TW_TYPE_FLOAT, left_capsule_set_length, get_float, &left_capsule_length, " min=0.001 max=100 step=0.01 visible=false");

#if 1
    right_bar = TwNewBar("Right");

    TwAddVarCB(right_bar, "Geom", gtype, right_set_geom_type, get_int, &right_geom_type, NULL);

    TwSetParam(right_bar, NULL, "position", TW_PARAM_CSTRING, 1, "650 50");

    TwAddVarCB(right_bar, "X", TW_TYPE_FLOAT, right_set_val, get_float, &right_x, "min=-100 max=100 step=0.01");
    TwAddVarCB(right_bar, "Y", TW_TYPE_FLOAT, right_set_val, get_float, &right_y, "min=-100 max=100 step=0.01");
    TwAddVarCB(right_bar, "Z", TW_TYPE_FLOAT, right_set_val, get_float, &right_z, "min=-100 max=100 step=0.01");
    TwAddVarCB(right_bar, "Quat", TW_TYPE_QUAT4F, right_set_quat, get_quat, right_quat, "showval=true opened=true");


    TwAddVarCB(right_bar, "SphereRadius", TW_TYPE_FLOAT, right_sphere_set_radius, get_float, &right_sphere_radius, " min=0.001 max=100 step=0.01 visible=false");

    TwAddVarCB(right_bar, "BoxWidth", TW_TYPE_FLOAT, right_box_set_size, get_float, &right_box_width, " min=0.001 max=100 step=0.01 visible=false");
    TwAddVarCB(right_bar, "BoxHeight", TW_TYPE_FLOAT, right_box_set_size, get_float, &right_box_height, " min=0.001 max=100 step=0.01 visible=false");
    TwAddVarCB(right_bar, "BoxDepth", TW_TYPE_FLOAT, right_box_set_size, get_float, &right_box_depth, " min=0.001 max=100 step=0.01 visible=false");

    TwAddVarCB(right_bar, "CapsuleRadius", TW_TYPE_FLOAT, right_capsule_set_radius, get_float, &right_capsule_radius, " min=0.001 max=100 step=0.01 visible=false");
    TwAddVarCB(right_bar, "CapsuleLength", TW_TYPE_FLOAT, right_capsule_set_length, get_float, &right_capsule_length, " min=0.001 max=100 step=0.01 visible=false");
#endif
}


void draw_sphere(float radius, Color fill, Color wire)
{
    const int stacks = std::max(16, int(radius * 16));
    const int slices = std::max(32, int(radius * 32));

    GLUquadric* q = gluNewQuadric();

    gluQuadricDrawStyle(q, GLU_FILL);

    glColor4f(fill.r, fill.g, fill.b, fill.a);
    gluSphere(q, radius, slices, stacks);

    gluQuadricDrawStyle(q, GLU_LINE);

    glColor4f(wire.r, wire.g, wire.b, wire.a);
    gluSphere(q, radius, slices, stacks);

    gluDeleteQuadric(q);
}


void draw_box_face(float w, float h, float d,
                   Color fill, Color wire)
{
    const int partsw = std::max(2, int(w*4));
    const int partsh = std::max(2, int(h*4));

    glLineWidth(1);

    glBegin(GL_QUADS);
    glColor4f(fill.r, fill.g, fill.b, fill.a);
    glVertex3f(-w/2, -h/2, d/2);
    glVertex3f( w/2, -h/2, d/2);
    glVertex3f( w/2,  h/2, d/2);
    glVertex3f(-w/2,  h/2, d/2);
    glEnd();

    glBegin(GL_LINES);
    glColor4f(wire.r, wire.g, wire.b, wire.a);
    for (int i=0; i<=partsw; ++i) {        // vertical lines
        glVertex3f(-w/2 + i*w/partsw, -h/2, d/2);
        glVertex3f(-w/2 + i*w/partsw,  h/2, d/2);
    }
    for (int i=0; i<=partsh; ++i) {        // horizontal lines
        glVertex3f(-w/2, -h/2+i*h/partsh, d/2);
        glVertex3f( w/2, -h/2+i*h/partsh, d/2);
    }
    glEnd();
}

void draw_box(float w, float h, float d, Color fill, Color wire)
{
    draw_box_face(w, h, d, fill, wire);

    glPushMatrix();
    glRotatef(90, 0, 1, 0);
    draw_box_face(d, h, w, fill, wire);
    glPopMatrix();

    glPushMatrix();
    glRotatef(180, 0, 1, 0);
    draw_box_face(w, h, d, fill, wire);
    glPopMatrix();

    glPushMatrix();
    glRotatef(270, 0, 1, 0);
    draw_box_face(d, h, w, fill, wire);
    glPopMatrix();

    glPushMatrix();
    glRotatef(90, 1, 0, 0);
    draw_box_face(w, d, h, fill, wire);
    glPopMatrix();

    glPushMatrix();
    glRotatef(270, 1, 0, 0);
    draw_box_face(w, d, h, fill, wire);
    glPopMatrix();
}


void draw_cylinder(float radius, float length, Color fill, Color wire)
{
    GLUquadric *q = gluNewQuadric();

    const int stacks = std::max(2, int(length*4));
    const int slices = std::max(32, int(radius * 32));

    glPushMatrix();
    glTranslatef(0, 0, -length/2);
    
    gluQuadricDrawStyle(q, GLU_FILL);
    glColor4f(fill.r, fill.g, fill.b, fill.a);
    gluCylinder(q, radius, radius, length, slices, stacks);

    gluQuadricDrawStyle(q, GLU_LINE);
    glColor4f(wire.r, wire.g, wire.b, wire.a);
    gluCylinder(q, radius, radius, length, slices, stacks);

    glPopMatrix();

    gluDeleteQuadric(q);
}




void draw_capsule(float radius, float length, Color fill, Color wire)
{
    draw_cylinder(radius, length, fill, wire);

    glEnable(GL_CLIP_PLANE0);

    glPushMatrix();
    glTranslatef(0, 0, length/2);

    GLdouble plane[4] = {0, 0, 1, 0};
    glClipPlane(GL_CLIP_PLANE0, plane);
    draw_sphere(radius, fill, wire);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0, 0, -length/2);
    plane[2] = -1;
    glClipPlane(GL_CLIP_PLANE0, plane);
    draw_sphere(radius, fill, wire);
    glPopMatrix();

    glDisable(GL_CLIP_PLANE0);
}




void draw_geom(dGeomID geom, Color fill, Color wire)
{
    if (!geom)
        return;

    glPushMatrix();
    const dReal *p = dGeomGetPosition(geom);
    const dReal *r = dGeomGetRotation(geom);
    GLfloat m[16] = {
        r[0], r[4], r[8], 0,
        r[1], r[5], r[9], 0,
        r[2], r[6], r[10], 0,
        p[0], p[1], p[2], 1
    };
    glMultMatrixf(m);
    
    glLineWidth(1);

    int c = dGeomGetClass(geom);
    switch (c) {
        case dSphereClass:
        {
            float r = dGeomSphereGetRadius(geom);
            draw_sphere(r, fill, wire);
            break;
        }
        case dBoxClass: 
        {
            dVector3 lengths;
            dGeomBoxGetLengths(geom, lengths);
            draw_box(lengths[0], lengths[1], lengths[2], fill, wire);
            break;
        }
        case dCapsuleClass:
        {
            dReal radius, length;
            dGeomCapsuleGetParams(geom, &radius, &length);
            draw_capsule(radius, length, fill, wire);
            break;
        }
    }

    glPopMatrix();
}


void draw_contact(const dContactGeom& c, Color p, Color d, Color n)
{
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor4f(p.r, p.g, p.b, p.a);
    glVertex3f(c.pos[0], c.pos[1], c.pos[2]);
    glEnd();

    const float scale = c.depth;
    
    glLineWidth(3);
    glBegin(GL_LINES);
    glColor4f(d.r, d.g, d.b, d.a);
    glVertex3f(c.pos[0], c.pos[1], c.pos[2]);
    glVertex3f(c.pos[0]+c.normal[0]*scale, c.pos[1]+c.normal[1]*scale, c.pos[2]+c.normal[2]*scale);
    glEnd();

    const float cscale = cam.dist() * 0.125;

    glLineWidth(1);
    glBegin(GL_LINES);
    glColor4f(n.r, n.g, n.b, n.a);
    glVertex3f(c.pos[0], c.pos[1], c.pos[2]);
    glVertex3f(c.pos[0]+c.normal[0]*cscale, c.pos[1]+c.normal[1]*cscale, c.pos[2]+c.normal[2]*cscale);
    glEnd();
}


void draw_contacts()
{
    if (!left_geom || !right_geom)
        return;
    
    const int max_contacts = 32;
    dContactGeom dg[max_contacts];
    
    int n = dCollide(left_geom, right_geom, max_contacts, &dg[0], sizeof(dContactGeom));
    for (int i=0; i<n; ++i)
        draw_contact(dg[i], contact_p, contact_d, contact_n);
}




void
draw()
{
    glMatrixMode(GL_PROJECTION);
    cam.transform_proj();
    
    glMatrixMode(GL_MODELVIEW);
    cam.transform_model();

    draw_geom(left_geom, left_fill, left_wire);
    draw_geom(right_geom, right_fill, right_wire);

    draw_contacts();
}



int main(int argc, char **argv)
{
    init_gl();

    init_ui();

    reset_camera(0);
    reset_geoms(0);

    left_fill = Color(1, 0, 0, 0.125);
    left_wire = Color(1, 0, 0, 0.25);
    right_fill = Color(0, 1, 0, 0.125);
    right_wire = Color(0, 1, 0, 0.25);
    contact_p = Color(1, 1, 0, 1);
    contact_d = Color(.5, .5, 0, 1);
    contact_n = Color(0, 0, 1, 1);

    dInitODE();

    loop();

    dCloseODE();

    TwTerminate();
}


/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// Convex demo.
// Serves as a test for the convex geometry.
// By Bram Stolk.

#include <assert.h>
#ifdef HAVE_UNISTD_H
#	include <unistd.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#include "halton235_geom.h"

#ifdef dDOUBLE
#	define dsDrawConvex dsDrawConvexD
#	define dsDrawLine dsDrawLineD
#endif


#ifdef _MSC_VER
#	pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


// Height at which we drop the composite block.
const dReal H=4.20;

static dWorldID world;
static dSpaceID space;

static dBodyID mbody;

static dBodyID hbody[ halton_numc ];
static dGeomID hgeom[ halton_numc ];

static dJointGroupID contactgroup;

static bool drawpos=false;
static bool solidkernel=false;


// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	assert(o1);
	assert(o2);
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
	{
		// colliding a space with something
		dSpaceCollide2(o1,o2,data,&nearCallback);
		// Note we do not want to test intersections within a space,
		// only between spaces.
		return;
	}

	const int N = 32;
	dContact contact[N];
	int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
	if (n > 0) 
	{
		for (int i=0; i<n; i++) 
		{
			contact[i].surface.slip1 = 0.7;
			contact[i].surface.slip2 = 0.7;
			contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
			contact[i].surface.mu = 500.0; // was: dInfinity
			contact[i].surface.soft_erp = 0.50;
			contact[i].surface.soft_cfm = 0.03;
			dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
			dJointAttach
			(
				c,
				dGeomGetBody(contact[i].geom.g1),
				dGeomGetBody(contact[i].geom.g2)
			);
		}
	}
}


// start simulation - set viewpoint

static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);
	static float xyz[3] = {-8,0,5};
	static float hpr[3] = {0.0f,-29.5000f,0.0000f};
	dsSetViewpoint (xyz,hpr);
	fprintf(stderr,"Press SPACE to reset the simulation.\n");
}


static void reset()
{
	dQuaternion q;
	dQSetIdentity(q);
	dBodySetPosition(mbody,0,0,0+H);
	dBodySetQuaternion(mbody, q);
	dBodySetLinearVel(mbody, 0,0,0);
	dBodySetAngularVel(mbody, 0,0,0);
	dBodyEnable(mbody);
	for ( int i=0; i<halton_numc; ++i )
	{
		dBodyID body = hbody[i];
		if ( !body ) continue;
		dBodySetPosition(body, halton_pos[i][0], halton_pos[i][1], halton_pos[i][2]+H);
		dBodySetQuaternion(body, q);
		dBodySetLinearVel(body, 0,0,0);
		dBodySetAngularVel(body, 0,0,0);
		dBodyEnable(body);
	}
}


// called when a key pressed

static void command(int cmd)
{
	switch (cmd) 
	{
		case ' ':
			reset();
			break;
		default:
			break;
	}
}



static void simLoop(int pause)
{
	double simstep = 1/240.0;
	double dt = dsElapsedTime();

	int nrofsteps = (int) ceilf(dt/simstep);
	nrofsteps = nrofsteps > 8 ? 8 : nrofsteps;

	for (int i=0; i<nrofsteps && !pause; i++)
	{
		dSpaceCollide (space,0,&nearCallback);
		dWorldQuickStep (world, simstep);
		dJointGroupEmpty (contactgroup);
	}

	dsSetColor (1,1,1);
	// Draw the convex objects.
	for ( int i=0; i<halton_numc; ++i )
	{
		dGeomID geom = hgeom[i];
		dBodyID body = dGeomGetBody(geom);
		//const dReal *pos = dBodyGetPosition(body);
		//const dReal *rot = dBodyGetRotation(body);
		const dReal *pos = dGeomGetPosition(geom);
		const dReal *rot = dGeomGetRotation(geom);
		dsDrawConvex
		(
			pos, rot,
			halton_planes[i],
			halton_numf[i],
			halton_verts[i],
			halton_numv[i],
			halton_faces[i]
		);
	}

	if (drawpos)
	{
		dsSetColor(1,0,0.2);
		dsSetTexture(DS_NONE);
		const dReal l = 0.35;
		for ( int i=0; i<halton_numc; ++i )
		{
			dBodyID body = hbody[i];
			const dReal *pos = dBodyGetPosition(body);
			dReal x0[3] = { pos[0]-l, pos[1], pos[2] };
			dReal x1[3] = { pos[0]+l, pos[1], pos[2] };
			dReal y0[3] = { pos[0], pos[1]-l, pos[2] };
			dReal y1[3] = { pos[0], pos[1]+l, pos[2] };
			dReal z0[3] = { pos[0], pos[1], pos[2]-l };
			dReal z1[3] = { pos[0], pos[1], pos[2]+l };
			dsDrawLine(x0,x1);
			dsDrawLine(y0,y1);
			dsDrawLine(z0,z1);
		}
	}
}


int main (int argc, char **argv)
{
	dMass m;

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	// create world
	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	dHashSpaceSetLevels(space, -3, 5);
	dCreatePlane(space,0,0,1,0);	// Add a ground plane.

	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity(world,0,0,-9.8);
	dWorldSetQuickStepNumIterations(world, 32);
	dWorldSetContactMaxCorrectingVel(world, 40);
	dWorldSetMaxAngularSpeed(world, 62.8);
	dWorldSetERP(world, 0.7);
	dWorldSetQuickStepW(world, 0.75); // For increased stability.

	dWorldSetAutoDisableFlag( world, true );
	dWorldSetAutoDisableLinearThreshold( world, 0.01 );
	dWorldSetAutoDisableAngularThreshold( world, 0.03 );
	dWorldSetAutoDisableTime( world, 0.15f );

	const float kernelrad = 0.7;

	mbody = dBodyCreate(world);
	dBodySetPosition(mbody, 0,0,0+H);
	dMassSetSphere( &m, 5, kernelrad );
	dBodySetMass( mbody, &m );

	for (int i=0; i<halton_numc; ++i )
	{
		dGeomID geom = dCreateConvex
		(
			space,
			halton_planes[i],
			halton_numf[i],
			halton_verts[i],
			halton_numv[i],
			halton_faces[i]
		);
		hgeom[i] = geom;
		const dReal x = halton_pos[i][0];
		const dReal y = halton_pos[i][1];
		const dReal z = halton_pos[i][2];
		const dReal dsqr = x*x + y*y + z*z;

		if ( dsqr < kernelrad*kernelrad && solidkernel )
		{
			dGeomSetBody(geom, mbody);
			dGeomSetOffsetPosition(geom, x,y,z);
		}
		else
		{
			dBodyID body = dBodyCreate(world);
			hbody[i] = body;
			dBodySetPosition(body, x,y,z+H);
			dReal volu = halton_volu[i];
			dReal rad = pow( volu * 3 / (4*M_PI), (1/3.0) );
			dMassSetSphere( &m,5,rad );
			dBodySetMass( body,&m );
#if 1
			dBodySetLinearDamping (body, 0.0005);
			dBodySetAngularDamping(body, 0.0300);
#endif
			dGeomSetBody(geom,body);
		}
	}

	// run simulation
	const int w=1280;
	const int h=720;
	dsSimulationLoop (argc,argv,w,h,&fn);

	dJointGroupEmpty (contactgroup);
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
	return 0;
}



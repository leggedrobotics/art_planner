#include <chrono>
#include <iostream>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>



void setPolygonValue(const grid_map::Polygon& poly,
                     std::shared_ptr<grid_map::GridMap> map,
                     float trav_val, float elev_val) {
  auto iter = grid_map::PolygonIterator(*map, poly);
  for (; !iter.isPastEnd(); ++iter) {
    map->at("traversability", *iter) = trav_val;
    map->at("elevation", *iter) = elev_val;
  }
}



std::shared_ptr<grid_map::GridMap> getTestMap() {
  auto map = std::make_shared<grid_map::GridMap>(std::vector<std::string>({"elevation", "traversability"}));
  map->setBasicLayers({"elevation", "traversability"});
  map->setGeometry(grid_map::Length(6.0, 6.0), 0.05);

  map->get("elevation").setZero();
  map->get("traversability").setConstant(1.0);

  grid_map::Polygon poly1({{-2.0, -2.0},
                          {2.0, -2.0},
                          {2.0, 2.0},
                          {-2.0, 2.0},
                          {-2.0, -2.0}});
  setPolygonValue(poly1, map, 0.0f, 0.1f);
  grid_map::Polygon poly2({{-1.3, -2.0},
                          {-1.2, -2.0},
                          {-1.2, 2.0},
                          {-1.3, 2.0},
                          {-1.3, -2.0}});
  setPolygonValue(poly2, map, 1.0f, 0.0f);
  grid_map::Polygon poly3({{-1.05, -2.0},
                          {-0.95, -2.0},
                          {-0.95, 2.0},
                          {-1.05, 2.0},
                          {-1.05, -2.0}});
  setPolygonValue(poly3, map, 1.0f, 0.0f);
  grid_map::Polygon poly4({{1.3, -2.0},
                          {1.2, -2.0},
                          {1.2, 2.0},
                          {1.3, 2.0},
                          {1.3, -2.0}});
  setPolygonValue(poly4, map, 1.0f, 0.0f);
  grid_map::Polygon poly5({{1.05, -2.0},
                          {0.95, -2.0},
                          {0.95, 2.0},
                          {1.05, 2.0},
                          {1.05, -2.0}});
  setPolygonValue(poly5, map, 1.0f, 0.0f);
  grid_map::Polygon poly6({{1.2, -2.0},
                           {1.05, -2.0},
                           {1.05, 1.5},
                           {1.2, 1.5},
                           {1.2, -2.0}});
  setPolygonValue(poly6, map, 0.0f, 0.8f);

  map->atPosition("elevation", grid_map::Position(2.5, 2.5)) = 0.5;

  map->at("elevation", grid_map::Index(119,119)) = NAN;
  map->at("elevation", grid_map::Index(118,119)) = NAN;
  map->at("elevation", grid_map::Index(119,118)) = NAN;
  map->at("elevation", grid_map::Index(118,118)) = NAN;

  // Reverse y axis for ODE.
  auto start = std::chrono::high_resolution_clock::now();
  map->get("elevation").rowwise().reverseInPlace();
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end-start;
  std::cout << "Time to do reverse checking "<< diff.count() << " s" << std::endl;

  return map;
}



static constexpr int kNBox = 9;



// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dBodyID body[kNBox];
static dBodyID body2;
static dGeomID geom[kNBox];
static dGeomID geom2;
static dMass m;
static dJointGroupID contactgroup;



static std::shared_ptr<grid_map::GridMap> grid;



void heightfield_draw_one() {
    dsSetColorAlpha(0.5, 0.9, 0.5, 1.0);
    //dsSetTexture(DS_WOOD);

  const dReal* pos = dGeomGetPosition(geom2);
  const dReal* R = dGeomGetRotation(geom2);

  int ox = (int)(-grid->getLength().x()/2);
  int oz = (int)(-grid->getLength().y()/2);

    int i, j, k = 0;

    int n = grid->getSize().x() * grid->getSize().y() * 2;
    dReal *v = (dReal *)malloc(sizeof(dReal) * n * 9);

    const float res = grid->getResolution();

    for(i = 0; i < grid->getSize().x() - 1; i++) {
    for(j = 0; j < grid->getSize().y() - 1; j++) {
      v[3*(k+0)+0]                = ox + i * res;
      v[3*(k+0)+1]                = grid->get("elevation").data()[j*120+i];
      v[3*(k+0)+2]                = oz + j * res;

      v[3*(k+2)+0] = v[3*(k+3)+0] = ox + (i + 1) * res;
      v[3*(k+2)+1] = v[3*(k+3)+1] = grid->get("elevation").data()[j*120+(i+1)];
      v[3*(k+2)+2] = v[3*(k+3)+2] = oz + j * res;

      v[3*(k+1)+0] = v[3*(k+4)+0] = ox + i * res;
      v[3*(k+1)+1] = v[3*(k+4)+1] = grid->get("elevation").data()[(j+1)*120+i];
      v[3*(k+1)+2] = v[3*(k+4)+2] = oz + (j + 1) * res;

      v[3*(k+5)+0]                = ox + (i + 1) * res;
      v[3*(k+5)+1]                = grid->get("elevation").data()[(j+1)*120+i+1];
      v[3*(k+5)+2]                = oz + (j + 1) * res;

      k += 6;
    }
  }

  dsDrawTriangles(pos, R, v, n, 1);

  free(v);
}



struct TriMesh {
  std::vector<unsigned int> indices;
  std::vector<std::array<dReal, 4> > vertices;

  size_t getNVertices() const {
    return vertices.size();
  }

  size_t getNIndices() const {
    return indices.size();
  }

  dReal* getVertexPtr() {
    return vertices.data()->data();
  }

  unsigned int* getIndexPtr() {
    return indices.data();
  }
};



TriMesh getTriMesh(std::shared_ptr<grid_map::GridMap> map) {
  TriMesh mesh;

  auto& elev = map->get("elevation");
  const auto size = map->getSize();

  Eigen::Matrix<long, Eigen::Dynamic, Eigen::Dynamic> index_cache(size.x(), size.y());
  index_cache.setConstant(-1l);
  long index_counter = 0;

  grid_map::Position pos00;
  grid_map::Position pos01;
  grid_map::Position pos10;
  grid_map::Position pos11;

  long ind00;
  long ind01;
  long ind10;
  long ind11;

  for (long i = 0; i < size.x()-1; ++i) {
    for (long j = 0; j < size.y()-1; ++j) {
      // Check validity.
      const bool valid00 = map->isValid(grid_map::Index(i,   j));
      const bool valid01 = map->isValid(grid_map::Index(i,   j+1));
      const bool valid10 = map->isValid(grid_map::Index(i+1, j));
      const bool valid11 = map->isValid(grid_map::Index(i+1, j+1));
      // Get positions.
      map->getPosition(grid_map::Index(i,  j), pos00);
      map->getPosition(grid_map::Index(i,  j+1), pos01);
      map->getPosition(grid_map::Index(i+1,j), pos10);
      map->getPosition(grid_map::Index(i+1,j+1), pos11);
      // Add upper left triangle.
      if (valid00 && valid01 && valid10) {
        // 00.
        if (index_cache(i, j) == -1) {
          ind00 = index_cache(i, j) = index_counter++;
          mesh.vertices.push_back({(dReal)pos00.x(), (dReal)pos00.y(), elev(i, j)});
        } else {
          ind00 = index_cache(i, j);
        }
        // 01.
        if (index_cache(i, j+1) == -1) {
          ind01 = index_cache(i, j+1) = index_counter++;
          mesh.vertices.push_back({(dReal)pos01.x(), (dReal)pos01.y(), elev(i, j+1)});
        } else {
          ind01 = index_cache(i, j+1);
        }
        // 10.
        ind10 = index_cache(i+1, j) = index_counter++;
        mesh.vertices.push_back({(dReal)pos10.x(), (dReal)pos10.y(), elev(i+1, j)});
        // Add indices to mesh.
        mesh.indices.push_back(ind00);
        mesh.indices.push_back(ind01);
        mesh.indices.push_back(ind10);
      }
      // Add lower right triangle;
      if (valid01 && valid10 && valid11) {
        // 01.
        if (index_cache(i, j+1) == -1) {
          ind01 = index_cache(i, j+1) = index_counter++;
          mesh.vertices.push_back({(dReal)pos01.x(), (dReal)pos01.y(), elev(i, j+1)});
        } else {
          ind01 = index_cache(i, j+1);
        }
        // 10.
        if (index_cache(i+1, j) == -1) {
          ind10 = index_cache(i+1, j) = index_counter++;
          mesh.vertices.push_back({(dReal)pos10.x(), (dReal)pos10.y(), elev(i+1, j)});
        } else {
          ind10 = index_cache(i+1, j);
        }
        // 11.
        if (index_cache(i+1, j+1) == -1) {
          ind11 = index_cache(i+1, j+1) = index_counter++;
          mesh.vertices.push_back({(dReal)pos11.x(), (dReal)pos11.y(), elev(i+1, j+1)});
        } else {
          ind11 = index_cache(i+1, j+1);
        }
        mesh.indices.push_back(ind01);
        mesh.indices.push_back(ind10);
        mesh.indices.push_back(ind11);
      }
    }
  }

  return mesh;
}



// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2) {
   dBodyID b1 = dGeomGetBody(o1);
   dBodyID b2 = dGeomGetBody(o2);
   dContact contact;
   contact.surface.mode = dContactBounce | dContactSoftCFM;
   // friction parameter
   contact.surface.mu = dInfinity;
   // bounce is the amount of "bouncyness".
   contact.surface.bounce = 0.0;
   // bounce_vel is the minimum incoming velocity to cause a bounce
   contact.surface.bounce_vel = 0.1;
   // constraint force mixing parameter
   contact.surface.soft_cfm = 0.001;
   if (int numc = dCollide (o1, o2, 1, &contact.geom, sizeof(dContact))) {
//       dJointID c = dJointCreateContact (world,contactgroup,&contact);
//       dJointAttach (c,b1,b2);
   }
}

// start simulation - set viewpoint
static void start() {
   float xyz[3] = {2.0f,-2.0f,1.7600f};
   float hpr[3] = {140.000f,-17.0000f,0.0000f};
   dsSetViewpoint (xyz,hpr);
}


// simulation loop
static void simLoop (int pause) {

   const dReal *pos;
   const dReal *R;

   // find collisions and add contact joints
   dSpaceCollide (space,0,&nearCallback);

   // step the simulation
   dWorldQuickStep (world,0.00001);
   // remove all contact joints
   dJointGroupEmpty (contactgroup);
   // redraw box at new location
   for (int i = 0; i < kNBox; ++i) {
     pos = dGeomGetPosition (geom[i]);
     R = dGeomGetRotation (geom[i]);
     dVector3 lengths;
     dGeomBoxGetLengths(geom[i], lengths);
     dsDrawBox (pos,R,lengths);
   }
   heightfield_draw_one();
}

int main (int argc, char **argv) {
   // setup pointers to drawstuff callback functions
   dsFunctions fn;
   fn.version = DS_VERSION;
   fn.start = &start;
   fn.step = &simLoop;
   fn.stop = 0;
   fn.command = 0;
   fn.path_to_textures = "/home/lorenwel/git/ode/drawstuff/textures";

   dInitODE ();
   // create world
   world = dWorldCreate ();
   space = dHashSpaceCreate (0);
   dWorldSetGravity (world,0,0,-0.2);
   dWorldSetCFM (world,1e-5);
//   dCreatePlane (space,0,0,1,0);
   contactgroup = dJointGroupCreate (0);


   // create object
   for (int i = 0; i < kNBox; ++i) {
     const auto box_length = 0.5f/3;
     body[i] = dBodyCreate (world);
     geom[i] = dCreateBox (space, box_length, box_length, 0.5f);
     dMassSetBox(&m, 1, 0.5, 0.5, 0.5);
     dBodySetMass (body[i],&m);
     dGeomSetBody (geom[i],body[i]);
     dBodySetPosition (body[i],1.1f+box_length*(i%3),-2.0f+box_length*(i/3),0.0f);
//     dBodySetPosition (body[i],0.0083333f+box_length*(i%3),0.0f+box_length*(i/3),-0.6f);
   }


   grid = getTestMap();
   ///// Create height field. /////
//   dHeightfieldDataID mydat = dGeomHeightfieldDataCreate();
//   geom2 = dCreateHeightfield(space, mydat, 1);
//   dGeomHeightfieldDataBuildSingle(mydat, grid->get("elevation").data(), 0, 6.0f, 6.0f, 120, 120, 1.0, 0.0, 0.0, 0);
//   dGeomHeightfieldSetHeightfieldData(geom2, mydat);

   ///// Create trimesh. //////
   dTriMeshDataID tri_mesh;
   tri_mesh = dGeomTriMeshDataCreate();

   TriMesh mesh_data = getTriMesh(grid);

   const unsigned int indices[6] = {2, 1, 0, 3, 2, 0};
   dVector3 triVert[4] = { { 10.0, 10.0, 0.0},
                           {-10.0, 10.0, 0.0},
                           {-10.0, -10.0, 0.0},
                           { 10.0, -10.0, 0.0} };
   dGeomTriMeshDataBuildSimple(tri_mesh,
                               mesh_data.getVertexPtr() /*(dReal*)triVert*/,
                               mesh_data.getNVertices() /*4*/,
                               mesh_data.getIndexPtr() /*indices*/,
                               mesh_data.getNIndices() /*6*/);
   geom2 = dCreateTriMesh(space, tri_mesh, nullptr, nullptr, nullptr);
   dGeomTriMeshSetData(geom2, tri_mesh);


//   dMatrix3 R;
//   dRFrom2Axes(R, -1, 0, 0, 0, 0, 1);
//   std::cout << R[0] << " " << R[1] << " " << R[2] << "\n";
//   std::cout << R[4] << " " << R[5] << " " << R[6] << "\n";
//   std::cout << R[8] << " " << R[9] << " " << R[10] << std::endl;

   body2 = dBodyCreate (world);
   dGeomSetBody (geom2, body2);
   dBodySetPosition(body2, 0.0f,0.0f,0.0f);
//   dBodySetRotation(body2, R);

   //////////////////
   auto start = std::chrono::high_resolution_clock::now();

   int col_tot = 0;
   static constexpr int kNCol = 4;
   dContactGeom contact_geom[kNCol];
   for (int i = 0; i < 1000; ++i) {
     for (int j = 0; j < kNBox; ++j) {
       col_tot += dCollide (geom[j], geom2, kNCol, contact_geom, sizeof(dContactGeom));
     }
   }
   std::cout << "col total " << col_tot << "\n";
   if (int numc = dCollide (geom[0], geom2, kNCol, contact_geom, sizeof(dContactGeom))) {
     std::cout << "Number contacts: " << numc << std::endl;
     for (int i = 0; i < numc; ++i) {
       std::cout << contact_geom[i].pos[0] << "\t"
                 << contact_geom[i].pos[1] << "\t"
                 << contact_geom[i].pos[2] << "\n";
     }
   }

   auto end = std::chrono::high_resolution_clock::now();
   std::cout << std::flush;
   std::chrono::duration<double> diff = end-start;
   std::cout << "Time to do collision checking "<< diff.count() << " s" << std::endl;


   /////////////////

   // run simulation
   dsSimulationLoop (argc,argv,704,576,&fn);


   // clean up
   dJointGroupDestroy (contactgroup);
   dSpaceDestroy (space);
   dWorldDestroy (world);
   dCloseODE();
   return 0;
}

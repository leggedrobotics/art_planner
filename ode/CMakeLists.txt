cmake_minimum_required(VERSION 3.0.2)

project(ODE)

include(CheckFunctionExists)
include(CheckIncludeFiles)
include(CMakeDependentOption)
include(GNUInstallDirs)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 16)
set(VERSION_PATCH 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

option(BUILD_SHARED_LIBS "Build shared libraries." ON)
option(ODE_16BIT_INDICES "Use 16-bit indices for trimeshes (default is 32-bit)." OFF)
option(ODE_NO_BUILTIN_THREADING_IMPL "Disable built-in multithreaded threading implementation." OFF)
option(ODE_NO_THREADING_INTF "Disable threading interface support (external implementations cannot be assigned." OFF)
option(ODE_OLD_TRIMESH "Use old OPCODE trimesh-trimesh collider." OFF)
option(ODE_WITH_DEMOS "Builds the demo applications and DrawStuff library." ON)
option(ODE_WITH_GIMPACT "Use GIMPACT for trimesh collisions (experimental)." OFF)
option(ODE_WITH_LIBCCD "Use libccd for handling some collision tests absent in ODE." OFF)
option(ODE_WITH_OPCODE "Use old OPCODE trimesh-trimesh collider." ON)
option(ODE_WITH_OU "Use TLS for global caches (allows threaded collision checks for separated spaces)." OFF)
option(ODE_WITH_TESTS "Builds the unit test application." ON)

cmake_dependent_option(ODE_WITH_LIBCCD_BOX_CYL "Use libccd for box-cylinder." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_CAP_CYL "Use libccd for capsule-cylinder." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_CYL_CYL "Use libccd for cylinder-cylinder." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_CONVEX_BOX "Use libccd for convex-box." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_CONVEX_CAP "Use libccd for convex-capsule." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_CONVEX_CONVEX "Use libccd for convex-convex." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_CONVEX_CYL "Use libccd for convex-cylinder." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_CONVEX_SPHERE "Use libccd for convex-sphere." ON "ODE_WITH_LIBCCD" OFF)
cmake_dependent_option(ODE_WITH_LIBCCD_SYSTEM "Use system libccd." OFF "ODE_WITH_LIBCCD" OFF)

option(ODE_DOUBLE_PRECISION "Use double-precision math." OFF)

find_package(OpenGL)
find_package(Threads)

set(CMAKE_REQUIRED_INCLUDES ${OPENGL_INCLUDE_DIR})
set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} ${OPENGL_LIBRARIES})

if(APPLE AND OPENGL_FOUND)
	set(HAVE_APPLE_OPENGL_FRAMEWORK ON)
endif()
check_include_files(alloca.h HAVE_ALLOCA_H)
check_function_exists(gettimeofday HAVE_GETTIMEOFDAY)
check_include_files(inttypes.h HAVE_INTTYPES_H)
check_function_exists(isnan HAVE_ISNAN)
check_function_exists(isnanf HAVE_ISNANF)
check_include_files(malloc.h HAVE_MALLOC_H)
check_function_exists(pthread_attr_setstacklazy HAVE_PTHREAD_ATTR_SETSTACKLAZY)
check_function_exists(pthread_condattr_setclock HAVE_PTHREAD_CONDATTR_SETCLOCK)
check_include_files(stdint.h HAVE_STDINT_H)
check_include_files(sys/time.h HAVE_SYS_TIME_H)
check_include_files(sys/types.h HAVE_SYS_TYPES_H)
check_include_files(unistd.h HAVE_UNISTD_H)
check_function_exists(_isnan HAVE__ISNAN)
check_function_exists(_isnanf HAVE__ISNANF)
check_function_exists(__isnan HAVE___ISNAN)
check_function_exists(__isnanf HAVE___ISNANF)
if(APPLE)
	set(MAC_OS_X_VERSION 1000)
	check_function_exists(OSAtomicAdd32Barrier MAC_OS_X_VERSION_1040)
	if(MAC_OS_X_VERSION_1040)
		set(MAC_OS_X_VERSION 1040)
	endif()
	check_function_exists(OSAtomicAnd32OrigBarrier MAC_OS_X_VERSION_1050)
	if(MAC_OS_X_VERSION_1050)
		set(MAC_OS_X_VERSION 1050)
	endif()
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|x86.*|x86_64.*|amd64.*|AMD64.*")
	set(PENTIUM ON)
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*|amd64.*|AMD64.*")
	set(X86_64_SYSTEM ON)
endif()
if(ODE_WITH_OU)
	set(_OU_FEATURE_SET _OU_FEATURE_SET_TLS)
elseif(NOT ODE_NO_THREADING_INTF)
	set(_OU_FEATURE_SET _OU_FEATURE_SET_ATOMICS)
else()
	set(_OU_FEATURE_SET _OU_FEATURE_SET_BASICS)
endif()
set(_OU_NAMESPACE odeou)
if(WIN32 OR CYGWIN)
	set(_OU_TARGET_OS _OU_TARGET_OS_WINDOWS)
elseif(APPLE)
	set(_OU_TARGET_OS _OU_TARGET_OS_MAC)
elseif(QNXNTO)
	set(_OU_TARGET_OS _OU_TARGET_OS_QNX)
elseif(CMAKE_SYSTEM MATCHES "SunOS-4")
	set(_OU_TARGET_OS _OU_TARGET_OS_SUNOS)
else()
	set(_OU_TARGET_OS _OU_TARGET_OS_GENUNIX)
endif()

configure_file(config.h.cmake.in ode/src/config.h)

if(ODE_DOUBLE_PRECISION)
	set(CCD_PRECISION CCD_DOUBLE)
	set(ODE_PRECISION dDOUBLE)
else()
	set(CCD_PRECISION CCD_SINGLE)
	set(ODE_PRECISION dSINGLE)
endif()

configure_file(libccd/src/ccd/precision.h.in include/ccd/precision.h)
configure_file(include/ode/precision.h.in include/ode/precision.h)

set(ODE_VERSION ${VERSION})

configure_file(include/ode/version.h.in include/ode/version.h)

set(
	HDRS
	include/ode/collision.h
	include/ode/collision_space.h
	include/ode/collision_trimesh.h
	include/ode/common.h
	include/ode/compatibility.h
	include/ode/contact.h
	include/ode/cooperative.h
	include/ode/error.h
	include/ode/export-dif.h
	include/ode/mass.h
	include/ode/matrix.h
	include/ode/matrix_coop.h
	include/ode/memory.h
	include/ode/misc.h
	include/ode/objects.h
	include/ode/ode.h
	include/ode/odeconfig.h
	include/ode/odecpp.h
	include/ode/odecpp_collision.h
	include/ode/odeinit.h
	include/ode/odemath.h
	include/ode/odemath_legacy.h
	include/ode/rotation.h
	include/ode/threading.h
	include/ode/threading_impl.h
	include/ode/timer.h
	${CMAKE_CURRENT_BINARY_DIR}/include/ode/precision.h
	${CMAKE_CURRENT_BINARY_DIR}/include/ode/version.h
)

set(
	SRCS
	ode/src/array.cpp
	ode/src/array.h
	ode/src/box.cpp
	ode/src/capsule.cpp
	ode/src/collision_cylinder_box.cpp
	ode/src/collision_cylinder_plane.cpp
	ode/src/collision_cylinder_sphere.cpp
	ode/src/collision_kernel.cpp
	ode/src/collision_kernel.h
	ode/src/collision_quadtreespace.cpp
	ode/src/collision_sapspace.cpp
	ode/src/collision_space.cpp
	ode/src/collision_space_internal.h
	ode/src/collision_std.h
	ode/src/collision_transform.cpp
	ode/src/collision_transform.h
	ode/src/collision_trimesh_colliders.h
	ode/src/collision_trimesh_disabled.cpp
	ode/src/collision_trimesh_gimpact.h
	ode/src/collision_trimesh_internal.h
	ode/src/collision_trimesh_opcode.h
	ode/src/collision_util.cpp
	ode/src/collision_util.h
	ode/src/common.h
	ode/src/convex.cpp
	ode/src/coop_matrix_types.h
	ode/src/cylinder.cpp
	ode/src/default_threading.cpp
	ode/src/default_threading.h
	ode/src/error.cpp
	ode/src/error.h
	ode/src/export-dif.cpp
	ode/src/fastdot.cpp
	ode/src/fastdot_impl.h
	ode/src/fastldltfactor.cpp
	ode/src/fastldltfactor_impl.h
	ode/src/fastldltsolve.cpp
	ode/src/fastldltsolve_impl.h
	ode/src/fastlsolve.cpp
	ode/src/fastlsolve_impl.h
	ode/src/fastltsolve.cpp
	ode/src/fastltsolve_impl.h
	ode/src/fastvecscale.cpp
	ode/src/fastvecscale_impl.h
	ode/src/heightfield.cpp
	ode/src/heightfield.h
	ode/src/lcp.cpp
	ode/src/lcp.h
	ode/src/mass.cpp
	ode/src/mat.cpp
	ode/src/mat.h
	ode/src/matrix.cpp
	ode/src/matrix.h
	ode/src/memory.cpp
	ode/src/misc.cpp
	ode/src/nextafterf.c
	ode/src/objects.cpp
	ode/src/objects.h
	ode/src/obstack.cpp
	ode/src/obstack.h
	ode/src/ode.cpp
	ode/src/odeinit.cpp
	ode/src/odemath.cpp
	ode/src/odemath.h
	ode/src/odeou.h
	ode/src/odetls.h
	ode/src/plane.cpp
	ode/src/quickstep.cpp
	ode/src/quickstep.h
	ode/src/ray.cpp
	ode/src/resource_control.cpp
	ode/src/resource_control.h
	ode/src/rotation.cpp
	ode/src/simple_cooperative.cpp
	ode/src/simple_cooperative.h
	ode/src/sphere.cpp
	ode/src/step.cpp
	ode/src/step.h
	ode/src/threaded_solver_ldlt.h
	ode/src/threading_atomics_provs.h
	ode/src/threading_base.cpp
	ode/src/threading_base.h
	ode/src/threading_fake_sync.h
	ode/src/threading_impl.cpp
	ode/src/threading_impl.h
	ode/src/threading_impl_posix.h
	ode/src/threading_impl_templates.h
	ode/src/threading_impl_win.h
	ode/src/threading_pool_posix.cpp
	ode/src/threading_pool_win.cpp
	ode/src/threadingutils.h
	ode/src/timer.cpp
	ode/src/typedefs.h
	ode/src/util.cpp
	ode/src/util.h
	ode/src/joints/amotor.cpp
	ode/src/joints/amotor.h
	ode/src/joints/ball.cpp
	ode/src/joints/ball.h
	ode/src/joints/contact.cpp
	ode/src/joints/contact.h
	ode/src/joints/dball.cpp
	ode/src/joints/dball.h
	ode/src/joints/dhinge.cpp
	ode/src/joints/dhinge.h
	ode/src/joints/fixed.cpp
	ode/src/joints/fixed.h
	ode/src/joints/hinge.cpp
	ode/src/joints/hinge.h
	ode/src/joints/hinge2.cpp
	ode/src/joints/hinge2.h
	ode/src/joints/joint.cpp
	ode/src/joints/joint.h
	ode/src/joints/joint_internal.h
	ode/src/joints/joints.h
	ode/src/joints/lmotor.cpp
	ode/src/joints/lmotor.h
	ode/src/joints/null.cpp
	ode/src/joints/null.h
	ode/src/joints/piston.cpp
	ode/src/joints/piston.h
	ode/src/joints/plane2d.cpp
	ode/src/joints/plane2d.h
	ode/src/joints/pr.cpp
	ode/src/joints/pr.h
	ode/src/joints/pu.cpp
	ode/src/joints/pu.h
	ode/src/joints/slider.cpp
	ode/src/joints/slider.h
	ode/src/joints/transmission.cpp
	ode/src/joints/transmission.h
	ode/src/joints/universal.cpp
	ode/src/joints/universal.h
)

if(ODE_WITH_GIMPACT AND NOT ODE_NO_TRIMESH)
	list(
		APPEND SRCS
		GIMPACT/src/gim_boxpruning.cpp
		GIMPACT/src/gim_contact.cpp
		GIMPACT/src/gim_math.cpp
		GIMPACT/src/gim_memory.cpp
		GIMPACT/src/gim_tri_tri_overlap.cpp
		GIMPACT/src/gim_trimesh.cpp
		GIMPACT/src/gim_trimesh_capsule_collision.cpp
		GIMPACT/src/gim_trimesh_ray_collision.cpp
		GIMPACT/src/gim_trimesh_sphere_collision.cpp
		GIMPACT/src/gim_trimesh_trimesh_collision.cpp
		GIMPACT/src/gimpact.cpp
		ode/src/collision_convex_trimesh.cpp
		ode/src/collision_cylinder_trimesh.cpp
		ode/src/collision_trimesh_box.cpp
		ode/src/collision_trimesh_ccylinder.cpp
		ode/src/collision_trimesh_gimpact.cpp
		ode/src/collision_trimesh_internal.cpp
		ode/src/collision_trimesh_internal_impl.h
		ode/src/collision_trimesh_internal.h
		ode/src/collision_trimesh_plane.cpp
		ode/src/collision_trimesh_ray.cpp
		ode/src/collision_trimesh_sphere.cpp
		ode/src/collision_trimesh_trimesh.cpp
		ode/src/gimpact_contact_export_helper.cpp
		ode/src/gimpact_contact_export_helper.h
		ode/src/gimpact_gim_contact_accessor.h
		ode/src/gimpact_plane_contact_accessor.h
	)
endif()

if(ODE_WITH_LIBCCD)
	if(NOT ODE_WITH_LIBCCD_SYSTEM)
		list(
			APPEND SRCS
			libccd/src/alloc.c
			libccd/src/ccd.c
			libccd/src/mpr.c
			libccd/src/polytope.c
			libccd/src/support.c
			libccd/src/vec3.c
			libccd/src/ccd/alloc.h
			libccd/src/ccd/ccd.h
			libccd/src/ccd/compiler.h
			libccd/src/ccd/dbg.h
			libccd/src/ccd/list.h
			libccd/src/ccd/quat.h
			libccd/src/ccd/polytope.h
			libccd/src/ccd/simplex.h
			libccd/src/ccd/support.h
			libccd/src/ccd/vec3.h
			libccd/src/custom/ccdcustom/quat.h
			libccd/src/custom/ccdcustom/vec3.h
		)
	endif()

	list(
		APPEND SRCS
		ode/src/collision_libccd.cpp
		ode/src/collision_libccd.h
	)
endif()

if(ODE_WITH_OPCODE AND NOT ODE_NO_TRIMESH)
	list(
		APPEND SRCS
		ode/src/collision_convex_trimesh.cpp
		ode/src/collision_cylinder_trimesh.cpp
		ode/src/collision_trimesh_box.cpp
		ode/src/collision_trimesh_ccylinder.cpp
		ode/src/collision_trimesh_internal.cpp
		ode/src/collision_trimesh_internal_impl.h
		ode/src/collision_trimesh_internal.h
		ode/src/collision_trimesh_opcode.cpp
		ode/src/collision_trimesh_plane.cpp
		ode/src/collision_trimesh_ray.cpp
		ode/src/collision_trimesh_sphere.cpp
		ode/src/collision_trimesh_trimesh.cpp
		ode/src/collision_trimesh_trimesh_old.cpp
		OPCODE/OPC_AABBCollider.cpp
		OPCODE/OPC_AABBCollider.h
		OPCODE/OPC_AABBTree.cpp
		OPCODE/OPC_AABBTree.h
		OPCODE/OPC_BaseModel.cpp
		OPCODE/OPC_BaseModel.h
		OPCODE/OPC_BoxBoxOverlap.h
		OPCODE/OPC_Collider.cpp
		OPCODE/OPC_Collider.h
		OPCODE/OPC_Common.cpp
		OPCODE/OPC_Common.h
		OPCODE/OPC_HybridModel.cpp
		OPCODE/OPC_HybridModel.h
		OPCODE/OPC_IceHook.h
		OPCODE/OPC_LSSAABBOverlap.h
		OPCODE/OPC_LSSCollider.cpp
		OPCODE/OPC_LSSCollider.h
		OPCODE/OPC_LSSTriOverlap.h
		OPCODE/OPC_MeshInterface.cpp
		OPCODE/OPC_MeshInterface.h
		OPCODE/OPC_Model.cpp
		OPCODE/OPC_Model.h
		OPCODE/OPC_OBBCollider.cpp
		OPCODE/OPC_OBBCollider.h
		OPCODE/OPC_OptimizedTree.cpp
		OPCODE/OPC_OptimizedTree.h
		OPCODE/OPC_Picking.cpp
		OPCODE/OPC_Picking.h
		OPCODE/OPC_PlanesAABBOverlap.h
		OPCODE/OPC_PlanesCollider.cpp
		OPCODE/OPC_PlanesCollider.h
		OPCODE/OPC_PlanesTriOverlap.h
		OPCODE/OPC_RayAABBOverlap.h
		OPCODE/OPC_RayCollider.cpp
		OPCODE/OPC_RayCollider.h
		OPCODE/OPC_RayTriOverlap.h
		OPCODE/OPC_Settings.h
		OPCODE/OPC_SphereAABBOverlap.h
		OPCODE/OPC_SphereCollider.cpp
		OPCODE/OPC_SphereCollider.h
		OPCODE/OPC_SphereTriOverlap.h
		OPCODE/OPC_TreeBuilders.cpp
		OPCODE/OPC_TreeBuilders.h
		OPCODE/OPC_TreeCollider.cpp
		OPCODE/OPC_TreeCollider.h
		OPCODE/OPC_TriBoxOverlap.h
		OPCODE/OPC_TriTriOverlap.h
		OPCODE/OPC_VolumeCollider.cpp
		OPCODE/OPC_VolumeCollider.h
		OPCODE/Opcode.cpp
		OPCODE/Opcode.h
		OPCODE/Stdafx.h
		OPCODE/Ice/IceAABB.cpp
		OPCODE/Ice/IceAABB.h
		OPCODE/Ice/IceAxes.h
		OPCODE/Ice/IceBoundingSphere.h
		OPCODE/Ice/IceContainer.cpp
		OPCODE/Ice/IceContainer.h
		OPCODE/Ice/IceFPU.h
		OPCODE/Ice/IceHPoint.cpp
		OPCODE/Ice/IceHPoint.h
		OPCODE/Ice/IceIndexedTriangle.cpp
		OPCODE/Ice/IceIndexedTriangle.h
		OPCODE/Ice/IceLSS.h
		OPCODE/Ice/IceMatrix3x3.cpp
		OPCODE/Ice/IceMatrix3x3.h
		OPCODE/Ice/IceMatrix4x4.cpp
		OPCODE/Ice/IceMatrix4x4.h
		OPCODE/Ice/IceMemoryMacros.h
		OPCODE/Ice/IceOBB.cpp
		OPCODE/Ice/IceOBB.h
		OPCODE/Ice/IcePairs.h
		OPCODE/Ice/IcePlane.cpp
		OPCODE/Ice/IcePlane.h
		OPCODE/Ice/IcePoint.cpp
		OPCODE/Ice/IcePoint.h
		OPCODE/Ice/IcePreprocessor.h
		OPCODE/Ice/IceRandom.cpp
		OPCODE/Ice/IceRandom.h
		OPCODE/Ice/IceRay.cpp
		OPCODE/Ice/IceRay.h
		OPCODE/Ice/IceRevisitedRadix.cpp
		OPCODE/Ice/IceRevisitedRadix.h
		OPCODE/Ice/IceSegment.cpp
		OPCODE/Ice/IceSegment.h
		OPCODE/Ice/IceTriangle.cpp
		OPCODE/Ice/IceTriangle.h
		OPCODE/Ice/IceTriList.h
		OPCODE/Ice/IceTypes.h
		OPCODE/Ice/IceUtils.cpp
		OPCODE/Ice/IceUtils.h
	)
endif()

list(
	APPEND SRCS
	ode/src/odeou.cpp
	ode/src/odeou.h
	ode/src/odetls.cpp
	ode/src/odetls.h
	ou/src/ou/atomic.cpp
	ou/src/ou/customization.cpp
	ou/src/ou/malloc.cpp
	ou/src/ou/threadlocalstorage.cpp
)

add_library(ODE ${SRCS})

set_target_properties(
	ODE
	PROPERTIES
	OUTPUT_NAME ode
	POSITION_INDEPENDENT_CODE ON
	VERSION ${VERSION}
)

if(WIN32)
	if(BUILD_SHARED_LIBS)
		set_target_properties(ODE PROPERTIES DEBUG_POSTFIX d)
	else()
		set_target_properties(
			ODE
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()

	if(ODE_DOUBLE_PRECISION)
		set_target_properties(ODE PROPERTIES OUTPUT_NAME ode_double)
	else()
		set_target_properties(ODE PROPERTIES OUTPUT_NAME ode_single)
	endif()
endif()

target_compile_definitions(
	ODE
	PRIVATE
	-D_OU_NAMESPACE=${_OU_NAMESPACE}
	-D_OU_FEATURE_SET=${_OU_FEATURE_SET}
	-D_OU_TARGET_OS=${_OU_TARGET_OS}
	$<$<NOT:$<CONFIG:Debug>>:dNODEBUG>
	-DdOU_ENABLED
)

if(APPLE)
	target_compile_definitions(ODE PRIVATE -DMAC_OS_X_VERSION=${MAC_OS_X_VERSION})
endif()

if(WIN32)
	target_compile_definitions(ODE PRIVATE -D_CRT_SECURE_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)
endif()

if(BUILD_SHARED_LIBS)
	target_compile_definitions(ODE PRIVATE -DODE_DLL)
else()
	target_compile_definitions(ODE PRIVATE -DODE_LIB)
endif()

if(ODE_DOUBLE_PRECISION)
	target_compile_definitions(ODE PUBLIC -DdIDEDOUBLE PRIVATE -DCCD_IDEDOUBLE)
else()
	target_compile_definitions(ODE PUBLIC -DdIDESINGLE PRIVATE -DCCD_IDESINGLE)
endif()

target_include_directories(
	ODE
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ode/src>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ode/src>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ode/src/joints>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ou/include>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>
)

if(ODE_16BIT_INDICES)
	target_compile_definitions(ODE PRIVATE -DdTRIMESH_16BIT_INDICES)
endif()

if(NOT ODE_NO_BUILTIN_THREADING_IMPL)
	target_compile_definitions(ODE PRIVATE -DdBUILTIN_THREADING_IMPL_ENABLED)
endif()

if(ODE_NO_THREADING_INTF)
	target_compile_definitions(ODE PRIVATE -DdTHREADING_INTF_DISABLED)
endif()

if(ODE_WITH_GIMPACT AND NOT ODE_NO_TRIMESH)
	target_compile_definitions(ODE PRIVATE -DdTRIMESH_ENABLED -DdTRIMESH_GIMPACT)
	target_include_directories(ODE PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/GIMPACT/include>)
endif()

if(ODE_WITH_LIBCCD)
	if(ODE_WITH_LIBCCD_SYSTEM)
		find_package(ccd)
		target_compile_definitions(ode PRIVATE -DdLIBCCD_ENABLED -DdLIBCCD_SYSTEM)
		target_link_libraries(ODE ccd::ccd)
	else()
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_ENABLED -DdLIBCCD_INTERNAL)
		target_include_directories(
			ODE
			PRIVATE
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libccd/src>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libccd/src/custom>
		)
	endif()

	if(ODE_WITH_LIBCCD_BOX_CYL)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_BOX_CYL)
	endif()

	if(ODE_WITH_LIBCCD_CAP_CYL)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_CAP_CYL)
	endif()

	if(ODE_WITH_LIBCCD_CYL_CYL)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_CYL_CYL)
	endif()

	if(ODE_WITH_LIBCCD_CONVEX_BOX)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_CONVEX_BOX)
	endif()

	if(ODE_WITH_LIBCCD_CONVEX_CAP)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_CONVEX_CAP)
	endif()

	if(ODE_WITH_LIBCCD_CONVEX_CONVEX)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_CONVEX_CONVEX)
	endif()

	if(ODE_WITH_LIBCCD_CONVEX_CYL)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_CONVEX_CYL)
	endif()

	if(ODE_WITH_LIBCCD_CONVEX_SPHERE)
		target_compile_definitions(ODE PRIVATE -DdLIBCCD_CONVEX_SPHERE)
	endif()
endif()

if(ODE_WITH_OPCODE AND NOT ODE_NO_TRIMESH)
	target_compile_definitions(ODE PRIVATE -DdTRIMESH_ENABLED -DdTRIMESH_OPCODE)

	if(ODE_OLD_TRIMESH)
		target_compile_definitions(ODE PRIVATE -DdTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER)
	endif()

	target_include_directories(
		ODE
		PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/OPCODE>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/OPCODE/Ice>
	)
endif()

if(ODE_WITH_OU)
	target_compile_definitions(ODE PRIVATE -DdATOMICS_ENABLED -DdTLS_ENABLED)
elseif(NOT ODE_NO_THREADING_INTF)
	target_compile_definitions(ODE PRIVATE -DdATOMICS_ENABLED)
endif()

if(ODE_WITH_OU OR NOT ODE_NO_THREADING_INTF)
	target_link_libraries(ODE ${CMAKE_THREAD_LIBS_INIT})
endif()

install(
	TARGETS	ODE
	EXPORT ODE
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)

if(BUILD_SHARED_LIBS)
	install(
		TARGETS	ODE
		EXPORT ODE
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
	)
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:ODE> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ode COMPONENT development)

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix "\${prefix}")
set(libdir "\${exec_prefix}/${CMAKE_INSTALL_LIBDIR}")
set(includedir "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
configure_file(ode.pc.in ode.pc @ONLY)
configure_file(ode-config.in ode-config @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ode.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig COMPONENT development)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/ode-config DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT development)

if(ODE_WITH_DEMOS)
	set(
		DRAWSTUFF_SRCS
		include/drawstuff/drawstuff.h
		include/drawstuff/version.h
		drawstuff/src/drawstuff.cpp
		drawstuff/src/internal.h
	)

	if(WIN32)
		list(
			APPEND DRAWSTUFF_SRCS
			drawstuff/src/resource.h
			drawstuff/src/resources.rc
			drawstuff/src/windows.cpp
		)
	elseif(APPLE)
		list(APPEND DRAWSTUFF_SRCS drawstuff/src/osx.cpp)
	else()
		list(APPEND DRAWSTUFF_SRCS drawstuff/src/x11.cpp)
	endif()

	add_library(drawstuff ${DRAWSTUFF_SRCS})
	target_compile_definitions(drawstuff PUBLIC -DDRAWSTUFF_TEXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/drawstuff/textures")

	if(BUILD_SHARED_LIBS)
		target_compile_definitions(drawstuff PRIVATE -DDS_DLL -DUSRDLL)
	else()
		target_compile_definitions(drawstuff PRIVATE -DDS_LIB)
	endif()

	target_include_directories(drawstuff PUBLIC ${OPENGL_INCLUDE_DIR})
	target_link_libraries(drawstuff ODE ${OPENGL_LIBRARIES})

	if(WIN32)
		target_link_libraries(drawstuff winmm)
	elseif(APPLE)
		find_package(GLUT)
		target_include_directories(drawstuff PUBLIC ${GLUT_INCLUDE_DIR})
		target_link_libraries(drawstuff ${GLUT_LIBRARIES})
	else()
		find_package(X11)
		target_include_directories(drawstuff PUBLIC ${X11_INCLUDE_DIR})
		target_link_libraries(drawstuff ${X11_LIBRARIES})
	endif()

	set(
		DEMO_SRCS
		ode/demo/demo_boxstack.cpp
		ode/demo/demo_buggy.cpp
		ode/demo/demo_cards.cpp
		ode/demo/demo_chain1.c
		ode/demo/demo_chain2.cpp
		ode/demo/demo_collision.cpp
		ode/demo/demo_convex.cpp
		ode/demo/demo_crash.cpp
		ode/demo/demo_cylvssphere.cpp
		ode/demo/demo_dball.cpp
		ode/demo/demo_dhinge.cpp
		ode/demo/demo_feedback.cpp
		ode/demo/demo_friction.cpp
		ode/demo/demo_gyro2.cpp
		ode/demo/demo_gyroscopic.cpp
		ode/demo/demo_heightfield.cpp
		ode/demo/demo_hinge.cpp
		ode/demo/demo_I.cpp
		ode/demo/demo_jointPR.cpp
		ode/demo/demo_jointPU.cpp
		ode/demo/demo_joints.cpp
		ode/demo/demo_kinematic.cpp
		ode/demo/demo_motion.cpp
		ode/demo/demo_motor.cpp
		ode/demo/demo_ode.cpp
		ode/demo/demo_piston.cpp
		ode/demo/demo_plane2d.cpp
		ode/demo/demo_rfriction.cpp
		ode/demo/demo_slider.cpp
		ode/demo/demo_space.cpp
		ode/demo/demo_space_stress.cpp
		ode/demo/demo_step.cpp
		ode/demo/demo_transmission.cpp
	)

	if(NOT ODE_NO_TRIMESH)
		list(
			APPEND DEMO_SRCS
			ode/demo/demo_basket.cpp
			ode/demo/demo_cyl.cpp
			ode/demo/demo_moving_convex.cpp
			ode/demo/demo_moving_trimesh.cpp
			ode/demo/demo_tracks.cpp
			ode/demo/demo_trimesh.cpp
		)
	endif()

	foreach(DEMO_SRC ${DEMO_SRCS})
		get_filename_component(DEMO ${DEMO_SRC} NAME_WE)
		add_executable(${DEMO} ${DEMO_SRC})
		target_link_libraries(${DEMO} drawstuff)

		if(NOT WIN32 AND ${DEMO} MATCHES "demo_chain1")
			target_link_libraries(${DEMO} m)
		endif()
	endforeach()
endif()

if(ODE_WITH_TESTS)
	set(
		TEST_SRCS
		tests/collision.cpp
		tests/friction.cpp
		tests/joint.cpp
		tests/main.cpp
		tests/odemath.cpp
		tests/joints/amotor.cpp
		tests/joints/ball.cpp
		tests/joints/dball.cpp
		tests/joints/fixed.cpp
		tests/joints/hinge.cpp
		tests/joints/hinge2.cpp
		tests/joints/piston.cpp
		tests/joints/pr.cpp
		tests/joints/pu.cpp
		tests/joints/slider.cpp
		tests/joints/universal.cpp
		tests/UnitTest++/src/AssertException.cpp
		tests/UnitTest++/src/AssertException.h
		tests/UnitTest++/src/CheckMacros.h
		tests/UnitTest++/src/Checks.cpp
		tests/UnitTest++/src/Checks.h
		tests/UnitTest++/src/Config.h
		tests/UnitTest++/src/DeferredTestReporter.cpp
		tests/UnitTest++/src/DeferredTestReporter.h
		tests/UnitTest++/src/DeferredTestResult.cpp
		tests/UnitTest++/src/DeferredTestResult.h
		tests/UnitTest++/src/MemoryOutStream.cpp
		tests/UnitTest++/src/MemoryOutStream.h
		tests/UnitTest++/src/ReportAssert.cpp
		tests/UnitTest++/src/ReportAssert.h
		tests/UnitTest++/src/Test.cpp
		tests/UnitTest++/src/TestDetails.cpp
		tests/UnitTest++/src/TestDetails.h
		tests/UnitTest++/src/Test.h
		tests/UnitTest++/src/TestList.cpp
		tests/UnitTest++/src/TestList.h
		tests/UnitTest++/src/TestMacros.h
		tests/UnitTest++/src/TestReporter.cpp
		tests/UnitTest++/src/TestReporter.h
		tests/UnitTest++/src/TestReporterStdout.cpp
		tests/UnitTest++/src/TestReporterStdout.h
		tests/UnitTest++/src/TestResults.cpp
		tests/UnitTest++/src/TestResults.h
		tests/UnitTest++/src/TestRunner.cpp
		tests/UnitTest++/src/TestRunner.h
		tests/UnitTest++/src/TestSuite.h
		tests/UnitTest++/src/TimeConstraint.cpp
		tests/UnitTest++/src/TimeConstraint.h
		tests/UnitTest++/src/TimeHelpers.h
		tests/UnitTest++/src/UnitTest++.h
		tests/UnitTest++/src/XmlTestReporter.cpp
		tests/UnitTest++/src/XmlTestReporter.h
	)

	if(WIN32)
		list(
			APPEND TEST_SRCS
			tests/UnitTest++/src/Win32/TimeHelpers.cpp
			tests/UnitTest++/src/Win32/TimeHelpers.h
		)
	else()
		list(
			APPEND TEST_SRCS
			tests/UnitTest++/src/Posix/SignalTranslator.cpp
			tests/UnitTest++/src/Posix/SignalTranslator.h
			tests/UnitTest++/src/Posix/TimeHelpers.cpp
			tests/UnitTest++/src/Posix/TimeHelpers.h
		)
	endif()

	add_executable(tests ${TEST_SRCS})
	target_include_directories(tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tests/UnitTest++/src)
	target_link_libraries(tests ODE)

	enable_testing()
	add_test(tests ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ode-config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/ode-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode
)
install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/ode-config.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode-${VERSION}
	COMPONENT development
)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/ode-config-version.cmake
	VERSION ${VERSION}
	COMPATIBILITY ExactVersion
)
install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/ode-config-version.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode-${VERSION}
	COMPONENT development
)
install(
	EXPORT ODE
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ode-${VERSION}
	NAMESPACE ODE::
	FILE ode-export.cmake
	COMPONENT development
)

configure_file(cmake/cmake_uninstall.cmake.in cmake_uninstall.cmake @ONLY)
add_custom_target(uninstall ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "ODE is a free, industrial quality library for simulating articulated rigid body dynamics - for example ground vehicles, legged creatures, and moving objects in VR environments. It is fast, flexible, robust and platform independent, with advanced joints, contact with friction, and built-in collision detection.")

set(CPACK_COMPONENT_DEVELOPMENT_DEPENDS runtime)
set(CPACK_COMPONENT_DEVELOPMENT_DESCRIPTION "Open Dynamics Engine - development files\n${CPACK_DEBIAN_PACKAGE_DESCRIPTION}")
set(CPACK_COMPONENT_RUNTIME_DESCRIPTION "Open Dynamics Engine - runtime library\n${CPACK_DEBIAN_PACKAGE_DESCRIPTION}")
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_DEBIAN_DEVELOPMENT_FILE_NAME "DEB-DEFAULT")
set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_SECTION "libdevel")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://www.ode.org/")
set(CPACK_DEBIAN_PACKAGE_NAME "libode")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_RUNTIME_FILE_NAME "DEB-DEFAULT")
set(CPACK_DEBIAN_RUNTIME_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_RUNTIME_PACKAGE_SHLIBDEPS ON)
set(CPACK_NSIS_PACKAGE_NAME "ODE ${VERSION}")
set(CPACK_NSIS_URL_INFO_ABOUT "http://www.ode.org/")
set(CPACK_PACKAGE_CONTACT "ode@ode.org")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "High performance library for simulating rigid body dynamics")
set(CPACK_PACKAGE_DISPLAY_NAME "ODE ${VERSION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "ode-${VERSION}")
set(CPACK_PACKAGE_NAME "ode")
set(CPACK_PACKAGE_VENDOR "")
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/COPYING)
set(CPACK_RPM_COMPONENT_INSTALL ON)
set(CPACK_RPM_DEVELOPMENT_FILE_NAME "RPM-DEFAULT")
set(CPACK_RPM_development_PACKAGE_DESCRIPTION "The ode-devel package contains libraries and header files for developing applications that use ode or ode-double.")
set(CPACK_RPM_development_PACKAGE_NAME "ode-devel")
set(CPACK_RPM_development_PACKAGE_SUMMARY "Development files for ODE")
set(CPACK_RPM_PACKAGE_DESCRIPTION "ODE is an open source, high performance library for simulating rigid body dynamics. It is fully featured, stable, mature and platform independent with an easy to use C/C++ API. It has advanced joint types and integrated collision detection with friction. ODE is useful for simulating vehicles, objects in virtual reality environments and virtual creatures. It is currently used in many computer games, 3D authoring tools and simulation tools.")
set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
set(CPACK_RPM_PACKAGE_LICENSE "BSD or LGPLv2+")
set(CPACK_RPM_PACKAGE_NAME "ode")
set(CPACK_RPM_PACKAGE_SUMMARY "High performance library for simulating rigid body dynamics")
set(CPACK_RPM_PACKAGE_URL "http://www.ode.org/")
set(CPACK_RPM_RUNTIME_FILE_NAME "RPM-DEFAULT")

if(ODE_DOUBLE_PRECISION)
	set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_CONFLICTS "libode-sp-dev")
	set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_DEPENDS "libode6")
	set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_NAME "libode-dev")
	set(CPACK_DEBIAN_RUNTIME_PACKAGE_CONFLICTS "libode6sp")
	set(CPACK_DEBIAN_RUNTIME_PACKAGE_NAME "libode6")
	set(CPACK_RPM_development_PACKAGE_REQUIRES "ode-double")
	set(CPACK_RPM_runtime_PACKAGE_CONFLICTS "ode")
	set(CPACK_RPM_runtime_PACKAGE_DESCRIPTION "The ode-double package contains a version of the ODE library for simulating rigid body dynamics compiled with double precision.")
	set(CPACK_RPM_runtime_PACKAGE_NAME "ode-double")
	set(CPACK_RPM_runtime_PACKAGE_SUMMARY "ODE physics library compiled with double precision")
else()
	set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_CONFLICTS "libode-dev")
	set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_DEPENDS "libode6sp")
	set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_NAME "libode-sp-dev")
	set(CPACK_DEBIAN_RUNTIME_PACKAGE_CONFLICTS "libode6")
	set(CPACK_DEBIAN_RUNTIME_PACKAGE_NAME "libode6sp")
	set(CPACK_RPM_development_PACKAGE_REQUIRES "ode")
	set(CPACK_RPM_runtime_PACKAGE_CONFLICTS "ode-double")
	set(CPACK_RPM_runtime_PACKAGE_NAME "ode")
endif()

include(CPack)

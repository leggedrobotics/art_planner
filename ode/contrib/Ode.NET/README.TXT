Ode.NET - .NET bindings for ODE
Jason Perkins (starkos@industriousone.com)
 

--------------------------------------------------------------------- 
 INSTALLATION
--------------------------------------------------------------------- 

 This binding uses a C# 2.0 feature (the UnmanagedFunctionPointer 
 attribute). You will need to use Visual Studio 2005 (C# Express is 
 fine) or Mono's gmcs compiler.
 
 Start by getting or building ODE as a shared library (DLL).
 
 The simplest way to build the bindings is probably to create a 
 new library assembly in your tool of choice and drop in the files
 Ode/Ode.cs and Ode/AssemblyInfo.cs (optional). Define the symbol 
 `dDOUBLE` if you used double-precision math in your ode.dll. 
 Build, done.
 
 For testing purposes, I have also created bindings for the 
 Drawstuff library and a C# version of the BoxStack demo. You can
 throw all of these files into a console executable and run it to
 see the demo.
 
 If you happen to have Premake installed (http://premake.sf.net/),
 you can generate build scripts for the library with:
 
  premake --target (toolset)                 # for single precision
  premake --with-doubles --target (toolset)  # for double precision

 To build the test application too, use:
 
  premake --with-tests --target (toolset)
  
 To build with Mono, you must add the --dotnet parameter to enable
 support .NET 2.0:
 
   premake --dotnet mono2 --target gnu
   

--------------------------------------------------------------------- 
 USAGE
--------------------------------------------------------------------- 

 I have tried to keep things as close to the original C API as I can,
 rather than forcing a class structure on everyone. Everything is
 contained within the `Ode.NET` namespace inside a static class
 named `d`. All ODE IDs are replaced with IntPtrs. A quick example:
 
   using Ode.NET;
   
   IntPtr world = d.WorldCreate();
   IntPtr body = d.BodyCreate(world);
 
 Take a look at Tests/BoxStack.cs for a more complete example.
 
 
--------------------------------------------------------------------- 
 KNOWN ISSUES
--------------------------------------------------------------------- 

 While I managed to map most of the API, there may still be some bits
 missing. If you find something, please submit a bug report or patch
 the Tracker on SourceForge.

 Collision response (contact joints) do not work when built under 
 Mono as double-precision. I have not tried to track down why.

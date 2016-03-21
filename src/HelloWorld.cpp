#include <iostream>
#include <glm/glm.hpp>

#include <maya/MSimple.h>
#include <maya/MIOStream.h>


DeclareSimpleCommand(HelloWorld, "Autodesk", "2015");

MStatus HelloWorld::doIt(const MArgList&)
{
	std::cout << "Hello World\n" << std::endl;
	return MS::kSuccess;
}
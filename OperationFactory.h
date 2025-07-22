#pragma once

#include "IOperation.h" // Include the IOperation interface

// Define a type for the factory function pointer
typedef std::unique_ptr<IOperation> (*CreateOperationInstanceFunc)();

// Define a type for the get name function pointer
typedef const char* (*GetOperationNameFunc)();

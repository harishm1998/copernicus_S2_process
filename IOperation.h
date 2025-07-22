#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory> // For std::unique_ptr

// Forward declaration for GDALDataset to avoid including gdal_priv.h here
// This keeps the interface header clean and reduces compilation dependencies.
class GDALDataset;

// The core interface for all operations
class IOperation {
public:
    // Virtual destructor is crucial for proper cleanup of derived classes
    virtual ~IOperation() = default;

    // Returns the name of the operation (e.g., "NDVI", "ADD")
    virtual std::string getName() const = 0;

    // Executes the operation.
    // bandPaths: Map of band names (e.g., "B04") to their file paths.
    // args: Operation-specific arguments (e.g., NIR and RED band names for NDVI).
    // outputPath: The path where the result TIFF should be saved.
    // Returns true on success, false on failure.
    virtual bool execute(const std::map<std::string, std::string>& bandPaths,
                         const std::vector<std::string>& args,
                         const std::string& outputPath) = 0;
};

// --- C-compatible factory function signature ---
// This function will be exported by each plugin shared library.
// It returns a unique_ptr to an IOperation instance.
extern "C" std::unique_ptr<IOperation> createOperationInstance();

// --- C-compatible function to get operation name (optional, but useful for discovery) ---
// This function will also be exported by each plugin shared library.
extern "C" const char* getOperationName();

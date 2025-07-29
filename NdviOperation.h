#pragma once

#include "IOperation.h" // Include the base interface
#include <string>
#include <vector>
#include <map>
#include <optional>   // For std::optional
#include <memory>     // For std::unique_ptr
#include <filesystem> // <--- ADDED: Required for std::filesystem

// Forward declarations for helper functions (if they remain global in s2_processor.cpp)
// In a fully modular setup, these utility functions would ideally be in a separate
// utility library or a static utility class that the plugins can link against.
// For now, we declare them here as they are defined in s2_processor.cpp.
class GDALRasterBand;
class GDALDataset;
namespace fs = std::filesystem; // Assuming fs is used in helpers

std::optional<std::vector<float>> readBandToFloat(GDALRasterBand* poBand, int nXSize, int nYSize);
bool writeOutputTiff(const std::string& outputPath, int nXSize, int nYSize,
                     GDALDataset* poRefDS, const std::vector<float>& outputData);


class NdviOperation : public IOperation {
public:
    // Returns the name of this operation.
    std::string getName() const override {
        return "NDVI";
    }

    // Implements the NDVI calculation logic.
    bool execute(const std::map<std::string, std::string>& bandPaths,
                 const std::vector<std::string>& args,
                 const std::string& outputPath) override;
    std::vector<std::string> getRequiredBands(const std::vector<std::string>& args) const override;

};

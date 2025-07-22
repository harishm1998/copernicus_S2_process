#include "NdviOperation.h"
#include <iostream>
#include <vector>
#include <numeric> // For std::accumulate (if needed, not directly for NDVI)
#include <cmath>   // For std::abs (if needed)
#include <stdexcept> // For std::runtime_error
#include <filesystem> // Required for std::filesystem

// GDAL headers (only include what's strictly necessary here)
#include "gdal_priv.h"

#include "GdalUtils.h" // <--- ADDED: Include GdalUtils.h

// No more re-declarations for readBandToFloat and writeOutputTiff here!


bool NdviOperation::execute(const std::map<std::string, std::string>& bandPaths,
                            const std::vector<std::string>& args,
                            const std::string& outputPath) {
    std::cout << "\n--- Performing NDVI Calculation (Modular) ---" << std::endl;
    if (args.size() != 2) {
        std::cerr << "[ERROR] NDVI operation requires 2 arguments: <NIR_band_name> <RED_band_name>." << std::endl;
        return false;
    }
    const std::string& nirBandName = args[0];
    const std::string& redBandName = args[1];

    if (bandPaths.find(nirBandName) == bandPaths.end() || bandPaths.find(redBandName) == bandPaths.end()) {
        std::cerr << "[ERROR] Required bands for NDVI (" << nirBandName << ", " << redBandName << ") not found." << std::endl;
        return false;
    }

    GDALDataset *poNirDS = (GDALDataset *) GDALOpen(bandPaths.at(nirBandName).c_str(), GA_ReadOnly);
    GDALDataset *poRedDS = (GDALDataset *) GDALOpen(bandPaths.at(redBandName).c_str(), GA_ReadOnly);

    if (poNirDS == nullptr || poRedDS == nullptr) {
        std::cerr << "[ERROR] Could not open NIR or Red band for NDVI. Check paths and file integrity." << std::endl;
        if (poNirDS) GDALClose(poNirDS);
        if (poRedDS) GDALClose(poRedDS);
        return false;
    }
    std::cout << "[DEBUG] NIR and Red datasets opened successfully." << std::endl;

    GDALRasterBand *poNirBand = poNirDS->GetRasterBand(1);
    GDALRasterBand *poRedBand = poRedDS->GetRasterBand(1);

    if (poNirBand == nullptr || poRedBand == nullptr) {
        std::cerr << "[ERROR] Could not get raster band from NIR or Red dataset." << std::endl;
        GDALClose(poNirDS); GDALClose(poRedDS);
        return false;
    }

    int nXSize = poNirBand->GetXSize();
    int nYSize = poNirBand->GetYSize();

    if (nXSize != poRedBand->GetXSize() || nYSize != poRedBand->GetYSize()) {
        std::cerr << "[ERROR] NIR and Red bands have different dimensions (" << nXSize << "x" << nYSize << " vs "
        << poRedBand->GetXSize() << "x" << poRedBand->GetYSize() << "). Cannot perform operation." << std::endl;
        GDALClose(poNirDS); GDALClose(poRedDS);
        return false;
    }
    std::cout << "[DEBUG] Bands have consistent dimensions: " << nXSize << "x" << nYSize << std::endl;

    std::optional<std::vector<float>> nirDataOpt = readBandToFloat(poNirBand, nXSize, nYSize);
    std::optional<std::vector<float>> redDataOpt = readBandToFloat(poRedBand, nXSize, nYSize);

    if (!nirDataOpt || !redDataOpt) {
        std::cerr << "[ERROR] Failed to read data for NDVI calculation." << std::endl;
        GDALClose(poNirDS); GDALClose(poRedDS);
        return false;
    }

    std::vector<float> nirData = *nirDataOpt;
    std::vector<float> redData = *redDataOpt;
    std::vector<float> ndviData(static_cast<size_t>(nXSize) * nYSize);

    for (size_t i = 0; i < ndviData.size(); ++i) {
        float nir = nirData[i];
        float red = redData[i];
        float sum = nir + red;
        if (sum == 0.0f) {
            ndviData[i] = 0.0f; // Handle division by zero, set to 0 or a specific NoData value
        } else {
            ndviData[i] = (nir - red) / sum;
        }
    }
    std::cout << "[INFO] NDVI pixel-wise calculation complete." << std::endl;

    bool success = writeOutputTiff(outputPath, nXSize, nYSize, poNirDS, ndviData);

    GDALClose(poNirDS);
    GDALClose(poRedDS);
    return success;
                            }

                            // --- Factory functions for dynamic loading ---
                            // These must be extern "C" to prevent name mangling
                            extern "C" std::unique_ptr<IOperation> createOperationInstance() {
                                return std::make_unique<NdviOperation>();
                            }

                            extern "C" const char* getOperationName() {
                                return "NDVI"; // Must match the name returned by getName() method
                            }

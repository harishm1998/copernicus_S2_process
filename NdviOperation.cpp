#include "NdviOperation.h"
#include <iostream>
#include <vector>
#include <numeric> // For std::accumulate (if needed, not directly for NDVI)
#include <cmath>   // For std::abs, std::isnan
#include <stdexcept> // For std::runtime_error
#include <filesystem> // Required for std::filesystem
#include <algorithm> // For std::min and std::max (for clipping)

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

    // Loop for NDVI calculation, precisely replicating the Python logic
    for (size_t i = 0; i < ndviData.size(); ++i) {
        float nir = nirData[i];
        float red = redData[i];

        // Replicate Python's 'cond' logic:
        // cond = np.equal((nir_band + red_band), 0)
        // cond = np.logical_or(cond, np.isnan(nir_band))
        // cond = np.logical_or(cond, np.isnan(red_band))
        // cond = np.logical_or(cond, red_band < 0)
        // cond = np.logical_or(cond, nir_band < 0)
        bool overall_cond = ((nir + red) == 0.0f) ||
                            std::isnan(nir) ||
                            std::isnan(red) ||
                            (red < 0.0f) ||
                            (nir < 0.0f);

        if (overall_cond) {
            // Equivalent to np.where(cond, 0.0, ...)
            ndviData[i] = 0.0f;
        } else {
            float sum = nir + red - 2000.0f;
            float raw_ndvi = (nir - red) / sum;

            // Replicate the Python scaling: 1 + np.clip(raw_ndvi, -1, 1) * 125
            // In C++, np.clip(value, min, max) is std::max(min, std::min(value, max))
            float clipped_raw_ndvi = std::max(0.0f, std::min(raw_ndvi, 1.0f));

            float intermediate_scaled_value = 1.0f + clipped_raw_ndvi * 250.0f;

            // Replicate .astype(np.uint8) behavior:
            // When a float is cast to uint8, values outside [0, 255] are typically clamped.
            // So, we clamp the float result to this range before storing.
            ndviData[i] = std::max(0.0f, std::min(intermediate_scaled_value, 255.0f));
        }
    }
    std::cout << "[INFO] NDVI pixel-wise calculation complete with Python-like scaling (0-255 range)." << std::endl;

    // Assuming writeOutputTiff will handle the final conversion from float to uint8
    // when writing the TIFF, or that it expects float values in the 0-255 range.
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

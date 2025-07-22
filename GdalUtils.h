#pragma once

#include <string>
#include <vector>
#include <optional>

// Forward declarations for GDAL types to avoid including gdal_priv.h directly here
class GDALRasterBand;
class GDALDataset;

// Reads a band's data into a float vector. Returns std::nullopt on failure.
std::optional<std::vector<float>> readBandToFloat(GDALRasterBand* poBand, int nXSize, int nYSize);

// Creates and writes processed data to an output TIFF file.
bool writeOutputTiff(const std::string& outputPath, int nXSize, int nYSize,
                     GDALDataset* poRefDS, const std::vector<float>& outputData);

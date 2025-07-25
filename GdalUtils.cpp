#include "GdalUtils.h"
#include <iostream>
#include <vector>

// GDAL headers
#include "gdal_priv.h"
#include "cpl_conv.h" // For CPLFree
#include "ogr_srs_api.h" // For OGRSpatialReference (if needed for more complex CRS handling)

// --- Helper Functions for Band Processing ---

// Reads a band's data into a float vector. Returns std::nullopt on failure.
std::optional<std::vector<float>> readBandToFloat(GDALRasterBand* poBand, int nXSize, int nYSize) {
    std::vector<float> data(static_cast<size_t>(nXSize) * nYSize);
    CPLErr err = poBand->RasterIO(GF_Read, 0, 0, nXSize, nYSize,
                                  data.data(), nXSize, nYSize,
                                  GDT_Float32, 0, 0);
    if (err != CE_None) {
        std::cerr << "[DEBUG] Failed to read raster data into float buffer. GDAL Error: " << err << std::endl;
        return std::nullopt;
    }
    std::cout << "[DEBUG] Successfully read band data into float buffer." << std::endl;
    return data;
}

// Creates and writes processed data to an output TIFF file.
bool writeOutputTiff(const std::string& outputPath, int nXSize, int nYSize,
                     GDALDataset* poRefDS, const std::vector<float>& outputData) {
    std::cout << "[DEBUG] Attempting to write output TIFF to: " << outputPath << std::endl;
    GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName("GTiff");
    if (poDriver == nullptr) {
        std::cerr << "[ERROR] GTiff driver not found. Cannot create output TIFF." << std::endl;
        return false;
    }

    // --- KEY CHANGE HERE: Create a new TIFF file with 1 band, GDT_Byte data type (for uint8) ---
    GDALDataset *poDstDS = poDriver->Create(outputPath.c_str(), nXSize, nYSize, 1, GDT_Byte, nullptr);
    if (poDstDS == nullptr) {
        std::cerr << "[ERROR] Could not create output TIFF file: " << outputPath << std::endl;
        return false;
    }
    std::cout << "[DEBUG] Output TIFF dataset created successfully as GDT_Byte." << std::endl;

    // Copy georeferencing information from the reference dataset
    double adfGeoTransform[6];
    if (poRefDS->GetGeoTransform(adfGeoTransform) == CE_None) {
        poDstDS->SetGeoTransform(adfGeoTransform);
        std::cout << "[DEBUG] Geotransform copied." << std::endl;
    } else {
        std::cerr << "[WARNING] Could not get geotransform from reference dataset. Output TIFF may lack georeferencing." << std::endl;
    }

    const char *pszProjection = poRefDS->GetProjectionRef();
    if (pszProjection != nullptr) {
        poDstDS->SetProjection(pszProjection);
        std::cout << "[DEBUG] Projection copied." << std::endl;
    } else {
        std::cerr << "[WARNING] Could not get projection from reference dataset. Output TIFF may lack projection." << std::endl;
    }

    GDALRasterBand *poDstBand = poDstDS->GetRasterBand(1);
    if (poDstBand == nullptr) {
        std::cerr << "[ERROR] Could not get raster band from output dataset." << std::endl;
        GDALClose(poDstDS);
        return false;
    }

    // --- OPTIONAL: Set NoData value for the uint8 band ---
    // Assuming 0.0f (which becomes 0 after casting to uint8) is your NoData value
    // for the scaled NDVI. This matches the Python code's profile.update(nodata=0).
    poDstBand->SetNoDataValue(0.0);
    std::cout << "[DEBUG] NoData value set to 0.0 for output band." << std::endl;


    // Write the data. GDAL will automatically cast the float values (0-255)
    // to byte values as it writes to the GDT_Byte band.
    CPLErr errWrite = poDstBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize,
                                         const_cast<float*>(outputData.data()), // Source data is still float
                                         nXSize, nYSize,
                                         GDT_Float32, 0, 0); // Source data type is GDT_Float32

    GDALClose(poDstDS); // Crucial for flushing data to disk and closing the file
    if (errWrite != CE_None) {
        std::cerr << "[ERROR] Failed to write processed data to output TIFF. GDAL Error: " << errWrite << std::endl;
        return false;
    }
    std::cout << "[INFO] Processed data successfully written to " << outputPath << std::endl;
    return true;
}

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <filesystem>
#include <optional>
#include <functional>
#include <stdexcept>
#include <fstream>
#include <chrono>
#include <cctype>
#include <memory> // For std::unique_ptr
#include <dlfcn.h> // For dynamic loading functions: dlopen, dlsym, dlclose

#include "unzip.h"
#include "zip.h"

// GDAL headers
#include "gdal_priv.h"
#include "cpl_conv.h"
#include "ogr_srs_api.h"

// Include the IOperation interface and the factory function types
#include "IOperation.h"
#include "OperationFactory.h" // New header for factory function types

// Use std::filesystem namespace for convenience
namespace fs = std::filesystem;

// --- Global GDAL Error Handler ---
void GDALErrorHandler(CPLErr eErrClass, int nErrNo, const char *pszMsg) {
    std::cerr << "[GDAL ERROR] Class: " << eErrClass << ", Code: " << nErrNo << ", Message: " << pszMsg << std::endl;
}

// --- Helper Functions for Band Processing (can be moved to a separate utility file if desired) ---

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

    // Create a new TIFF file with 1 band, Float32 data type
    GDALDataset *poDstDS = poDriver->Create(outputPath.c_str(), nXSize, nYSize, 1, GDT_Float32, nullptr);
    if (poDstDS == nullptr) {
        std::cerr << "[ERROR] Could not create output TIFF file: " << outputPath << std::endl;
        return false;
    }
    std::cout << "[DEBUG] Output TIFF dataset created successfully." << std::endl;

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

    CPLErr errWrite = poDstBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize,
                                          const_cast<float*>(outputData.data()),
                                          nXSize, nYSize,
                                          GDT_Float32, 0, 0);

    GDALClose(poDstDS); // Crucial for flushing data to disk and closing the file
    if (errWrite != CE_None) {
        std::cerr << "[ERROR] Failed to write processed data to output TIFF. GDAL Error: " << errWrite << std::endl;
        return false;
    }
    std::cout << "[INFO] Processed data successfully written to " << outputPath << std::endl;
    return true;
}

// --- Zip Extraction Class (Actual Implementation) ---
class MinizipExtractor {
public:
    // Creates a unique temporary directory for extraction
    std::string createTempDir() {
        std::cout << "[DEBUG] Creating temporary directory for extraction..." << std::endl;
        fs::path tempPath = fs::temp_directory_path() / fs::path("s2_extract_" + std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
        if (!fs::create_directories(tempPath)) {
            throw std::runtime_error("Failed to create temporary directory: " + tempPath.string());
        }
        std::cout << "[DEBUG] Temporary directory created: " << tempPath.string() << std::endl;
        return tempPath.string();
    }

    // Extracts the zip file to the specified destination directory.
    // Returns the path to the root of the extracted .SAFE directory.
    std::string extract(const std::string& zipFilePath, const std::string& destDirPath) {
        std::cout << "\n--- Zip Extraction (Actual Implementation) ---" << std::endl;
        std::cout << "[INFO] Attempting to extract '" << zipFilePath << "' to '" << destDirPath << "'" << std::endl;

        unzFile uf = unzOpen(zipFilePath.c_str());
        if (!uf) {
            std::cerr << "[ERROR] Could not open zip file: " << zipFilePath << std::endl;
            throw std::runtime_error("Failed to open zip file.");
        }
        std::cout << "[DEBUG] Zip file opened with minizip." << std::endl;

        unz_global_info global_info;
        if (unzGetGlobalInfo(uf, &global_info) != UNZ_OK) {
            unzClose(uf);
            throw std::runtime_error("Failed to get global zip info.");
        }

        char filename_in_zip[256]; // Buffer for filename inside zip
        std::string extractedSafePath = ""; // To store the path to the .SAFE directory

        for (uLong i = 0; i < global_info.number_entry; ++i) {
            unz_file_info file_info;
            if (unzGetCurrentFileInfo(uf, &file_info, filename_in_zip, sizeof(filename_in_zip),
                                      nullptr, 0, nullptr, 0) != UNZ_OK) {
                unzClose(uf);
                throw std::runtime_error("Failed to get current file info in zip.");
            }

            fs::path current_file_fs_path = fs::path(destDirPath) / filename_in_zip;
            std::string current_file_path_str = current_file_fs_path.string();

            // Check if it's a directory (ends with '/')
            if (filename_in_zip[strlen(filename_in_zip) - 1] == '/') {
                fs::create_directories(current_file_fs_path);
                std::cout << "[DEBUG] Created directory: " << current_file_path_str << std::endl;
            } else { // It's a file
                // Ensure parent directory exists before writing file
                fs::create_directories(current_file_fs_path.parent_path());

                if (unzOpenCurrentFile(uf) != UNZ_OK) {
                    std::cerr << "[ERROR] Could not open current file in zip: " << filename_in_zip << std::endl;
                    unzClose(uf);
                    throw std::runtime_error("Failed to open current file in zip.");
                }

                std::ofstream outfile(current_file_path_str, std::ios::binary);
                if (!outfile.is_open()) {
                    std::cerr << "[ERROR] Could not create output file: " << current_file_path_str << std::endl;
                    unzCloseCurrentFile(uf);
                    unzClose(uf);
                    throw std::runtime_error("Failed to create output file.");
                }

                char buffer[4096];
                int bytes_read = 0;
                do {
                    bytes_read = unzReadCurrentFile(uf, buffer, sizeof(buffer));
                    if (bytes_read < 0) {
                        std::cerr << "[ERROR] Error reading from zip file: " << filename_in_zip << std::endl;
                        outfile.close();
                        unzCloseCurrentFile(uf);
                        unzClose(uf);
                        throw std::runtime_error("Error reading from zip file.");
                    }
                    if (bytes_read > 0) {
                        outfile.write(buffer, bytes_read);
                    }
                } while (bytes_read > 0);

                outfile.close();
                unzCloseCurrentFile(uf);
                std::cout << "[DEBUG] Extracted file: " << current_file_path_str << std::endl;
            }

            // After extracting the first entry, determine the .SAFE root directory
            if (extractedSafePath.empty()) {
                // Assuming the first entry will always be inside the .SAFE root
                // This is a common pattern for Sentinel-2 archives
                size_t safe_pos = current_file_path_str.find(".SAFE/");
                if (safe_pos != std::string::npos) {
                    extractedSafePath = current_file_path_str.substr(0, safe_pos + 5); // +5 for ".SAFE" length
                } else {
                    // Fallback: If .SAFE not found in first entry, try to infer from zip name
                    fs::path zipFsPath = zipFilePath;
                    std::string expectedSafeDirName = zipFsPath.stem().string();
                    if (!expectedSafeDirName.ends_with(".SAFE")) {
                        expectedSafeDirName += ".SAFE";
                    }
                    extractedSafePath = destDirPath + "/" + expectedSafeDirName;
                    std::cout << "[DEBUG] Inferred .SAFE path as fallback: " << extractedSafePath << std::endl;
                }
            }


            if (i < global_info.number_entry - 1) {
                if (unzGoToNextFile(uf) != UNZ_OK) {
                    unzClose(uf);
                    throw std::runtime_error("Failed to go to next file in zip.");
                }
            }
        }
        unzClose(uf);
        std::cout << "[INFO] Minizip extraction complete." << std::endl;
        
        if (extractedSafePath.empty() || !fs::exists(extractedSafePath) || !fs::is_directory(extractedSafePath)) {
            std::cerr << "[FATAL] Error: Could not determine or verify the extracted .SAFE directory path." << std::endl;
            throw std::runtime_error("Failed to find extracted .SAFE directory.");
        }

        return extractedSafePath;
    }
};

// --- S2Processor Class ---
class S2Processor {
public:
    S2Processor() {
        std::cout << "[INFO] S2Processor instance created." << std::endl;
        CPLSetErrorHandler(GDALErrorHandler);
        GDALAllRegister();
        std::cout << "[INFO] GDAL initialized." << std::endl;
    }

    ~S2Processor() {
        std::cout << "[INFO] S2Processor instance destroyed." << std::endl;
        GDALDestroyDriverManager();
        std::cout << "[INFO] GDAL resources cleaned up." << std::endl;
    }

    // Registers a new operation using a unique_ptr to the IOperation interface
    void registerOperation(std::unique_ptr<IOperation> operation) {
        std::string name = operation->getName();
        std::transform(name.begin(), name.end(), name.begin(), ::toupper); // Normalize name
        operations_[name] = std::move(operation);
        std::cout << "[DEBUG] Operation '" << name << "' registered." << std::endl;
    }

    // Check if an operation with the given name is already registered
    bool isOperationRegistered(const std::string& name) const {
        std::string normalizedName = name;
        std::transform(normalizedName.begin(), normalizedName.end(), normalizedName.begin(), ::toupper);
        return operations_.count(normalizedName) > 0;
    }

    // Adds a plugin handle to the internal map
    void addPluginHandle(const std::string& opName, void* handle) {
        pluginHandles_[opName] = handle;
    }

    // Getter for plugin handles (for cleanup in main)
    const std::map<std::string, void*>& getPluginHandles() const {
        return pluginHandles_;
    }

    // Main processing pipeline
    bool process(const std::string& zipFilePath, const std::string& outputTiffPath,
                 const std::string& operationName, const std::vector<std::string>& operationArgs) {
        std::cout << "\n--- Starting S2Processor Pipeline ---" << std::endl;
        std::cout << "[INFO] Input ZIP: " << zipFilePath << std::endl;
        std::cout << "[INFO] Output TIFF: " << outputTiffPath << std::endl;
        std::cout << "[INFO] Operation: " << operationName << std::endl;

        std::string extractedDirPath;
        try {
            MinizipExtractor extractor;
            std::string tempDir = extractor.createTempDir();
            extractedDirPath = extractor.extract(zipFilePath, tempDir);
        } catch (const std::runtime_error& e) {
            std::cerr << "[FATAL] Error during zip extraction: " << e.what() << std::endl;
            return false;
        }
        std::cout << "[INFO] Zip extraction phase complete. Extracted to: " << extractedDirPath << std::endl;

        bandPaths_ = findBandFiles(extractedDirPath);
        if (bandPaths_.empty()) {
            std::cerr << "[FATAL] No Sentinel-2 bands found in the extracted directory. Exiting." << std::endl;
            return false;
        }
        std::cout << "[INFO] Band discovery complete. Found " << bandPaths_.size() << " bands." << std::endl;

        printBandDetails(bandPaths_);

        std::string normalizedOperationName = operationName;
        std::transform(normalizedOperationName.begin(), normalizedOperationName.end(), normalizedOperationName.begin(), ::toupper);

        auto it = operations_.find(normalizedOperationName);
        if (it != operations_.end()) {
            std::cout << "[INFO] Executing operation: " << normalizedOperationName << std::endl;
            return it->second->execute(bandPaths_, operationArgs, outputTiffPath); // Call execute on the IOperation object
        } else {
            std::cerr << "[ERROR] Unknown or unsupported operation: '" << operationName << "'" << std::endl;
            std::cerr << "[INFO] Available operations: ";
            for (const auto& op : operations_) {
                std::cerr << op.first << " ";
            }
            std::cerr << std::endl;
            return false;
        }
    }

private:
    std::map<std::string, std::string> bandPaths_;
    std::map<std::string, std::unique_ptr<IOperation>> operations_;
    std::map<std::string, void*> pluginHandles_; // To store dlopen handles

    // Recursively finds Sentinel-2 band files (.jp2 or .tif) within the extracted directory.
    std::map<std::string, std::string> findBandFiles(const std::string& extractedDirPath) {
        std::cout << "\n--- Discovering Bands ---" << std::endl;
        std::map<std::string, std::string> discoveredBandPaths;
        const std::string imgDataSubPath = "IMG_DATA"; // Common path component in Sentinel-2 archives

        if (!fs::exists(extractedDirPath) || !fs::is_directory(extractedDirPath)) {
            std::cerr << "[ERROR] Extracted directory path does not exist or is not a directory: " << extractedDirPath << std::endl;
            return discoveredBandPaths;
        }

        try {
            for (const auto& entry : fs::recursive_directory_iterator(extractedDirPath)) {
                if (entry.is_regular_file()) {
                    std::string filePath = entry.path().string();
                    std::string filename = entry.path().filename().string();

                    // Check for common Sentinel-2 band extensions and naming conventions
                    if ((filePath.ends_with(".jp2") || filePath.ends_with(".tif")) &&
                        filePath.find(imgDataSubPath) != std::string::npos && // Must be within IMG_DATA
                        filename.find("_B") != std::string::npos) { // Contains "_B" for band
                        
                        // Extract band name (e.g., B04, B08, B8A)
                        size_t bPos = filename.find("_B");
                        if (bPos != std::string::npos && bPos + 1 < filename.length()) { // Ensure there's content after _B
                            std::string potentialBandName = filename.substr(bPos + 1); // Get everything after _B

                            // Now, parse potentialBandName to get BXX or B8A
                            std::string bandName = "";
                            if (potentialBandName.length() >= 3 && potentialBandName.substr(0, 3) == "B8A") {
                                bandName = "B8A";
                            } else if (potentialBandName.length() >= 3 && potentialBandName[0] == 'B' && isdigit(potentialBandName[1]) && isdigit(potentialBandName[2])) {
                                bandName = potentialBandName.substr(0, 3); // e.g., "B02", "B04", "B08"
                            }
                            
                            if (!bandName.empty()) {
                                // Only add if the band name is not already present, OR if it's a higher resolution band
                                // Sentinel-2 has bands at 10m, 20m, 60m. For NDVI (B04, B08), we prefer 10m.
                                // The current recursive_directory_iterator order is not guaranteed.
                                // Let's prioritize 10m bands if available.
                                std::string resolution = "";
                                if (filePath.find("/R10m/") != std::string::npos) resolution = "10m";
                                else if (filePath.find("/R20m/") != std::string::npos) resolution = "20m";
                                else if (filePath.find("/R60m/") != std::string::npos) resolution = "60m";

                                // If a band with this name is already found, check resolution.
                                // Prefer 10m > 20m > 60m.
                                bool addBand = false;
                                if (discoveredBandPaths.find(bandName) == discoveredBandPaths.end()) {
                                    addBand = true; // Not found, so add it
                                } else {
                                    // Already found, check if this new one has a better resolution
                                    std::string existingPath = discoveredBandPaths[bandName];
                                    std::string existingResolution = "";
                                    if (existingPath.find("/R10m/") != std::string::npos) existingResolution = "10m";
                                    else if (existingPath.find("/R20m/") != std::string::npos) existingResolution = "20m";
                                    else if (existingPath.find("/R60m/") != std::string::npos) existingResolution = "60m";

                                    if (resolution == "10m" && existingResolution != "10m") addBand = true;
                                    else if (resolution == "20m" && existingResolution == "60m") addBand = true;
                                    // No change if new is worse or same resolution unless existing is 60m and new is 20m
                                }

                                if (addBand) {
                                    discoveredBandPaths[bandName] = filePath;
                                    std::cout << "[DEBUG] Found band " << bandName << " (" << resolution << "): " << filePath << std::endl;
                                } else {
                                    std::cout << "[DEBUG] Skipping band " << bandName << " (" << resolution << ") as a better resolution already found." << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "[ERROR] Filesystem error during band discovery: " << e.what() << std::endl;
        }
        
        if (discoveredBandPaths.empty()) {
            std::cerr << "[WARNING] No Sentinel-2 band files (.jp2 or .tif) found in: " << extractedDirPath << std::endl;
        }
        return discoveredBandPaths;
    }

    // Prints detailed information about each discovered band.
    void printBandDetails(const std::map<std::string, std::string>& currentBandPaths) {
        std::cout << "\n--- Discovered Band Details ---" << std::endl;
        if (currentBandPaths.empty()) {
            std::cout << "[INFO] No bands found to display details." << std::endl;
            return;
        }

        for (const auto& pair : currentBandPaths) {
            const std::string& bandName = pair.first;
            const std::string& filePath = pair.second;

            GDALDataset *poDS = (GDALDataset *) GDALOpen(filePath.c_str(), GA_ReadOnly);
            if (poDS == nullptr) {
                std::cerr << "  [ERROR] Could not open " << filePath << " for details." << std::endl;
                continue;
            }

            GDALRasterBand *poBand = poDS->GetRasterBand(1);
            if (poBand == nullptr) {
                std::cerr << "  [ERROR] Could not get raster band from " << filePath << std::endl;
                GDALClose(poDS);
                continue;
            }

            int nXSize = poBand->GetXSize();
            int nYSize = poBand->GetYSize();
            GDALDataType dataType = poBand->GetRasterDataType();
            const char* dataTypeStr = GDALGetDataTypeName(dataType);

            std::cout << "  Band: " << bandName << std::endl;
            std::cout << "    Path: " << filePath << std::endl;
            std::cout << "    Dimensions: " << nXSize << "x" << nYSize << std::endl;
            std::cout << "    Data Type: " << (dataTypeStr ? dataTypeStr : "Unknown") << std::endl;

            // Get NoData value if available
            int bHasNoData;
            double noDataValue = poBand->GetNoDataValue(&bHasNoData);
            if (bHasNoData) {
                std::cout << "    NoData Value: " << noDataValue << std::endl;
            }

            // Get spatial reference system (CRS)
            const char *pszProjection = poDS->GetProjectionRef();
            if (pszProjection != nullptr && std::string(pszProjection).length() > 0) {
                OGRSpatialReference oSRS;
                oSRS.SetFromUserInput(pszProjection);
                char *pszPrettyWKT = nullptr;
                oSRS.exportToPrettyWkt(&pszPrettyWKT, false); // Pretty WKT, not strict
                if (pszPrettyWKT) {
                    std::cout << "    Projection (CRS): " << pszPrettyWKT << std::endl;
                    CPLFree(pszPrettyWKT);
                }
            } else {
                std::cout << "    Projection (CRS): Not available or empty." << std::endl;
            }

            // Get GeoTransform (pixel size, origin, rotation)
            double adfGeoTransform[6];
            if (poDS->GetGeoTransform(adfGeoTransform) == CE_None) {
                std::cout << "    GeoTransform (OriginX, PixelSizeX, RotX, OriginY, RotY, PixelSizeY):" << std::endl;
                std::cout << "      (" << adfGeoTransform[0] << ", " << adfGeoTransform[1] << ", " << adfGeoTransform[2] << ", "
                          << adfGeoTransform[3] << ", " << adfGeoTransform[4] << ", " << adfGeoTransform[5] << ")" << std::endl;
            } else {
                std::cout << "    GeoTransform: Not available." << std::endl;
            }

            GDALClose(poDS);
        }
    }
};

// --- Main Program Entry Point ---
int main(int argc, char* argv[]) {
    std::cout << "[INFO] Program started." << std::endl;

    // --- Command Line Argument Parsing ---
    if (argc < 4) {
        std::cerr << "[FATAL] Usage: " << argv[0] << " <zip_file_path> <output_tiff_path> <operation> [operation_args...]" << std::endl;
        std::cerr << "Operations and their arguments:" << std::endl;
        std::cerr << "  NDVI <NIR_band_name> <RED_band_name> (e.g., B08 B04)" << std::endl;
        std::cerr << "  ADD <band1_name> <band2_name>" << std::endl;
        std::cerr << "  SUB <band1_name> <band2_name> (band1 - band2)" << std::endl;
        std::cerr << "  FILTER <band_name> (applies 3x3 averaging filter)" << std::endl;
        std::cerr << "Example: " << argv[0] << " S2A_MSIL2A_...zip output_ndvi.tif NDVI B08 B04" << std::endl;
        return 1;
    }

    std::string zipFilePath = argv[1];
    std::string outputTiffPath = argv[2];
    std::string operationName = argv[3];

    // Collect operation-specific arguments
    std::vector<std::string> operationArgs;
    for (int i = 4; i < argc; ++i) {
        operationArgs.push_back(argv[i]);
    }
    std::cout << "[DEBUG] Command-line arguments parsed." << std::endl;

    // Create S2Processor instance
    S2Processor processor;

    // --- Dynamic Plugin Loading ---
    // Define a directory where plugins are located
    fs::path pluginDir = fs::current_path() / "plugins"; // Or any other desired path

    if (fs::exists(pluginDir) && fs::is_directory(pluginDir)) {
        std::cout << "[INFO] Searching for operation plugins in: " << pluginDir.string() << std::endl;
        for (const auto& entry : fs::directory_iterator(pluginDir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                std::string pluginPath = entry.path().string();
                std::cout << "[DEBUG] Found potential plugin: " << pluginPath << std::endl;

                void* handle = dlopen(pluginPath.c_str(), RTLD_LAZY | RTLD_LOCAL);
                if (!handle) {
                    std::cerr << "[ERROR] Failed to load plugin '" << pluginPath << "': " << dlerror() << std::endl;
                    continue;
                }

                // Reset dlerror()
                dlerror();

                // Get the getOperationName function pointer
                GetOperationNameFunc getOpName = (GetOperationNameFunc)dlsym(handle, "getOperationName");
                const char* dlsym_error_name = dlerror();
                if (dlsym_error_name) {
                    std::cerr << "[ERROR] Cannot find 'getOperationName' in plugin '" << pluginPath << "': " << dlsym_error_name << std::endl;
                    dlclose(handle);
                    continue;
                }

                std::string opName = getOpName();
                // Normalize the name to match how S2Processor stores it (uppercase)
                std::transform(opName.begin(), opName.end(), opName.begin(), ::toupper);

                if (processor.isOperationRegistered(opName)) {
                    std::cerr << "[WARNING] Operation '" << opName << "' from plugin '" << pluginPath << "' already registered. Skipping." << std::endl;
                    dlclose(handle); // Close if already registered to avoid memory leaks
                    continue;
                }

                // Get the createOperationInstance function pointer
                CreateOperationInstanceFunc createOp = (CreateOperationInstanceFunc)dlsym(handle, "createOperationInstance");
                const char* dlsym_error_create = dlerror();
                if (dlsym_error_create) {
                    std::cerr << "[ERROR] Cannot find 'createOperationInstance' in plugin '" << pluginPath << "': " << dlsym_error_create << std::endl;
                    dlclose(handle);
                    continue;
                }

                // Create an instance of the operation and register it
                std::unique_ptr<IOperation> opInstance = createOp();
                if (opInstance) {
                    processor.registerOperation(std::move(opInstance));
                    processor.addPluginHandle(opName, handle); // <--- Using the new public method
                    std::cout << "[INFO] Successfully loaded and registered plugin: " << opName << " from " << pluginPath << std::endl;
                } else {
                    std::cerr << "[ERROR] Factory function 'createOperationInstance' returned nullptr for plugin '" << pluginPath << "'." << std::endl;
                    dlclose(handle);
                }
            }
        }
    } else {
        std::cout << "[INFO] Plugin directory '" << pluginDir.string() << "' not found or not a directory. No plugins loaded." << std::endl;
    }

    // Execute the processing pipeline
    bool success = processor.process(zipFilePath, outputTiffPath, operationName, operationArgs);

    std::cout << "[INFO] Program finished with status: " << (success ? "SUCCESS" : "FAILURE") << std::endl;

    // --- Clean up dynamically loaded plugins ---
    for (auto const& [name, handle] : processor.getPluginHandles()) {
        std::cout << "[DEBUG] Closing plugin handle for operation: " << name << std::endl;
        dlclose(handle);
    }

    return success ? 0 : 1;
}

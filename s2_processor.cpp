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
#include "GdalUtils.h"        // Include GdalUtils.h

// Use std::filesystem namespace for convenience
namespace fs = std::filesystem;

// --- Global Debug Flag ---
// Set to true by default or controlled by a command-line argument.
// We'll initialize it to false and enable it with --debug.
bool g_isDebugMode = false;

// A helper macro for conditional debug output
#define DEBUG_LOG(msg) \
    if (g_isDebugMode) { std::cout << "[DEBUG] " << msg << std::endl; }

// --- Global GDAL Error Handler ---
void GDALErrorHandler(CPLErr eErrClass, int nErrNo, const char *pszMsg) {
    // GDAL errors should generally always be shown, regardless of debug mode
    std::cerr << "[GDAL ERROR] Class: " << eErrClass << ", Code: " << nErrNo << ", Message: " << pszMsg << std::endl;
}

class TempDirManager {
public:
    // Constructor creates the temporary directory
    TempDirManager() {
        tempPath_ = fs::temp_directory_path() / fs::path("s2_extract_" + std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
        if (!fs::create_directories(tempPath_)) {
            throw std::runtime_error("Failed to create temporary directory: " + tempPath_.string());
        }
        DEBUG_LOG("Temporary directory created by TempDirManager: " << tempPath_.string());
    }

    // Destructor deletes the temporary directory and its contents
    ~TempDirManager() {
        if (fs::exists(tempPath_)) {
            std::error_code ec; // For non-throwing removal
            fs::remove_all(tempPath_, ec);
            if (ec) {
                std::cerr << "[ERROR] Failed to remove temporary directory '" << tempPath_.string() << "': " << ec.message() << std::endl;
            } else {
                DEBUG_LOG("Temporary directory removed by TempDirManager: " << tempPath_.string());
            }
        }
    }

    // Getter to retrieve the path
    std::string getPath() const {
        return tempPath_.string(); // Returns a copy, no dangling reference
    }

    // Prevent copy and assignment
    TempDirManager(const TempDirManager&) = delete;
    TempDirManager& operator=(const TempDirManager&) = delete;

private:
    fs::path tempPath_;
};

//---

class MinizipExtractor {
public:
    // Extracts the zip file to the specified destination directory.
    // It now takes an optional `requiredBandNames` vector for selective extraction.
    // Returns the path to the root of the extracted .SAFE directory.
    std::string extract(const std::string& zipFilePath, const std::string& destDirPath,
                        const std::vector<std::string>& requiredBandNames = {}) {
        std::cout << "\n--- Zip Extraction (Actual Implementation) ---" << std::endl;
        std::cout << "[INFO] Attempting to extract '" << zipFilePath << "' to '" << destDirPath << "'" << std::endl;

        unzFile uf = unzOpen(zipFilePath.c_str());
        if (!uf) {
            std::cerr << "[ERROR] Could not open zip file: " << zipFilePath << std::endl;
            throw std::runtime_error("Failed to open zip file.");
        }
        DEBUG_LOG("Zip file opened with minizip.");

        unz_global_info global_info;
        if (unzGetGlobalInfo(uf, &global_info) != UNZ_OK) {
            unzClose(uf);
            throw std::runtime_error("Failed to get global zip info.");
        }

        char filename_in_zip[256]; // Buffer for filename inside zip
        std::string extractedSafePath = ""; // To store the path to the root of the .SAFE directory

        for (uLong i = 0; i < global_info.number_entry; ++i) {
            unz_file_info file_info;
            if (unzGetCurrentFileInfo(uf, &file_info, filename_in_zip, sizeof(filename_in_zip),
                nullptr, 0, nullptr, 0) != UNZ_OK) {
                unzClose(uf);
                throw std::runtime_error("Failed to get current file info in zip.");
            }

            std::string currentFilename = filename_in_zip;

            // --- Selective Extraction Logic ---
            bool shouldExtract = false;

            // 1. Always extract directories
            if (currentFilename.back() == '/') {
                shouldExtract = true;
            }
            // 2. Always extract essential metadata XML files
            else if (currentFilename.find("MTD_MSIL2A.xml") != std::string::npos ||
                     currentFilename.find("MTD_TL.xml") != std::string::npos ||
                     currentFilename.ends_with(".xml")) { // Catch other XMLs
                shouldExtract = true;
            }
            // 3. Extract specific band files if they are requested
            else if (currentFilename.find("IMG_DATA") != std::string::npos && // Must be in IMG_DATA
                     (currentFilename.ends_with(".jp2") || currentFilename.ends_with(".tif"))) { // Must be an image file
                // If specific bands are requested (i.e., requiredBandNames is not empty)
                if (!requiredBandNames.empty()) {
                    std::string baseFilename = fs::path(currentFilename).filename().string();
                    bool foundRequiredBand = false;
                    for (const std::string& reqBand : requiredBandNames) {
                        // Check if the filename contains the band identifier (e.g., "_B08", "_B04", "_B8A")
                        if (baseFilename.find("_" + reqBand + ".") != std::string::npos ||
                            baseFilename.find("_" + reqBand + "_") != std::string::npos) {
                            foundRequiredBand = true;
                            break;
                        }
                    }
                    if (foundRequiredBand) {
                        shouldExtract = true;
                    }
                } else {
                    // If no specific bands are requested, extract all image data files.
                    shouldExtract = true;
                }
            }

            if (shouldExtract) {
                fs::path current_file_fs_path = fs::path(destDirPath) / currentFilename;
                std::string current_file_path_str = current_file_fs_path.string();

                if (currentFilename.back() == '/') { // It's a directory
                    fs::create_directories(current_file_fs_path);
                    DEBUG_LOG("Created directory: " << current_file_path_str);
                } else { // It's a file
                    fs::create_directories(current_file_fs_path.parent_path());

                    if (unzOpenCurrentFile(uf) != UNZ_OK) {
                        std::cerr << "[ERROR] Could not open current file in zip: " << currentFilename << std::endl;
                        unzCloseCurrentFile(uf);
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
                            std::cerr << "[ERROR] Error reading from zip file: " << currentFilename << std::endl;
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
                    DEBUG_LOG("Extracted file: " << current_file_path_str);
                }

                if (extractedSafePath.empty() && currentFilename.find(".SAFE/") != std::string::npos) {
                    size_t safe_pos = current_file_path_str.find(".SAFE/");
                    if (safe_pos != std::string::npos) {
                        extractedSafePath = current_file_path_str.substr(0, safe_pos + 5);
                    }
                }
            } else {
                DEBUG_LOG("Skipping file (not required): " << currentFilename);
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

        if (extractedSafePath.empty()) {
            fs::path zipFsPath = fs::path(zipFilePath);
            std::string expectedSafeDirName = zipFsPath.stem().string();
            if (!expectedSafeDirName.ends_with(".SAFE")) {
                expectedSafeDirName += ".SAFE";
            }
            extractedSafePath = destDirPath + "/" + expectedSafeDirName;
            DEBUG_LOG("Inferred .SAFE path as fallback: " << extractedSafePath);
        }

        if (extractedSafePath.empty() || !fs::exists(extractedSafePath) || !fs::is_directory(extractedSafePath)) {
            std::cerr << "[FATAL] Error: Could not determine or verify the extracted .SAFE directory path." << std::endl;
            throw std::runtime_error("Failed to find extracted .SAFE directory.");
        }

        return extractedSafePath;
    }
};

//---

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
        operations_.clear();

        for (auto const& [name, handle] : pluginHandles_) {
            DEBUG_LOG("Closing plugin handle for operation: " << name);
            dlclose(handle);
        }
        pluginHandles_.clear();

        GDALDestroyDriverManager();
        std::cout << "[INFO] GDAL resources cleaned up." << std::endl;
    }

    void registerOperation(std::unique_ptr<IOperation> operation) {
        std::string name = operation->getName();
        std::transform(name.begin(), name.end(), name.begin(), ::toupper);
        operations_[name] = std::move(operation);
        DEBUG_LOG("Operation '" << name << "' registered.");
    }

    bool isOperationRegistered(const std::string& name) const {
        std::string normalizedName = name;
        std::transform(normalizedName.begin(), normalizedName.end(), normalizedName.begin(), ::toupper);
        return operations_.count(normalizedName) > 0;
    }

    void addPluginHandle(const std::string& opName, void* handle) {
        pluginHandles_[opName] = handle;
    }

    const std::map<std::string, void*>& getPluginHandles() const {
        return pluginHandles_;
    }

    bool process(const std::string& zipFilePath, const std::string& outputTiffPath,
                 const std::string& operationName, const std::vector<std::string>& operationArgs) {
        std::cout << "\n--- Starting S2Processor Pipeline ---" << std::endl;
        std::cout << "[INFO] Input ZIP: " << zipFilePath << std::endl;
        std::cout << "[INFO] Output TIFF: " << outputTiffPath << std::endl;
        std::cout << "[INFO] Operation: " << operationName << std::endl;

        std::string extractedDirPath;
        std::unique_ptr<TempDirManager> tempDirManager;
        try {
            tempDirManager = std::make_unique<TempDirManager>();
            MinizipExtractor extractor;
            std::string tempDir = tempDirManager->getPath();

            // Get required bands from the operation for selective extraction
            std::vector<std::string> requiredBands;
            std::string normalizedOperationName = operationName;
            std::transform(normalizedOperationName.begin(), normalizedOperationName.end(), normalizedOperationName.begin(), ::toupper);

            auto it = operations_.find(normalizedOperationName);
            if (it != operations_.end()) {
                requiredBands = it->second->getRequiredBands(operationArgs);
                DEBUG_LOG("Operation '" << normalizedOperationName << "' requested bands for extraction: ");
                for (const auto& band : requiredBands) {
                    DEBUG_LOG("  - " << band);
                }
            } else {
                std::cerr << "[ERROR] Unknown or unsupported operation: '" << operationName << "'. Cannot determine required bands for extraction." << std::endl;
                // If operation is not found, we can't get required bands, so extract everything or throw
                // For now, let's proceed with an empty requiredBands list, meaning all will be extracted.
                // Alternatively, you might want to return false here and stop.
            }

            extractedDirPath = extractor.extract(zipFilePath, tempDir, requiredBands);
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

        if (g_isDebugMode) { // Only print band details in debug mode
            printBandDetails(bandPaths_);
        }

        std::string normalizedOperationName = operationName;
        std::transform(normalizedOperationName.begin(), normalizedOperationName.end(), normalizedOperationName.begin(), ::toupper);

        auto it = operations_.find(normalizedOperationName);
        if (it != operations_.end()) {
            std::cout << "[INFO] Executing operation: " << normalizedOperationName << std::endl;
            return it->second->execute(bandPaths_, operationArgs, outputTiffPath);
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
    std::map<std::string, void*> pluginHandles_;

    std::map<std::string, std::string> findBandFiles(const std::string& extractedDirPath) {
        std::cout << "\n--- Discovering Bands ---" << std::endl;
        std::map<std::string, std::string> discoveredBandPaths;
        const std::string imgDataSubPath = "IMG_DATA";

        if (!fs::exists(extractedDirPath) || !fs::is_directory(extractedDirPath)) {
            std::cerr << "[ERROR] Extracted directory path does not exist or is not a directory: " << extractedDirPath << std::endl;
            return discoveredBandPaths;
        }

        try {
            for (const auto& entry : fs::recursive_directory_iterator(extractedDirPath)) {
                if (entry.is_regular_file()) {
                    std::string filePath = entry.path().string();
                    std::string filename = entry.path().filename().string();

                    if ((filePath.ends_with(".jp2") || filePath.ends_with(".tif")) &&
                        filePath.find(imgDataSubPath) != std::string::npos &&
                        filename.find("_B") != std::string::npos) {

                        size_t bPos = filename.find("_B");
                        if (bPos != std::string::npos && bPos + 1 < filename.length()) {
                            std::string potentialBandName = filename.substr(bPos + 1);
                            std::string bandName = "";
                            if (potentialBandName.length() >= 3 && potentialBandName.substr(0, 3) == "B8A") {
                                bandName = "B8A";
                            } else if (potentialBandName.length() >= 3 && potentialBandName[0] == 'B' && isdigit(potentialBandName[1]) && isdigit(potentialBandName[2])) {
                                bandName = potentialBandName.substr(0, 3);
                            }

                            if (!bandName.empty()) {
                                std::string resolution = "";
                                if (filePath.find("/R10m/") != std::string::npos) resolution = "10m";
                                else if (filePath.find("/R20m/") != std::string::npos) resolution = "20m";
                                else if (filePath.find("/R60m/") != std::string::npos) resolution = "60m";

                                bool addBand = false;
                                if (discoveredBandPaths.find(bandName) == discoveredBandPaths.end()) {
                                    addBand = true;
                                } else {
                                    std::string existingPath = discoveredBandPaths[bandName];
                                    std::string existingResolution = "";
                                    if (existingPath.find("/R10m/") != std::string::npos) existingResolution = "10m";
                                    else if (existingPath.find("/R20m/") != std::string::npos) existingResolution = "20m";
                                    else if (existingPath.find("/R60m/") != std::string::npos) existingResolution = "60m";

                                    if (resolution == "10m" && existingResolution != "10m") addBand = true;
                                    else if (resolution == "20m" && existingResolution == "60m") addBand = true;
                                }

                                if (addBand) {
                                    discoveredBandPaths[bandName] = filePath;
                                    DEBUG_LOG("Found band " << bandName << " (" << resolution << "): " << filePath);
                                } else {
                                    DEBUG_LOG("Skipping band " << bandName << " (" << resolution << ") as a better resolution already found.");
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

            int bHasNoData;
            double noDataValue = poBand->GetNoDataValue(&bHasNoData);
            if (bHasNoData) {
                std::cout << "    NoData Value: " << noDataValue << std::endl;
            }

            const char *pszProjection = poDS->GetProjectionRef();
            if (pszProjection != nullptr && std::string(pszProjection).length() > 0) {
                OGRSpatialReference oSRS;
                oSRS.SetFromUserInput(pszProjection);
                char *pszPrettyWKT = nullptr;
                oSRS.exportToPrettyWkt(&pszPrettyWKT, false);
                if (pszPrettyWKT) {
                    std::cout << "    Projection (CRS): " << pszPrettyWKT << std::endl;
                    CPLFree(pszPrettyWKT);
                }
            } else {
                std::cout << "    Projection (CRS): Not available or empty." << std::endl;
            }

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

//---

int main(int argc, char* argv[]) {
    // --- Parse the --debug flag first ---
    std::vector<std::string> remaining_args;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--debug") {
            g_isDebugMode = true;
        } else {
            remaining_args.push_back(arg);
        }
    }

    if (g_isDebugMode) {
        std::cout << "[INFO] Debug mode is ENABLED." << std::endl;
    }

    std::cout << "[INFO] Program started." << std::endl;

    // --- Command Line Argument Parsing (adjusted for --debug flag) ---
    // Now, check remaining_args count
    if (remaining_args.size() < 3) { // 3 expected: zip, output, operation
        std::cerr << "[FATAL] Usage: " << argv[0] << " [--debug] <zip_file_path> <output_tiff_path> <operation> [operation_args...]" << std::endl;
        std::cerr << "Operations and their arguments:" << std::endl;
        std::cerr << "  NDVI <NIR_band_name> <RED_band_name> (e.g., B08 B04)" << std::endl;
        std::cerr << "Example: " << argv[0] << " S2A_MSIL2A_...zip output_ndvi.tif NDVI B08 B04" << std::endl;
        std::cerr << "Example (Debug Mode): " << argv[0] << " --debug S2A_MSIL2A_...zip output_ndvi.tif NDVI B08 B04" << std::endl;
        return 1;
    }

    std::string zipFilePath = remaining_args[0];
    std::string outputTiffPath = remaining_args[1];
    std::string operationName = remaining_args[2];

    std::vector<std::string> operationArgs;
    for (size_t i = 3; i < remaining_args.size(); ++i) {
        operationArgs.push_back(remaining_args[i]);
    }
    DEBUG_LOG("Command-line arguments parsed.");

    S2Processor processor;

    // --- Dynamic Plugin Loading ---
    fs::path pluginDir = fs::current_path() / "plugins";

    if (fs::exists(pluginDir) && fs::is_directory(pluginDir)) {
        std::cout << "[INFO] Searching for operation plugins in: " << pluginDir.string() << std::endl;
        for (const auto& entry : fs::directory_iterator(pluginDir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                std::string pluginPath = entry.path().string();
                DEBUG_LOG("Found potential plugin: " << pluginPath);

                void* handle = dlopen(pluginPath.c_str(), RTLD_LAZY | RTLD_LOCAL);
                if (!handle) {
                    std::cerr << "[ERROR] Failed to load plugin '" << pluginPath << "': " << dlerror() << std::endl;
                    continue;
                }

                dlerror(); // Clear any existing error

                GetOperationNameFunc getOpName = (GetOperationNameFunc)dlsym(handle, "getOperationName");
                const char* dlsym_error_name = dlerror();
                if (dlsym_error_name) {
                    std::cerr << "[ERROR] Cannot find 'getOperationName' in plugin '" << pluginPath << "': " << dlsym_error_name << std::endl;
                    dlclose(handle);
                    continue;
                }

                std::string opName = getOpName();
                std::transform(opName.begin(), opName.end(), opName.begin(), ::toupper);

                if (processor.isOperationRegistered(opName)) {
                    std::cerr << "[WARNING] Operation '" << opName << "' from plugin '" << pluginPath << "' already registered. Skipping." << std::endl;
                    dlclose(handle);
                    continue;
                }

                CreateOperationInstanceFunc createOp = (CreateOperationInstanceFunc)dlsym(handle, "createOperationInstance");
                const char* dlsym_error_create = dlerror();
                if (dlsym_error_create) {
                    std::cerr << "[ERROR] Cannot find 'createOperationInstance' in plugin '" << pluginPath << "': " << dlsym_error_create << std::endl;
                    dlclose(handle);
                    continue;
                }

                std::unique_ptr<IOperation> opInstance = createOp();
                if (opInstance) {
                    processor.registerOperation(std::move(opInstance));
                    processor.addPluginHandle(opName, handle);
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

    bool success = processor.process(zipFilePath, outputTiffPath, operationName, operationArgs);

    std::cout << "[INFO] Program finished with status: " << (success ? "SUCCESS" : "FAILURE") << std::endl;

    return success ? 0 : 1;
}

#include <Camera/Daheng.h>
#include <Utils/AsyncWriter.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include <memory>

namespace {
    // GXInitLib/GXCloseLib are process wide: initialise the library for the first
    // camera and close it only when the last camera is destroyed.
    std::mutex gxLibMutex;
    int gxLibUsers = 0;

    bool acquireGxLib()
    {
        std::lock_guard<std::mutex> lock(gxLibMutex);
        if (gxLibUsers == 0) {
            GX_STATUS emStatus = GXInitLib();
            if (emStatus != GX_STATUS_SUCCESS) {
                std::cout << "<Error failed to init the API>" << std::endl;
                GetErrorString(emStatus);
                return false;
            }
        }
        gxLibUsers++;
        return true;
    }

    void releaseGxLib()
    {
        std::lock_guard<std::mutex> lock(gxLibMutex);
        if (gxLibUsers > 0 && --gxLibUsers == 0) {
            GXCloseLib();
        }
    }

    bool nodeIsAccessible(GX_DEV_HANDLE hDevice, const char* node)
    {
        GX_NODE_ACCESS_MODE accessMode;
        GX_STATUS emStatus = GXGetNodeAccessMode(hDevice, node, &accessMode);
        if (emStatus != GX_STATUS_SUCCESS) {
            GetErrorString(emStatus);
            return false;
        }
        return accessMode == GX_NODE_ACCESS_MODE_RO
            || accessMode == GX_NODE_ACCESS_MODE_WO
            || accessMode == GX_NODE_ACCESS_MODE_RW;
    }
}


Daheng::Daheng(const std::string& name, const std::string& ip, int rFlag, std::string calibFileName):
    name(name),
    ip(ip),
    rFlag(rFlag)
{
    // Read in the calibrion matrix and intrinsics
    // loadCalib(calibFileName, name, K, tLidarCam);

    // Initialize the camera
    if (!acquireGxLib()) {
        return;
    }

    //Opens the device via an IP address.
    std::cout << "Opening device (" << ip << ")" << std::endl;
    GX_OPEN_PARAM stOpenParam;
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_IP;
    stOpenParam.pszContent = const_cast<char*>(this->ip.c_str());
    GX_STATUS emStatus = GXOpenDevice(&stOpenParam, &hDevice);
    if (emStatus != GX_STATUS_SUCCESS) {
        std::cout << "<Error when opening device (" << ip << ")>" << std::endl;
        GetErrorString(emStatus);
        hDevice = NULL;
        return;
    }

    // The camera stays reserved for this process until the heartbeat times out. Keep it short,
    // so a crashed or killed program does not lock the camera for minutes (default 5 min).
    emStatus = GXSetIntValue(hDevice, "GevHeartbeatTimeout", HEARTBEAT_TIMEOUT_MS);
    if (emStatus != GX_STATUS_SUCCESS) {
        GetErrorString(emStatus);
    }
}

Daheng::~Daheng()
{
    stop();
    releaseGxLib();
}

void Daheng::stop()
{
    //Stop Acquisition thread
    bAcquisitionFlag.store(false);
    if (acquisitionThread.joinable()) {
        acquisitionThread.join();
    }

    if (hDevice == NULL) {
        return;
    }

    std::cout << "Closing device (" << ip << ")." << std::endl;
    //Close device
    GX_STATUS emStatus = GXCloseDevice(hDevice);
    if (emStatus != GX_STATUS_SUCCESS) {
        GetErrorString(emStatus);
    }
    hDevice = NULL;
}

void Daheng::start()
{
    if (hDevice == NULL) {
        std::cout << "Initialization device (" << ip << ") failed." << std::endl;
        return;
    }
    if (acquisitionThread.joinable()) {
        std::cout << "Device (" << ip << ") is already acquiring." << std::endl;
        return;
    }

    // Set the flag before the thread starts, so a stop() right after start() cannot be missed
    bAcquisitionFlag.store(true);
    acquisitionThread = std::thread(&Daheng::acquisitionLoop, this);
}

void Daheng::resetTimestamp()
{
    if (hDevice == NULL) {
        return;
    }

    // reset timestamp
    GXSendCommand(hDevice, GX_COMMAND_TIMESTAMP_RESET);
    auto now = std::chrono::system_clock::now();
    tsRef = std::chrono::time_point_cast<std::chrono::microseconds>(now).time_since_epoch().count();
}

const std::string& Daheng::getName() const
{
    return name;
}

const std::string& Daheng::getIp() const
{
    return ip;
}

bool Daheng::isOpen() const
{
    return hDevice != NULL;
}

bool Daheng::isAcquiring() const
{
    return bAcquiring.load();
}

double Daheng::getFps() const
{
    return fps.load();
}

uint64_t Daheng::getFrameId() const
{
    return frameId.load();
}

uint64_t Daheng::getFrameSequence() const
{
    return frameSequence.load();
}

uint64_t Daheng::getTimeStamp() const
{
    return ts.load() + tsRef;
}

void Daheng::getMetaData(CameraMetaData& cameraMetaData) const
{
    cameraMetaData.frameId = getFrameId();
    cameraMetaData.ts = getTimeStamp();
    cameraMetaData.fps = getFps();
    cameraMetaData.ip = ip;
}

void Daheng::getMetaData(const Frame& frame, CameraMetaData& cameraMetaData) const
{
    cameraMetaData.frameId = frame.frameId;
    cameraMetaData.ts = frame.ts;
    cameraMetaData.fps = getFps();
    cameraMetaData.ip = ip;
}

bool Daheng::getFrameNear(uint64_t timeStamp, Frame& frame)
{
    std::lock_guard<std::mutex> lock(imageMutex);
    auto distance = [timeStamp](const Frame& f) { return f.ts > timeStamp ? f.ts - timeStamp : timeStamp - f.ts; };
    const Frame* best = nullptr;
    for (const Frame& f : recentFrames) {
        if (best == nullptr || distance(f) < distance(*best)) {
            best = &f;
        }
    }
    if (best == nullptr) {
        return false;
    }
    frame = *best;
    return true;
}

int Daheng::getRotation() const
{
    switch (rFlag.load()) {
        case cv::ROTATE_90_CLOCKWISE: return 90;
        case cv::ROTATE_180: return 180;
        case cv::ROTATE_90_COUNTERCLOCKWISE: return 270;
        default: return 0;
    }
}

void Daheng::setRotation(int degrees)
{
    switch (((degrees % 360) + 360) % 360) {
        case 90: rFlag.store(cv::ROTATE_90_CLOCKWISE); break;
        case 180: rFlag.store(cv::ROTATE_180); break;
        case 270: rFlag.store(cv::ROTATE_90_COUNTERCLOCKWISE); break;
        default: rFlag.store(-1); break;
    }
}

//-------------------------------------------------
/**
\brief Acquisition thread function
*/
//-------------------------------------------------
void Daheng::acquisitionLoop()
{
    std::cout << "<Acquisition thread start! (" << ip << ")>" << std::endl;

    frameId.store(0);
    frameSequence.store(0);
    fps.store(0.0);
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Pre acquisition
    //Set buffer quantity of acquisition queue
    emStatus = GXSetAcqusitionBufferNumber(hDevice, ACQ_BUFFER_NUM);
    if (emStatus != GX_STATUS_SUCCESS) {
        GetErrorString(emStatus);
    }

    if (nodeIsAccessible(hDevice, "StreamTransferSize")) {
        //Set size of data transfer block
        GXSetIntValue(hDevice, "StreamTransferSize", ACQ_TRANSFER_SIZE);
    }

    if (nodeIsAccessible(hDevice, "StreamTransferNumberUrb")) {
        //Set qty. of data transfer block
        GXSetIntValue(hDevice, "StreamTransferNumberUrb", ACQ_TRANSFER_NUMBER_URB);
    }

    // Get color filter
    int64_t i64ColorFilter = GX_COLOR_FILTER_NONE;      // Color filter of device
    GX_ENUM_VALUE emValue;
    emStatus = GXGetEnumValue(hDevice, "PixelColorFilter", &emValue);
    if (emStatus == GX_STATUS_SUCCESS) {
        i64ColorFilter = emValue.stCurValue.nCurValue;
    } else {
        GetErrorString(emStatus);
    }

    //Start acquisition
    emStatus = GXStreamOn(hDevice);
    if (emStatus != GX_STATUS_SUCCESS) {
        GetErrorString(emStatus);
        bAcquisitionFlag.store(false);
        std::cout << "<Acquisition thread Exit! (" << ip << ")>" << std::endl;
        return;
    }
    bAcquiring.store(true);

    // Memory for RAW8toRGB24, (re)allocated when the frame size changes
    std::vector<unsigned char> rgbBuffer;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    // Timing for fps
    auto tic = std::chrono::steady_clock::now();

    while (bAcquisitionFlag.load())
    {
        // Get a frame from Queue
        emStatus = GXDQBuf(hDevice, &pFrameBuffer, 1000);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            GetErrorString(emStatus);
            break;
        }

        if (pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            std::cout << "<Abnormal Acquisition (" << ip << "): Exception code: " << pFrameBuffer->nStatus << ">" << std::endl;
        }
        else
        {
            const int width = pFrameBuffer->nWidth;
            const int height = pFrameBuffer->nHeight;
            rgbBuffer.resize(static_cast<size_t>(width) * height * 3);

            // Convert to the RGB image
            VxInt32 emDXStatus = DxRaw8toRGB24(static_cast<unsigned char*>(pFrameBuffer->pImgBuf),rgbBuffer.data(), width, height,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                std::cout << "DxRaw8toRGB24 Failed, Error Code: " << emDXStatus << std::endl;
                GXQBuf(hDevice, pFrameBuffer);
                break;
            }

            cv::Mat imageRGB(height, width, CV_8UC3, rgbBuffer.data());
            cv::Mat frameImage;
            const int rotateFlag = rFlag.load();
            if (rotateFlag >= 0) {
                cv::rotate(imageRGB, frameImage, static_cast<cv::RotateFlags>(rotateFlag));
            } else {
                frameImage = imageRGB.clone();
            }

            // Update frame info
            const uint64_t newFrameId = pFrameBuffer->nFrameID;
            const bool isNewFrame = frameId.load() != newFrameId;
            frameId.store(newFrameId);

            if (isNewFrame) {
                // Update timestamp: camera counts per 8 ns -> microseconds
                ts.store(static_cast<uint64_t>(pFrameBuffer->nTimestamp * 8 / 1000));

                // Update fps
                auto toc = std::chrono::steady_clock::now();
                std::chrono::duration<double> duration = toc - tic;
                if (duration.count() > 0.0) {
                    fps.store(1.0 / duration.count());
                }
                tic = toc;
            }

            {
                std::lock_guard<std::mutex> lock(imageMutex);
                image = frameImage;
                recentFrames.push_back({frameImage, newFrameId, getTimeStamp(), frameSequence.load() + 1});
                if (recentFrames.size() > RECENT_FRAME_NUM) {
                    recentFrames.pop_front();
                }
            }

            // Increment last: readers use the sequence to detect a new frame
            frameSequence.fetch_add(1);
        }

        // Otherwise image is not consumed
        emStatus = GXQBuf(hDevice, pFrameBuffer);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            break;
        }
    }
    std::cout << "<Acquisition thread Exit! (" << ip << ")>" << std::endl;
    bAcquiring.store(false);
    fps.store(0.0);

    //Stop acquisition
    emStatus = GXStreamOff(hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
    }
}


std::shared_ptr<cv::Mat> Daheng::getCvImage()
{
    std::shared_ptr<cv::Mat> imageBGR = nullptr;

    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if (!image.empty()) {
            imageBGR = std::make_shared<cv::Mat>();
            cv::cvtColor(image, *imageBGR, cv::COLOR_RGB2BGR);
        }
    }

    return imageBGR;
}

std::shared_ptr<QImage> Daheng::getQtImage()
{
    std::lock_guard<std::mutex> lock(imageMutex);
    if (image.empty()) {
        return nullptr;
    }

    // Wrap the cv::Mat with its real row stride (QImage rows are 4-byte aligned) and deep copy
    QImage view(image.data, image.cols, image.rows, static_cast<qsizetype>(image.step), QImage::Format_RGB888);
    return std::make_shared<QImage>(view.copy());
}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS emErrorStatus)
{
    size_t size = 0;

    // Get length of error description
    GX_STATUS emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        std::cout << "<Error when calling GXGetLastError>" << std::endl;
        return;
    }

    // Get error description
    std::string errorInfo(size, '\0');
    emStatus = GXGetLastError(&emErrorStatus, errorInfo.data(), &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        std::cout << "<Error when calling GXGetLastError>" << std::endl;
        return;
    }
    std::cout << errorInfo.c_str() << std::endl;
}

bool Daheng::saveFrame(const Frame& frame, const std::string& fileName)
{
    if (frame.image.empty()) {
        return false;
    }

    // Convert and save on the writer pool; the frame image is shared, never modified
    cv::Mat imageRGB = frame.image;
    return AsyncWriter::instance().enqueue([fileName, imageRGB]() {
        cv::Mat imageBGR;
        cv::cvtColor(imageRGB, imageBGR, cv::COLOR_RGB2BGR);
        if (!cv::imwrite(fileName, imageBGR)) {
            std::cerr << "Failed to write " << fileName << std::endl;
        }
    });
}

bool Daheng::saveIm(const std::string& fileName)
{
    auto cvIm = getCvImage();
    if (!cvIm || cvIm->empty()) {
        std::cout << "No image available to save." << std::endl;
        return false;
    }

    // Save the image on the writer pool
    return AsyncWriter::instance().enqueue([fileName, cvIm]() {
        if (!cv::imwrite(fileName, *cvIm)) {
            std::cerr << "Failed to write " << fileName << std::endl;
        }
    });
}

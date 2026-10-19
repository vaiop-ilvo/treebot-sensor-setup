#pragma once
#include <GxIAPI.h>
#include <DxImageProc.h>
#include <string>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>

#include <QImage>

#include <Utils/MetaData.hpp>

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define HEARTBEAT_TIMEOUT_MS    3000            ///< Camera is released this long after the program stops
#define RECENT_FRAME_NUM        8               ///< Frames kept to match against a trigger timestamp

//Get description of error
void GetErrorString(GX_STATUS);

class Daheng
{
public:
    // One received frame. The image is never modified after it is stored, so copies share the pixels.
    struct Frame {
        cv::Mat image;          // RGB, rotated
        uint64_t frameId = 0;
        uint64_t ts = 0;        // Host time in microseconds since epoch
        uint64_t sequence = 0;  // Value of getFrameSequence() for this frame
    };

private:
    std::string name;
    std::string ip;
    std::atomic<int> rFlag;                             // cv::RotateFlags, -1 = no rotation
    GX_DEV_HANDLE hDevice = NULL;                       // Device handle

    std::thread acquisitionThread;
    std::atomic<uint64_t> frameId{0};
    std::atomic<uint64_t> frameSequence{0};
    std::atomic<double> fps{0.0};
    std::atomic<uint64_t> ts{0};
    std::atomic<uint64_t> tsRef{0};
    std::atomic<bool> bAcquisitionFlag{false};          // Flag to control acquisition loop
    std::atomic<bool> bAcquiring{false};                // True while frames are streaming

    cv::Mat image;
    std::deque<Frame> recentFrames;                     // Oldest first, at most RECENT_FRAME_NUM
    std::mutex imageMutex;

    //Acquisition thread function
    void acquisitionLoop();
public:
    Daheng(const std::string& name, const std::string& ip, int rFlag=-1, std::string calibFileName="");
    ~Daheng();

    Daheng(const Daheng&) = delete;
    Daheng& operator=(const Daheng&) = delete;

    std::shared_ptr<cv::Mat> getCvImage();
    std::shared_ptr<QImage> getQtImage();

    const std::string& getName() const;
    const std::string& getIp() const;
    bool isOpen() const;
    bool isAcquiring() const;
    double getFps() const;
    uint64_t getFrameId() const;
    uint64_t getFrameSequence() const;
    uint64_t getTimeStamp() const;
    void getMetaData(CameraMetaData&) const;
    void getMetaData(const Frame& frame, CameraMetaData&) const;

    // The recent frame with the timestamp closest to ts (host time, µs). Returns false when there is no frame.
    bool getFrameNear(uint64_t ts, Frame& frame);

    // Rotation applied to new frames (and so to the saved images), in degrees clockwise: 0, 90, 180 or 270
    int getRotation() const;
    void setRotation(int degrees);

    void start();
    void stop();
    void resetTimestamp();

    // Queues the latest image to be written to disk. Returns false when there is no image or the write queue is full.
    bool saveIm(const std::string& fileName);
    // Queues the given frame to be written to disk. Returns false when the write queue is full.
    static bool saveFrame(const Frame& frame, const std::string& fileName);
};

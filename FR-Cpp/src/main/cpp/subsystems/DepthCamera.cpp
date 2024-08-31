#include "subsystems/DepthCamera.h"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <bitset>

std::ostream &operator<<(std::ostream &os, const OBFrameType frameType) {
    if(frameType == OB_FRAME_DEPTH) {
        os << "OB_FRAME_DEPTH";
    }
    else {
        os << (int)frameType;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, std::shared_ptr<ob::DepthFrame> frame) {
    uint16_t *data       = (uint16_t *)frame->data();
    auto      width      = frame->width();
    auto      height     = frame->height();
    auto      scale      = frame->getValueScale();
    uint16_t  pixelValue = *(data + width * height / 2 + width / 2);
    os << "FrameType: " << frame->type() << ", index: " << frame->index() << ", width: " << width << ", height: " << height << ", format: " << frame->format()
       << ", timeStampUs: " << frame->timeStampUs() << "us, centerDepth: " << (float)pixelValue * scale << "mm";
    return os;
}

DepthCamera::DepthCamera() {
    setupOrbbecSDK();
}

DepthCamera::~DepthCamera() {
    teardownOrbbecSDK();
}

void DepthCamera::Periodic() {
    // std::cout << "Printing here would flood the console output window" << std::endl;
}

void DepthCamera::setupOrbbecSDK() {
    // Print the sdk version number, the sdk version number is divided into major version number, minor version number and revision number
    std::cout << "SDK version: " << ob::Version::getMajor() << "." << ob::Version::getMinor() << "." << ob::Version::getPatch() << std::endl;
    // Print sdk stage version
    std::cout << "SDK stage version: " << ob::Version::getStageVersion() << std::endl;
    obContex_ = std::make_shared<ob::Context>();
    obContex_->setDeviceChangedCallback([&](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList) {
        handleDeviceDisconnected(removedList);
        handleDeviceConnected(addedList);
    });
    handleDeviceConnected(obContex_->queryDeviceList());
}

void DepthCamera::teardownOrbbecSDK() {
    for(const auto &pair: pipelineHolderMap_) {
        stopStream(pair.second);
        std::cout << "teardownOrbbecSDK: stop streams for device: " << pair.second->deviceInfo << std::endl;
    }
    pipelineHolderMap_.clear();
    obContex_.reset();
}

// Device connection callback
void DepthCamera::handleDeviceConnected(std::shared_ptr<ob::DeviceList> connectList) {
    std::lock_guard<std::recursive_mutex> lk(pipelineHolderMutex_);
    const auto                            deviceCount = connectList->deviceCount();
    std::cout << "Device connect, deviceCount: " << deviceCount << std::endl;
    for(uint32_t i = 0; i < deviceCount; i++) {
        auto uid = std::string(connectList->uid(i));
        auto itr = pipelineHolderMap_.find(uid);
        if(itr != pipelineHolderMap_.end()) {
            std::cout << "Device connected. device already exit. " << itr->second->deviceInfo << std::endl;
        }
        else {
            auto device     = connectList->getDevice(i);
            auto deviceInfo = device->getDeviceInfo();

            std::shared_ptr<ob::Pipeline>   pipeline(new ob::Pipeline(device));
            std::shared_ptr<PipelineHolder> holder(new PipelineHolder{ pipeline, deviceInfo, false });
            pipelineHolderMap_.insert({ uid, holder });

            std::cout << "Device connected. " << deviceInfo << std::endl;
            startStream(holder);
        }
    }
}

// Device disconnect callback
void DepthCamera::handleDeviceDisconnected(std::shared_ptr<ob::DeviceList> disconnectList) {
    std::lock_guard<std::recursive_mutex> lk(pipelineHolderMutex_);
    const auto                            deviceCount = disconnectList->deviceCount();
    std::cout << "Device disconnect, deviceCount: " << deviceCount << std::endl;
    for(uint32_t i = 0; i < deviceCount; i++) {
        auto uid = std::string(disconnectList->uid(i));
        auto itr = pipelineHolderMap_.find(uid);
        if(itr != pipelineHolderMap_.end()) {
            auto deviceInfo = itr->second->deviceInfo;
            stopStream(itr->second);
            pipelineHolderMap_.erase(uid);
            std::cout << "Device disconnected. " << deviceInfo << std::endl;
        }
        else {
            std::cout << "Device disconnect, unresolve deviceUid: " << uid << std::endl;
        }
    }
}

void DepthCamera::rebootDevices() {
    std::cout << "Reboot devices, count: " << pipelineHolderMap_.size() << std::endl;
    for(auto itr = pipelineHolderMap_.begin(); itr != pipelineHolderMap_.end(); itr++) {
        try {
            std::cout << "Reboot device, " << itr->second->deviceInfo << std::endl;
            stopStream(itr->second);
            itr->second->pipeline->getDevice()->reboot();
        }
        catch(ob::Error &e) {
            std::cerr << "Reboot device failed. function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
                      << "\ntype:" << e.getExceptionType() << std::endl;
        }
    }
}

void DepthCamera::processFrameset(std::string deviceSN, std::shared_ptr<FramePrintInfo> printInfo, std::shared_ptr<ob::FrameSet> frameSet) {
    // Get the depth data frame
    auto depthFrame = frameSet->depthFrame();
    if(depthFrame) {
        printInfo->depthCount++;
        if(printInfo->depthCount == 15) {
            std::cout << "=====Depth Frame Info====== SN: " << std::string(deviceSN) << ", " << depthFrame << std::endl;
            printInfo->depthCount = 0;
        }
    }
}

void DepthCamera::startStream(std::shared_ptr<PipelineHolder> holder) {
    std::shared_ptr<FramePrintInfo> printInfo(new FramePrintInfo{});
    std::string                     deviceSN = std::string(holder->deviceInfo->serialNumber());
    ob::FrameSetCallback callback = [&, deviceSN, printInfo](std::shared_ptr<ob::FrameSet> frameSet) { processFrameset(deviceSN, printInfo, frameSet); };

    // Start video stream according to the stream profile of the configuration file.If there is no configuration file, the first stream profile will be used.
    try {
        std::cout << "startStream " << holder->deviceInfo << std::endl;
        // Create the config for starting streams.
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
        // Config for depth stream.
        try {
            auto depthProfileList = holder->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            // For example: You can also get StreamProfile this way.
            // for (int i = 0, N = depthProfileList->count(); i < N; i++) {
            //     auto vsp = depthProfileList->getProfile(i)->as<ob::VideoStreamProfile>();
            //     std::cout << vsp << std::endl;
            // }

            // Filter out the StreamProfile you want based on specified arguments.
            // auto depthProfile =  depthProfileList->getVideoStreamProfile(640, 480, Format.Y16, 30);
            auto depthProfile = depthProfileList->getProfile(0)->as<ob::VideoStreamProfile>();
            std::cout << "DepthProfile: " << depthProfile << std::endl;
            config->enableStream(depthProfile);
        }
        catch(...) {
            std::cerr << "Config for depth profile failed!" << std::endl;
        }

        holder->pipeline->start(config, callback);
        holder->isStarted = true;
    }
    catch(...) {
        std::cout << "Pipeline start failed!" << std::endl;
        holder->isStarted = false;
    }
}

void DepthCamera::stopStream(std::shared_ptr<PipelineHolder> holder) {
    if(!holder->isStarted) {
        return;
    }

    try {
        std::cout << "stopStream " << holder->deviceInfo << std::endl;
        holder->isStarted = false;
        holder->pipeline->stop();
    }
    catch(ob::Error &e) {
        std::cerr << "stopStream failed., function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
                  << "\ntype:" << e.getExceptionType() << std::endl;
    }
}

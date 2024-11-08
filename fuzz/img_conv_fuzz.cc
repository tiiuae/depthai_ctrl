#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <fuzzer/FuzzedDataProvider.h>

#include "depthai-shared/common/CameraInfo.hpp"
#include <depthai/depthai.hpp>
#include <depthai_ctrl/ImageConverter.hpp>
#include <depthai_ctrl/ImgDetectionConverter.hpp>
#include <depthai_ctrl/ImuConverter.hpp>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/hal/interface.h>
#include <ratio>
#include <tuple>
#include <vector>

// clang-format off
static const char* knownErrors[] = {
  "Division by 0",
  "imdecode",
    "depthOut empty",
  nullptr
};
// clang-format on

const std::vector<std::string> ENCODINGS = {
    "rgb8",  "rgba8",  "rgb16",       "rgba16",      "bgr8",        "bgra8",       "bgr16", "bgra16",
    "mono8", "mono16", "8UC1",        "8UC3",        "16UC1",       "16UC3",       "32FC1", "32FC3",
    "64FC1", "64FC3",  "bayer_rggb8", "bayer_bggr8", "bayer_gbrg8", "bayer_grbg8", "yuv422"};

sensor_msgs::msg::Image generateFuzzedImage(FuzzedDataProvider &fdp)
{
    sensor_msgs::msg::Image img;

    // Generate header
    img.header.stamp = rclcpp::Time(fdp.ConsumeIntegral<uint32_t>()); // seconds
    img.header.frame_id = fdp.ConsumeRandomLengthString(16);          // frame_id

    // Generate dimensions (with reasonable bounds to avoid excessive memory
    // usage)
    img.height = fdp.ConsumeIntegralInRange<uint32_t>(1, 2048);
    img.width = fdp.ConsumeIntegralInRange<uint32_t>(1, 2048);

    // Pick a random encoding from common types
    img.encoding = ENCODINGS[fdp.ConsumeIntegralInRange<size_t>(0, ENCODINGS.size() - 1)];

    // Generate endianness
    img.is_bigendian = fdp.ConsumeBool();

    // Calculate step based on encoding and width
    // This is a simplified calculation - you might want to make it more precise
    // based on your specific encodings
    size_t bytes_per_pixel;
    if (img.encoding.find("8") != std::string::npos)
    {
        bytes_per_pixel = 1;
    }
    else if (img.encoding.find("16") != std::string::npos)
    {
        bytes_per_pixel = 2;
    }
    else if (img.encoding.find("32") != std::string::npos)
    {
        bytes_per_pixel = 4;
    }
    else if (img.encoding.find("64") != std::string::npos)
    {
        bytes_per_pixel = 8;
    }
    else
    {
        bytes_per_pixel = 1; // default
    }

    size_t channels;
    if (img.encoding.find("C3") != std::string::npos || img.encoding.find("rgb") != std::string::npos ||
        img.encoding.find("bgr") != std::string::npos)
    {
        channels = 3;
    }
    else if (img.encoding.find("rgba") != std::string::npos || img.encoding.find("bgra") != std::string::npos)
    {
        channels = 4;
    }
    else
    {
        channels = 1; // default to single channel
    }

    img.step = img.width * bytes_per_pixel * channels;

    // Generate image data
    size_t data_size = img.step * img.height;
    img.data.resize(data_size);

    // Fill with random data
    std::vector<uint8_t> random_data = fdp.ConsumeBytes<uint8_t>(data_size);
    if (!random_data.empty())
    {
        std::copy(random_data.begin(), random_data.end(), img.data.begin());
    }

    return img;
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size)
{
    FuzzedDataProvider fdp(Data, Size);

    auto inData = std::make_shared<dai::ImgFrame>();

    unsigned int width = fdp.ConsumeIntegral<int64_t>();
    unsigned int height = fdp.ConsumeIntegral<int64_t>();
    unsigned int channelSize = fdp.ConsumeIntegral<int64_t>();
    size_t imageSize = width * height * channelSize;
    std::vector<uint8_t> frameData = fdp.ConsumeBytes<uint8_t>(imageSize);

    // Set the generated data to the ImgFrame object
    inData->setWidth(width);
    inData->setHeight(height);
    inData->setData(frameData);

    // NOTE: Check if inData is populated
    // Otherwise we're triggering an assert:
    //     what():  OpenCV(4.6.0) ./modules/imgcodecs/src/loadsave.cpp:816: error:
    //     (-215:Assertion failed) !buf.empty() in function 'imdecode_'
    // if (inData->getData().empty())
    //{
    //    return 1;
    //}

    // NOTE: Another sanity check for:
    //     what():  OpenCV(4.6.0)
    //     /usr/include/opencv4/opencv2/core/utility.hpp:627: error:
    //     (-215:Assertion failed) !empty() in function 'forEach_impl'
    // if (width <= 0 || height <= 0)
    //{
    //    return 1;
    //}

    std::deque<dai::ros::ImageMsgs::Image> outImageMsgs;
    dai::RawImgFrame::Type type;
    // NOTE: Some are throwing   what():  Converted type not supported!
    switch (fdp.ConsumeIntegralInRange(0, 2))
    {
    case 0:
        type = dai::RawImgFrame::Type::RAW8;
        break;
    case 1:
        type = dai::RawImgFrame::Type::GRAY8;
        break;
    case 2:
        type = dai::RawImgFrame::Type::BGR888i;
        break;
    case 3:
        type = dai::RawImgFrame::Type::RGBA8888;
        break;
    case 4:
        type = dai::RawImgFrame::Type::RGB888i;
        break;
    case 5:
        type = dai::RawImgFrame::Type::RAW16;
        break;
    case 6:
        type = dai::RawImgFrame::Type::YUV420p;
        break;
    default:
        type = dai::RawImgFrame::Type::RAW8;
    }
    inData->setType(type);

    inData->setCategory(fdp.ConsumeIntegral<uint64_t>());
    inData->setInstanceNum(fdp.ConsumeIntegral<uint64_t>());
    inData->setSequenceNum(fdp.ConsumeIntegral<uint64_t>());

    sensor_msgs::msg::CameraInfo info;
    std::string frameName = fdp.ConsumeRandomLengthString();
    bool interleaved = fdp.ConsumeBool();

    dai::ros::ImageConverter imageConverter(frameName, interleaved);

    try
    {
        switch (fdp.ConsumeIntegralInRange(0, 4))
        {
        case 0:
            imageConverter.toRosMsgFromBitStream(inData, outImageMsgs, type, info);
            break;
        case 1:
            imageConverter.toRosMsg(inData, outImageMsgs);
            break;
        case 2:
            imageConverter.toRosMsgPtr(inData);
            break;
        case 3: {
            auto fuzzed_image = generateFuzzedImage(fdp);
            dai::ImgFrame outData;
            imageConverter.toDaiMsg(fuzzed_image, outData);
            break;
        }
        case 4: {
            auto fuzzed_image2 = generateFuzzedImage(fdp);
            imageConverter.rosMsgtoCvMat(fuzzed_image2);
            break;
        }
        case 5: {
            // ImageMsgs::CameraInfo
            // ImageConverter::calibrationToCameraInfo(dai::CalibrationHandler
            // calibHandler,
            //                                                     dai::CameraBoardSocket
            //                                                     cameraId, int
            //                                                     width, int height,
            //                                                     Point2f
            //                                                     topLeftPixelId,
            //                                                     Point2f
            //                                                     bottomRightPixelId)
            break;
        }
        }
    }
    // TODO: calibrationToCameraInfo
    catch (const std::runtime_error &e)
    {
        // Handle runtime errors separately
        const char *errorMsg = e.what();
        for (const char **known = knownErrors; *known != nullptr; ++known)
        {
            if (strstr(errorMsg, *known) != nullptr)
            {

                inData.reset();
                outImageMsgs.clear(); // Clear before exit

                return 0;
            }
        }
        inData.reset();
        outImageMsgs.clear(); // Clear before exit

        throw;
    }

    // Check if the output image messages are valid
    // for (const auto &msg : outImageMsgs)
    //{
    //    if (msg.width <= 0 || msg.height <= 0 || msg.encoding.empty())
    //    {
    //        throw(std::runtime_error("Received invalid image back"));
    //        return -1;
    //    }
    //}
    inData.reset();
    outImageMsgs.clear(); // Clear before exit

    return 0;
}

//+ 1. assertion trigger in opencv -> crash-d22580....ec5 (!empty())
//+ 2. assertion trigger in opencv -> crash-c064...b50 (validateInputImageSize)

// ? 3. OOB-R in ImageConverter::toDaiMsg:281 -> crash-ce84...5e15

// + 4. deadly signal what stoi: in ImageConverter:261 (stoi) ->
// crash-7f64...bf09 (invalid argument)
//   -> in particular the call to stoi(encoding_info[1])
//   -> RCA: The while loop before just adds a single element in slot [0]
//   -> encoding_info[1] is really an OOB access

// + 5. Another SEGV (OOB) right befor the while-loop revEncodingIter->second in
// ImageConverter.cpp:260
//   -> crash-22f7....afa67

// + 6. Address Sanitizer FPE - Division by 0
// ->         outImageMsg.step = inData->getData().size() / inData->getHeight();
// (crash-21d0....8f48)

// + 7. Deadly signal UBSAN memcpy 2nd argument 0 (crash-8deac....f8202)

// 8. SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior
// /usr/bin/../lib/gcc/x86_64-linux-gnu/13/../../../../include/c++/13/bits/basic_string.h:551:48
// terminate called after throwing an instance of 'std::length_error'
//   what():  basic_string::_M_create
// toRosMsgPtr(std::shared_ptr<dai::ImgFrame>) ImageConverter.cpp:316:16
//  ---> crash-b8b....9dec

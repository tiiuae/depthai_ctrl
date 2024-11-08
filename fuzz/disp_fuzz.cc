#include <cv_bridge/cv_bridge.hpp>
#include <fuzzer/FuzzedDataProvider.h>

#include "depthai-shared/common/CameraInfo.hpp"
#include <depthai/depthai.hpp>
#include <depthai_ctrl/DisparityConverter.hpp>
#include <depthai_ctrl/ImageConverter.hpp>
#include <depthai_ctrl/ImgDetectionConverter.hpp>
#include <depthai_ctrl/ImuConverter.hpp>
#include <deque>
#include <memory>
#include <ratio>
#include <tuple>

dai::RawImgFrame::Type PickFrameType(FuzzedDataProvider &fdp)
{
    return fdp.PickValueInArray<dai::RawImgFrame::Type>(
        {dai::RawImgFrame::Type::YUV422i,       dai::RawImgFrame::Type::YUV444p,
         dai::RawImgFrame::Type::YUV420p,       dai::RawImgFrame::Type::YUV422p,
         dai::RawImgFrame::Type::YUV400p,       dai::RawImgFrame::Type::RGBA8888,
         dai::RawImgFrame::Type::RGB161616,     dai::RawImgFrame::Type::RGB888p,
         dai::RawImgFrame::Type::BGR888p,       dai::RawImgFrame::Type::RGB888i,
         dai::RawImgFrame::Type::BGR888i,       dai::RawImgFrame::Type::RGBF16F16F16p,
         dai::RawImgFrame::Type::BGRF16F16F16p, dai::RawImgFrame::Type::RGBF16F16F16i,
         dai::RawImgFrame::Type::BGRF16F16F16i, dai::RawImgFrame::Type::GRAY8,
         dai::RawImgFrame::Type::GRAYF16,       dai::RawImgFrame::Type::LUT2,
         dai::RawImgFrame::Type::LUT4,          dai::RawImgFrame::Type::LUT16,
         dai::RawImgFrame::Type::RAW16,         dai::RawImgFrame::Type::RAW14,
         dai::RawImgFrame::Type::RAW12,         dai::RawImgFrame::Type::RAW10,
         dai::RawImgFrame::Type::RAW8,          dai::RawImgFrame::Type::PACK10,
         dai::RawImgFrame::Type::PACK12,        dai::RawImgFrame::Type::YUV444i,
         dai::RawImgFrame::Type::NV12,          dai::RawImgFrame::Type::NV21,
         dai::RawImgFrame::Type::BITSTREAM,     dai::RawImgFrame::Type::HDR,
         dai::RawImgFrame::Type::NONE});
}

std::chrono::time_point<std::chrono::steady_clock> GenerateFuzzTimestamp(FuzzedDataProvider &fdp)
{
    using namespace std::chrono;
    auto now = steady_clock::now();
    auto now_ts = duration_cast<nanoseconds>(now.time_since_epoch()).count();

    // Range: ±1 day in nanoseconds
    constexpr uint64_t ONE_DAY_NS = 86400ULL * 1000000000ULL;

    uint64_t fuzz_ns = fdp.ConsumeIntegralInRange<uint64_t>(now_ts - ONE_DAY_NS, // min: now - 1 day
                                                            now_ts + ONE_DAY_NS  // max: now + 1 day
    );

    // Convert back to time_point by creating from epoch
    return steady_clock::time_point(nanoseconds(fuzz_ns));
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{

    if (size < 64)
    {
        return -1;
    }

    FuzzedDataProvider fdp(data, size);

    std::string frameName = fdp.ConsumeRandomLengthString();
    float focalLength = fdp.ConsumeFloatingPoint<float>();
    float baseline = fdp.ConsumeFloatingPoint<float>();
    float minDepth = fdp.ConsumeFloatingPoint<float>();
    float maxDepth = fdp.ConsumeFloatingPoint<float>();

    // printf("FOO\n");

    try
    {
        dai::ros::DisparityConverter converter(frameName, focalLength, baseline, minDepth, maxDepth);
        // dai::ros::DisparityConverter converter("asdf", 1.2);
        // printf("BAR\n");

        auto category = fdp.ConsumeIntegral<int>();
        auto height = fdp.ConsumeIntegral<int>();
        auto instanceNum = fdp.ConsumeIntegral<int>();
        auto seqNum = fdp.ConsumeIntegral<int>();
        auto width = fdp.ConsumeIntegral<int>();
        auto type = PickFrameType(fdp);
        auto ts = GenerateFuzzTimestamp(fdp);

        // Determine the number of elements for the vector
        size_t num_elements = fdp.ConsumeIntegralInRange<size_t>(1, 100);

        // Create a vector to hold the integers
        std::vector<int> int_list;

        // Populate the vector by consuming integers from FuzzedDataProvider
        for (size_t i = 0; i < num_elements; i++)
        {
            int_list.push_back(fdp.ConsumeIntegral<int>());
        }

        // Convert the std::vector<int> to std::vector<uint8_t>
        std::vector<std::uint8_t> byte_list;
        for (const auto &val : int_list)
        {
            // Safely cast/convert each int value to uint8_t
            byte_list.push_back(static_cast<std::uint8_t>(val & 0xFF)); // Mask to 8 bits
        }

        auto inData = std::make_shared<dai::ImgFrame>();
        inData->setWidth(width);   // Image width in pixels
        inData->setHeight(height); // Image height in pixels
        inData->setCategory(category);
        inData->setType(type);      // Format/type (e.g.,dai::RawImgFrame::Type::BGR888)
        inData->setData(byte_list); // Raw image  data buffer
        // FIXME:
        inData->setTimestamp(ts);            // Timestamp
        inData->setSequenceNum(seqNum);      // Sequence number
        inData->setInstanceNum(instanceNum); // Instance number

        std::deque<dai::ros::DisparityMsgs::DisparityImage> outImageMsg;
        converter.toRosMsg(inData, outImageMsg);
    }
    catch (const std::exception &e)
    {
        printf("[!] ERR");
        return -1;
    }

    return 0;
}

// 1. NULL PTR into memcpy as dst
// --> DisparityConverter:75 (crash-adc...2fc)

// 2. Multiple OOM situations in toRosMsg where aaray allocation takes place.
// --> e.g. ./oom-ba3c2d81f661ecb7fffa1aef25893db89f10b297

// 3. /home/user/git/work/depthai_ctrl/src/DisparityConverter.cpp:99:65: runtime
// error: -8 is outside the range of representable values of type 'unsigned
// long' SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior
// /home/user/git/work/depthai_ctrl/src/DisparityConverter.cpp:99:65
//         std::transform(raw16Data.begin(), raw16Data.end(),
//         std::back_inserter(convertedData),
//                       [](int16_t disp) -> std::size_t { return
//                       static_cast<float>(disp) / 32.0; });

// 4. Dvi by 0
//         outImageMsg.step = size / inData->getHeight();
// crash-crash-4b3043662cdde20ecb4bfc70fa0ca03001438378

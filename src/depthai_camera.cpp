#include "depthai_camera.h"
#include "depthai_utils.h"
#include <nlohmann/json.hpp>

using namespace depthai_ctrl;

using std::placeholders::_1;
using Profile = dai::VideoEncoderProperties::Profile;

using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::seconds;

void DepthAICamera::Initialize()
{
    RCLCPP_INFO(get_logger(), "[%s]: Initializing...", get_name());
    declare_parameter<std::string>("left_camera_topic", "camera/left/image_raw");
    declare_parameter<std::string>("right_camera_topic", "camera/right/image_raw");
    declare_parameter<std::string>("color_camera_topic", "camera/color/image_raw");
    declare_parameter<std::string>("video_stream_topic", "camera/color/video");
    declare_parameter<std::string>("stream_control_topic", "videostreamcmd");

    const std::string left_camera_topic = get_parameter("left_camera_topic").as_string();
    const std::string right_camera_topic = get_parameter("right_camera_topic").as_string();
    const std::string color_camera_topic = get_parameter("color_camera_topic").as_string();
    const std::string video_stream_topic = get_parameter("video_stream_topic").as_string();
    const std::string stream_control_topic = get_parameter("stream_control_topic").as_string();

    _left_publisher = create_publisher<ImageMsg>(left_camera_topic, rclcpp::SensorDataQoS());
    _right_publisher = create_publisher<ImageMsg>(right_camera_topic, rclcpp::SensorDataQoS());
    _color_publisher = create_publisher<ImageMsg>(color_camera_topic, rclcpp::SensorDataQoS());
    _video_publisher = create_publisher<CompressedImageMsg>(video_stream_topic, rclcpp::SystemDefaultsQoS());
    _stream_command_subscriber = create_subscription<std_msgs::msg::String>(
        stream_control_topic, rclcpp::SystemDefaultsQoS(), std::bind(&DepthAICamera::VideoStreamCommand, this, _1));

    // Video Stream parameters
    declare_parameter<std::string>("encoding", "H264");
    declare_parameter<int>("width", 1280);
    declare_parameter<int>("height", 720);
    declare_parameter<int>("fps", 25);
    declare_parameter<int>("bitrate", 3000000);

    _videoWidth = get_parameter("width").as_int();
    _videoHeight = get_parameter("height").as_int();
    _videoFps = get_parameter("fps").as_int();
    _videoBitrate = get_parameter("bitrate").as_int();
    _videoH265 = (get_parameter("encoding").as_string() == "H265");
}

void DepthAICamera::VideoStreamCommand(std_msgs::msg::String::SharedPtr msg)
{
    nlohmann::json cmd{};
    try
    {
        cmd = nlohmann::json::parse(msg->data.c_str());
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while parsing JSON string from VideoCommand");
        return;
    }

    if (!cmd["Command"].empty())
    {
        std::string command = cmd["Command"];
        std::transform(
            command.begin(), command.end(), command.begin(), [](unsigned char c) { return std::tolower(c); });
        if (command == "start")
        {
            int width = _videoWidth;
            int height = _videoHeight;
            int fps = _videoFps;
            int bitrate = _videoBitrate;
            std::string encoding = _videoH265 ? "H265" : "H264";
            std::string error_message{};

            if (!cmd["Width"].empty() && cmd["Width"].is_number_integer())
            {
                nlohmann::from_json(cmd["Width"], width);
            }
            if (!cmd["Height"].empty() && cmd["Height"].is_number_integer())
            {
                nlohmann::from_json(cmd["Height"], height);
            }
            if (!cmd["Fps"].empty() && cmd["Fps"].is_number_integer())
            {
                nlohmann::from_json(cmd["Fps"], fps);
            }
            if (!cmd["Bitrate"].empty() && cmd["Bitrate"].is_number_integer())
            {
                nlohmann::from_json(cmd["Bitrate"], bitrate);
            }
            if (!cmd["Encoding"].empty() && cmd["Encoding"].is_string())
            {
                nlohmann::from_json(cmd["Encoding"], encoding);
            }

            if (DepthAIUtils::ValidateCameraParameters(width, height, fps, bitrate, encoding, error_message))
            {
                _videoWidth = width;
                _videoHeight = height;
                _videoFps = fps;
                _videoBitrate = bitrate;
                _videoH265 = (encoding == "H265");
                TryRestarting();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), error_message);
            }
        }
    }
}

void DepthAICamera::TryRestarting()
{
    if (_thread_running)
    {
        _thread_running = false;
    }
    if (_processing_thread.joinable())
    {
        _processing_thread.join();
    }

    RCLCPP_INFO(this->get_logger(), "[%s]: (Re)Starting...", get_name());

    _pipeline = std::make_shared<dai::Pipeline>();

    auto monoLeft = _pipeline->create<dai::node::MonoCamera>();
    auto monoRight = _pipeline->create<dai::node::MonoCamera>();
    auto colorCamera = _pipeline->create<dai::node::ColorCamera>();
    auto videoEncoder = _pipeline->create<dai::node::VideoEncoder>();

    auto xoutLeft = _pipeline->create<dai::node::XLinkOut>();
    auto xoutRight = _pipeline->create<dai::node::XLinkOut>();
    auto xoutColor = _pipeline->create<dai::node::XLinkOut>();
    auto xoutVideo = _pipeline->create<dai::node::XLinkOut>();

    // Setup Grayscale Cameras
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // Setup Color Camera
    colorCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCamera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    // Preview resolution cannot be larger than Video's, thus resolution color camera image is limited
    colorCamera->setPreviewSize(_videoWidth, _videoHeight);
    colorCamera->setVideoSize(_videoWidth, _videoHeight);
    colorCamera->setFps(float(_videoFps));
    videoEncoder->setBitrate(_videoBitrate);
    Profile encoding = _videoH265 ? Profile::H265_MAIN : Profile::H264_BASELINE;
    videoEncoder->setDefaultProfilePreset(_videoWidth, _videoHeight, static_cast<float>(_videoFps), encoding);

    colorCamera->video.link(videoEncoder->input);
    videoEncoder->bitstream.link(xoutVideo->input);
    monoLeft->out.link(xoutLeft->input);
    monoRight->out.link(xoutRight->input);
    colorCamera->preview.link(xoutColor->input);

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutColor->setStreamName("color");
    xoutVideo->setStreamName("video");

    for (int i = 0; i < 3 && !_device; i++)
    {
        try
        {
            _device = std::make_shared<dai::Device>(*_pipeline, true);
        }
        catch (const std::runtime_error& err)
        {
            RCLCPP_ERROR(get_logger(), "Cannot start DepthAI camera: " + std::string(err.what()));
            _device.reset();
        }
    }
    if (!_device)
    {
        return;
    }

    _leftQueue = _device->getOutputQueue("left", 30, false);
    _rightQueue = _device->getOutputQueue("right", 30, false);
    _colorQueue = _device->getOutputQueue("color", 30, false);
    _videoQueue = _device->getOutputQueue("video", 30, true);

    _thread_running = true;
    _processing_thread = std::thread(&DepthAICamera::ProcessingThread, this);
}

void DepthAICamera::ProcessingThread()
{
    static uint64_t frame_count = 0;
    while (rclcpp::ok() && _thread_running && !_device->isClosed())
    {
        auto leftPtr = _leftQueue->tryGet<dai::ImgFrame>();
        auto rightPtr = _rightQueue->tryGet<dai::ImgFrame>();
        auto colorPtr = _colorQueue->tryGet<dai::ImgFrame>();
        auto videoPtr = _videoQueue->tryGet<dai::ImgFrame>();

        if (leftPtr != nullptr)
        {
            auto image = ConvertImage(leftPtr, _left_camera_frame);
            _left_publisher->publish(*image);
        }
        if (rightPtr != nullptr)
        {
            auto image = ConvertImage(rightPtr, _right_camera_frame);
            _right_publisher->publish(*image);
        }
        if (colorPtr != nullptr)
        {
            auto image = ConvertImage(colorPtr, _color_camera_frame);
            _color_publisher->publish(*image);
        }
        if (videoPtr != nullptr)
        {
            const auto stamp = videoPtr->getTimestamp();
            const int32_t sec = duration_cast<seconds>(stamp.time_since_epoch()).count();
            const int32_t nsec = duration_cast<nanoseconds>(stamp.time_since_epoch()).count() % 1000000000UL;

            CompressedImageMsg video_stream_chunk{};
            video_stream_chunk.header.frame_id = _color_camera_frame;
            video_stream_chunk.header.stamp = rclcpp::Time(sec, nsec, RCL_STEADY_TIME);
            video_stream_chunk.data.swap(videoPtr->getData());
            video_stream_chunk.format = _videoH265 ? "H265" : "H264";
            _video_publisher->publish(video_stream_chunk);
            if (frame_count < 100 || frame_count % 100 == 0)
            {
                RCLCPP_INFO(get_logger(), "Submit video-chunk #%d with time: %d.%09d", frame_count, sec, nsec);
            }
            frame_count++;
        }
    }
}

std::shared_ptr<DepthAICamera::ImageMsg> DepthAICamera::ConvertImage(const std::shared_ptr<dai::ImgFrame> input,
                                                                     const std::string& frame_id)
{
    auto message = std::make_shared<ImageMsg>();
    const auto stamp = input->getTimestamp();
    const int32_t sec = duration_cast<seconds>(stamp.time_since_epoch()).count();
    const int32_t nsec = duration_cast<nanoseconds>(stamp.time_since_epoch()).count() % 1000000000UL;

    message->header.stamp = rclcpp::Time(sec, nsec, RCL_STEADY_TIME);
    message->header.frame_id = frame_id;

    if (encodingEnumMap.find(input->getType()) != encodingEnumMap.end())
    {
        message->encoding = encodingEnumMap[input->getType()];
    }

    message->height = input->getHeight();
    message->width = input->getWidth();
    message->step = input->getData().size() / input->getHeight();
    message->data.swap(input->getData());
    return message;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAICamera)

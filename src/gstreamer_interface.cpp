#include <../include/gstreamer_interface.hpp>


using namespace depthai_ctrl;
using std::placeholders::_1;

GstInterface::GstInterface(int argc, char * argv[])
: _mLoopContext(nullptr), _mLoopThread(nullptr), _mLoop(nullptr), _pipeline(nullptr),
  _pipeline(nullptr), _appSource(nullptr), _encoderProfile("H264"),
  _busWatchId(0), _bus(nullptr), _needDataSignalId(0)
  _isStreamPlaying(false), _isStreamDefault(false)
{
  _streamAddress = "";
  gst_init(&argc, &argv);
  _mLoopContext = g_main_context_default();
  _mLoop = g_main_loop_new(_mLoopContext, false);
}

GstInterface::~GstInterface()
{
  if (_needDataSignalId != 0) {
    g_signal_handler_disconnect(_appSource, _needDataSignalId);
  }
  if (_appSource != nullptr) {
    g_signal_emit_by_name(_appSource, "end-of-stream", &ret);
    if (ret != GST_FLOW_OK) {
      g_printerr("Error: Emit end-of-stream failed\n");
    }
  }
  if (_pipeline) {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
    _pipeline = nullptr;
  }
  if (_bus) {
    gst_bus_remove_watch(_bus);
    gst_object_unref(_bus);
  }
  if (_busWatchId != 0) {
    g_source_remove(_busWatchId);
    _busWatchId = 0;
  }
  if (_mLoop) {
    g_main_loop_quit(_mLoop);
    _mLoop = nullptr;
  }
  if (_mLoopThread) {
    g_thread_join(_mLoopThread);
  }
  if (_mLoopContext) {
    g_main_context_unref(_mLoopContext);
  }
  _isStreamPlaying = false;
}

void GstInterface::BuildDefaultPipeline(
  const std::string & h26xencoder, const std::string & sink,
  const std::string & payload)
{
  _isStreamDefault = true;

  const std::string pipeline_string = "videotestsrc name=source pattern=chroma-zone-plate "
    "! video/x-raw,format=I420,width=1280,height=720 "
    "! textoverlay text=\"Camera not detected\" valignment=4 halignment=1 font-desc=Sans "
    "! videoconvert ! " + h26xencoder + " speed-preset=2 ! queue " +
    payload + "! " + sink;
  std::cout << "Starting no-camera pipeline:" << std::endl;
  std::cout << pipeline_string << std::endl;
  GError * parse_error = nullptr;
  _pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
  if (parse_error != nullptr) {
    std::cerr << "Gst Parse Error " << parse_error->code << ": " << parse_error->message <<
      std::endl;
    g_clear_error(&parse_error);
    parse_error = nullptr;
    _isStreamPlaying = false;
    return;
  }
  g_assert(_pipeline);
}

void GstInterface::BuildPipeline()
{
  const bool is_udp_protocol = (streamAddress.find("udp://") == 0);
  const std::string h26xparse = (encoderProfile == "H264") ? "h264parse" : "h265parse";
  const std::string h26xencoder = (encoderProfile == "H264") ? "x264enc" : "x265enc";
  const std::string gstFormat = (encoderProfile == "H264") ? "video/x-h264" : "video/x-h265";
  std::string payload = " ";
  std::string sink{};

  if (is_udp_protocol) {
    std::string host = DepthAIUtils::ReadIpFromUdpAddress(streamAddress);
    int port = DepthAIUtils::ReadPortFromUdpAddress(streamAddress);

    sink = "udpsink host=" + host + " port=" + std::to_string(port) + " ";
    payload = (encoderProfile == "H264") ? "! rtph264pay " : "! rtph265pay ";
  } else {
    sink = "rtspclientsink protocols=tcp tls-validation-flags=0 location=" + streamAddress;
  }

  if (queue.empty()) {     // video-data is not available - use "default" video output
    BuildDefaultPipeline(h26xencoder, sink, payload);
  } else {
    _isStreamDefault = false;

    const std::string pipeline_string = "appsrc name=source ! " + h26xparse + " " + payload + "! " +
      sink;
    std::cout << "Starting pipeline:" << std::endl;
    std::cout << pipeline_string << std::endl;
    GError * parse_error = nullptr;
    _pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
    if (parse_error != nullptr) {
      std::cerr << "Gst Parse Error " << parse_error->code << ": " << parse_error->message <<
        std::endl;
      g_clear_error(&parse_error);
      parse_error = nullptr;
      _isStreamPlaying = false;
      return;
    }
    g_assert(_pipeline);
    appSource = gst_bin_get_by_name(GST_BIN(_pipeline), "source");
    g_assert(_appSource);
    g_assert(GST_IS_APP_SRC(_appSource));

    g_object_set(
      G_OBJECT(_appSource),
      "stream-type",
      0,
      "is-live",
      TRUE,
      "block",
      FALSE,
      "format",
      GST_FORMAT_TIME,
      nullptr);
  }
  _mLoopThread = g_thread_new("GstThread", (GThreadFunc)DepthAIGst::PlayStream, this);

  _needDataSignalId = g_signal_connect(_appSource, "need-data", G_CALLBACK(NeedDataCallBack), this);
}

static void NeedDataCallBack(GstElement * appsrc, guint unused_size, gpointer user_data)
{
  GstInterface * data = (GstInterface *)user_data;
  if (!data->queue.empty() && !data->_isStreamDefault) {
    auto videoPtr = data->queue.front();
    data->queueMutex.lock();
    data->queue.pop();
    data->queueMutex.unlock();

    auto & frame = videoPtr->data;
    GstBuffer * buffer = gst_buffer_new_and_alloc(frame.size());
    gst_buffer_fill(buffer, 0, &frame[0], frame.size());
    const auto stamp = videoPtr->header.stamp;
    const GstClockTime gst_stamp = stamp.sec * 1000000000UL + stamp.nanosec;

    if (data->stamp0 == 0) {
      data->stamp0 = gst_stamp;
    }

    const GstClockTime local_stamp = gst_stamp - data->stamp0;
    GST_BUFFER_PTS(buffer) = local_stamp;

    const auto result = ::gst_app_src_push_buffer(GST_APP_SRC(data->_appSource), buffer);
    std::cout << "Submitted: " << gst_stamp << " local=" << local_stamp << ": " << std::to_string(
      result) <<
      std::endl;
  }
}

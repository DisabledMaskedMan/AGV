#include "scan_core.hpp"

#ifdef SCAN_WITH_ROS2
#include "ros_obstacle_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#endif

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

namespace scan_web {
namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace ws = beast::websocket;
using Tcp = net::ip::tcp;
using Error = boost::system::error_code;

struct Url {
  std::string host, port, target;
  static Url parse(const std::string & source) {
    if (source.rfind("ws://", 0) != 0) throw std::invalid_argument("数据源需要 ws:// 地址");
    const auto slash = source.find('/', 5);
    const auto authority = source.substr(5, slash == std::string::npos ? slash : slash - 5);
    const auto colon = authority.find(':');
    Url result{authority.substr(0, colon), colon == std::string::npos ? "80" : authority.substr(colon + 1),
               slash == std::string::npos ? "/" : source.substr(slash)};
    if (result.host.empty() || result.port.empty() || authority.find('@') != std::string::npos) throw std::invalid_argument("数据源地址无效");
    return result;
  }
};

class ScanReader : public std::enable_shared_from_this<ScanReader> {
public:
  ScanReader(net::io_context & context, Service & service)
  : context_(context), service_(service), url_(Url::parse(service.url)), resolver_(context), retry_(context), watchdog_(context) {}

  void start() {
    stopped_ = false;
    connect();
  }

  void stop() {
    stopped_ = true;
    resolver_.cancel(); retry_.cancel(); watchdog_.cancel();
    close_socket();
  }

private:
  using Stream = ws::stream<beast::tcp_stream>;
  net::io_context & context_;
  Service & service_;
  Url url_;
  Tcp::resolver resolver_;
  net::steady_timer retry_, watchdog_;
  std::shared_ptr<Stream> stream_;
  std::shared_ptr<beast::flat_buffer> buffer_;
  Clock::time_point last_target_;
  bool stopped_{false}, retrying_{false};
  unsigned failures_{0};
  const std::string subscription_{"{\"channel\":\"ros\",\"op\":\"subscribe_scan\",\"topic\":\"/scan\"}"};

  void close_socket() {
    if (!stream_) return;
    Error ignored;
    auto & socket = beast::get_lowest_layer(*stream_).socket();
    socket.cancel(ignored); socket.shutdown(Tcp::socket::shutdown_both, ignored); socket.close(ignored);
  }

  void failed(const std::string & message, const std::shared_ptr<Stream> & stream) {
    if (stopped_ || stream != stream_ || retrying_) return;
    retrying_ = true;
    service_.fail(message, "disconnected"); service_.connection("disconnected", true);
    resolver_.cancel(); watchdog_.cancel(); close_socket();
    const int delays[] = {1, 2, 5};
    retry_.expires_after(std::chrono::seconds(delays[std::min(failures_++, 2U)]));
    retry_.async_wait([self = shared_from_this()](Error error) {if (!error && !self->stopped_) self->connect();});
  }

  bool ok(Error error, const std::shared_ptr<Stream> & stream) {
    if (stopped_ || stream != stream_ || retrying_) return false;
    if (error) {failed(error.message(), stream); return false;}
    return true;
  }

  void connect() {
    retrying_ = false;
    service_.connection("connecting");
    stream_ = std::make_shared<Stream>(context_);
    buffer_ = std::make_shared<beast::flat_buffer>();
    const auto stream = stream_;
    watchdog_.expires_after(std::chrono::seconds(5));
    watchdog_.async_wait([self = shared_from_this(), stream](Error error) {
      if (!error) self->failed("连接或订阅握手超时", stream);
    });
    resolver_.async_resolve(url_.host, url_.port,
      [self = shared_from_this(), stream](Error error, Tcp::resolver::results_type endpoints) {
        if (!self->ok(error, stream)) return;
        beast::get_lowest_layer(*stream).expires_after(std::chrono::seconds(5));
        beast::get_lowest_layer(*stream).async_connect(endpoints,
          [self, stream](Error connect_error, const auto &) {
            if (!self->ok(connect_error, stream)) return;
            beast::get_lowest_layer(*stream).expires_never();
            stream->set_option(ws::stream_base::timeout::suggested(beast::role_type::client));
            stream->read_message_max(16 * 1024 * 1024);
            stream->async_handshake(self->url_.host + ':' + self->url_.port, self->url_.target,
              [self, stream](Error handshake_error) {
                if (!self->ok(handshake_error, stream)) return;
                stream->async_write(net::buffer(self->subscription_), [self, stream](Error write_error, std::size_t) {
                  if (!self->ok(write_error, stream)) return;
                  self->watchdog_.cancel(); self->last_target_ = Clock::now();
                  self->service_.connection("waiting"); self->watch(stream); self->read(stream, self->buffer_);
                });
              });
          });
      });
  }

  void watch(const std::shared_ptr<Stream> & stream) {
    watchdog_.expires_after(std::chrono::milliseconds(100));
    watchdog_.async_wait([self = shared_from_this(), stream](Error error) {
      if (!self->ok(error, stream)) return;
      if (std::chrono::duration<double>(Clock::now() - self->last_target_).count() > self->service_.config().cloud_timeout) {
        self->failed("目标扫描接收超时", stream); return;
      }
      self->watch(stream);
    });
  }

  void read(const std::shared_ptr<Stream> & stream, const std::shared_ptr<beast::flat_buffer> & buffer) {
    stream->async_read(*buffer, [self = shared_from_this(), stream, buffer](Error error, std::size_t) {
      if (!self->ok(error, stream)) return;
      try {
        auto packet = parse_packet(json::parse(beast::buffers_to_string(buffer->data())), self->service_.frame);
        if (packet) {
          self->service_.accept(std::move(*packet)); self->last_target_ = Clock::now(); self->failures_ = 0;
        }
      } catch (const std::exception & exception) {self->service_.fail(exception.what());}
      buffer->consume(buffer->size());
      self->read(stream, buffer);
    });
  }
};

struct Assets {
  std::map<std::string, std::pair<std::string, std::string>> files;
  explicit Assets(const std::filesystem::path & root) {
    for (const auto & entry : std::map<std::string, std::string>{{"index.html", "text/html"}, {"app.js", "text/javascript"},
                                                               {"roi.js", "text/javascript"}, {"style.css", "text/css"}}) {
      std::ifstream input(root / entry.first, std::ios::binary);
      if (!input) throw std::runtime_error("无法读取网页资源: " + (root / entry.first).string());
      std::ostringstream content; content << input.rdbuf();
      files[entry.first == "index.html" ? "/" : '/' + entry.first] = {content.str(), entry.second + "; charset=utf-8"};
    }
  }
};

class HttpSession : public std::enable_shared_from_this<HttpSession> {
public:
  HttpSession(Tcp::socket socket, Service & service, const Assets & assets)
  : stream_(std::move(socket)), service_(service), assets_(assets) {}
  void start() {
    stream_.expires_after(std::chrono::seconds(5));
    parser_.body_limit(16384);
    http::async_read(stream_, buffer_, parser_, [self = shared_from_this()](Error error, std::size_t) {
      if (error) return;
      self->respond();
    });
  }
private:
  beast::tcp_stream stream_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
  Service & service_;
  const Assets & assets_;

  void respond() {
    const auto & request = parser_.get();
    auto response = std::make_shared<http::response<http::string_body>>(http::status::ok, request.version());
    response->set(http::field::content_type, "application/json; charset=utf-8");
    response->set(http::field::cache_control, "no-store");
    response->set("X-Content-Type-Options", "nosniff");
    response->keep_alive(false);
    const auto route = std::string(request.target().substr(0, request.target().find('?')));
    try {
      if (request.method() == http::verb::get && route == "/api/state") response->body() = service_.state_text();
      else if (request.method() == http::verb::get && route == "/api/config") response->body() = json::serialize(service_.config().object());
      else if (request.method() == http::verb::post && (route == "/api/config" || route == "/api/config/save")) {
        if (!request[http::field::origin].empty() && request[http::field::origin] != "http://" + std::string(request[http::field::host])) {
          response->result(http::status::forbidden); response->body() = json::serialize(json::object{{"error", "只允许本页面修改参数"}});
        } else if (request[http::field::content_type].substr(0, 16) != "application/json") {
          response->result(http::status::unsupported_media_type); response->body() = json::serialize(json::object{{"error", "请使用 application/json"}});
        } else response->body() = json::serialize(service_.update(json::parse(request.body()), route == "/api/config/save"));
      } else if (request.method() == http::verb::get && assets_.files.count(route)) {
        const auto & asset = assets_.files.at(route);
        response->body() = asset.first; response->set(http::field::content_type, asset.second);
      } else {
        response->result(http::status::not_found); response->body() = json::serialize(json::object{{"error", "路径不存在"}});
      }
    } catch (const std::system_error & exception) {
      response->result(http::status::internal_server_error);
      response->body() = json::serialize(json::object{{"error", std::string("保存失败: ") + exception.what()}});
    } catch (const std::exception & exception) {
      response->result(http::status::bad_request); response->body() = json::serialize(json::object{{"error", exception.what()}});
    }
    response->prepare_payload();
    stream_.expires_after(std::chrono::seconds(5));
    http::async_write(stream_, *response, [self = shared_from_this(), response](Error, std::size_t) {
      Error ignored; self->stream_.socket().shutdown(Tcp::socket::shutdown_send, ignored);
    });
  }
};

class HttpServer {
public:
  HttpServer(net::io_context & context, const std::string & host, unsigned short port, Service & service, const Assets & assets)
  : acceptor_(context), service_(service), assets_(assets) {
    const Tcp::endpoint endpoint(net::ip::make_address(host), port);
    acceptor_.open(endpoint.protocol()); acceptor_.set_option(net::socket_base::reuse_address(true));
    acceptor_.bind(endpoint); acceptor_.listen();
    accept();
  }
private:
  Tcp::acceptor acceptor_;
  Service & service_;
  const Assets & assets_;
  void accept() {
    acceptor_.async_accept([this](Error error, Tcp::socket socket) {
      if (!error) std::make_shared<HttpSession>(std::move(socket), service_, assets_)->start();
      if (acceptor_.is_open()) accept();
    });
  }
};
}

int main(int argc, char ** argv) {
  using namespace scan_web;
  try {
    std::map<std::string, std::string> options{{"--ws-url", "ws://192.168.31.7:6060"}, {"--frame", "scan_link"},
      {"--host", "127.0.0.1"}, {"--port", "8765"}, {"--config", ""}, {"--static-dir", ""}};
    for (int index = 1; index < argc; ++index) {
      const std::string name = argv[index];
      if (name == "--ros-args") break;
      if (name == "--help") {
        std::cout << "scan_web_server [--ws-url ws://host:port] [--frame scan_link] [--host 127.0.0.1] [--port 8765] [--config path] [--static-dir path]\n";
        return 0;
      }
      if (!options.count(name) || index + 1 >= argc) throw std::invalid_argument("未知参数或缺少参数值: " + name);
      options[name] = argv[++index];
    }
    if (options["--frame"].find_first_not_of('/') == std::string::npos) throw std::invalid_argument("frame 不能为空");
    const auto executable = std::filesystem::canonical(argv[0]);
    const auto installed = executable.parent_path().parent_path().parent_path() / "share" / "mid360_suspended_object_detection";
    const std::filesystem::path root = std::filesystem::exists(installed / "config" / "scan_web.yaml") ? installed : SCAN_SOURCE_ROOT;
    if (options["--config"].empty()) options["--config"] = (root / "config" / "scan_web.yaml").string();
    if (options["--static-dir"].empty()) options["--static-dir"] = (root / "scan_web" / "static").string();
    std::size_t consumed = 0;
    const auto port = std::stoul(options["--port"], &consumed);
    if (consumed != options["--port"].size() || port == 0 || port > 65535) throw std::invalid_argument("端口必须在 1..65535");
    Service service(Config::load(options["--config"]), options["--ws-url"], options["--frame"], options["--config"]);
    net::io_context reader_context, http_context;
    auto reader = std::make_shared<ScanReader>(reader_context, service);
    const Assets assets(options["--static-dir"]);
    HttpServer server(http_context, options["--host"], static_cast<unsigned short>(port), service, assets);
#ifdef SCAN_WITH_ROS2
    std::vector<const char *> ros_arguments{argv[0]};
    for (int index = 1; index < argc; ++index) {
      if (std::string(argv[index]) == "--ros-args") {
        for (; index < argc; ++index) ros_arguments.push_back(argv[index]);
        break;
      }
    }
    rclcpp::init(static_cast<int>(ros_arguments.size()), ros_arguments.data(),
      rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto ros_node = std::make_shared<RosObstaclePublisher>(service);
    auto ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    ros_executor->add_node(ros_node);
    std::thread ros_thread([&] {ros_executor->spin();});
#endif
    net::signal_set signals(http_context, SIGINT, SIGTERM);
    signals.async_wait([&](Error, int) {
      net::post(reader_context, [reader] {reader->stop();}); http_context.stop();
#ifdef SCAN_WITH_ROS2
      ros_executor->cancel();
#endif
    });
    service.start(); reader->start();
    std::thread receiver([&] {reader_context.run();});
    std::cout << "MID-360 C++ Web: http://" << options["--host"] << ':' << port << "\nSource: " << service.url << " [" << service.frame << "]" << std::endl;
    http_context.run();
    net::post(reader_context, [reader] {reader->stop();});
    receiver.join(); service.stop();
#ifdef SCAN_WITH_ROS2
    ros_executor->cancel();
    ros_thread.join();
    rclcpp::shutdown();
#endif
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "启动失败: " << exception.what() << '\n'; return 1;
  }
}

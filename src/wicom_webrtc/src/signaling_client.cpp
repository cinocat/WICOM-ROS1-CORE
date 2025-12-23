#include "wicom_webrtc/signaling_client.h"

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <iostream>
#include <thread>
#include <mutex>

typedef websocketpp::client<websocketpp::config::asio_client> client_t;

struct SignalingClient::Impl {
  client_t client;
  websocketpp::connection_hdl hdl;
  std::string uri;
  std::thread asio_thread;
  bool connected = false;

  SignalingClient::OnOpenFn on_open;
  SignalingClient::OnMessageFn on_message;
  SignalingClient::OnCloseFn on_close;
  std::mutex mutex;
};

SignalingClient::SignalingClient(const std::string& url) : impl_(new Impl()) {
  impl_->uri = url;
  impl_->client.init_asio();

  impl_->client.set_open_handler([this](websocketpp::connection_hdl h) {
    impl_->connected = true;
    if (impl_->on_open) impl_->on_open();
  });

  impl_->client.set_message_handler([this](websocketpp::connection_hdl, client_t::message_ptr msg) {
    if (impl_->on_message) impl_->on_message(msg->get_payload());
  });

  impl_->client.set_close_handler([this](websocketpp::connection_hdl) {
    impl_->connected = false;
    if (impl_->on_close) impl_->on_close();
  });

  impl_->client.set_fail_handler([this](websocketpp::connection_hdl) {
    impl_->connected = false;
    if (impl_->on_close) impl_->on_close();
  });
}

SignalingClient::~SignalingClient() {
  try {
    impl_->client.stop();
    if (impl_->asio_thread.joinable()) impl_->asio_thread.join();
  } catch (...) {
    // swallow
  }
  // unique_ptr cleans up impl_ automatically; do NOT delete impl_ manually
}

bool SignalingClient::Connect() {
  websocketpp::lib::error_code ec;
  client_t::connection_ptr con = impl_->client.get_connection(impl_->uri, ec);
  if (ec) {
    std::cerr << "WebSocket get_connection error: " << ec.message() << std::endl;
    return false;
  }
  impl_->hdl = con->get_handle();
  impl_->client.connect(con);

  impl_->asio_thread = std::thread([this]() {
    try {
      impl_->client.run();
    } catch (const std::exception& e) {
      std::cerr << "WebSocket ASIO thread exception: " << e.what() << std::endl;
    }
  });

  int attempts = 0;
  while (!impl_->connected && attempts < 50) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    attempts++;
  }
  return impl_->connected;
}

bool SignalingClient::Send(const std::string& message) {
  if (!impl_->connected) return false;
  websocketpp::lib::error_code ec;
  impl_->client.send(impl_->hdl, message, websocketpp::frame::opcode::text, ec);
  if (ec) {
    std::cerr << "WebSocket send error: " << ec.message() << std::endl;
    return false;
  }
  return true;
}

void SignalingClient::SetOnOpen(OnOpenFn cb) { impl_->on_open = cb; }
void SignalingClient::SetOnMessage(OnMessageFn cb) { impl_->on_message = cb; }
void SignalingClient::SetOnClose(OnCloseFn cb) { impl_->on_close = cb; }
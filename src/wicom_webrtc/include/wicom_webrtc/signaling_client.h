#ifndef SIGNALING_CLIENT_H
#define SIGNALING_CLIENT_H

#include <functional>
#include <string>
#include <memory>

class SignalingClient {
public:
  using OnOpenFn = std::function<void()>;
  using OnMessageFn = std::function<void(const std::string&)>;
  using OnCloseFn = std::function<void()>;

  SignalingClient(const std::string& url);
  ~SignalingClient();

  bool Connect();
  bool Send(const std::string& message);

  void SetOnOpen(OnOpenFn cb);
  void SetOnMessage(OnMessageFn cb);
  void SetOnClose(OnCloseFn cb);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif // SIGNALING_CLIENT_H
// include/wm/core/status.hpp
#pragma once

#include <optional>
#include <string>
#include <utility>

namespace wm {

class Status {
 public:
  enum class Code : int {
    kOk = 0,

    // Caller errors
    kInvalidArgument,
    kOutOfRange,

    // Environment / IO
    kNotFound,
    kIoError,
    kPermissionDenied,

    // Data / parsing
    kParseError,
    kCorruptData,

    // System / unexpected
    kUnsupported,
    kInternal,
  };

  Status() = default;  // OK
  Status(Code code, std::string message)
      : code_(code), message_(std::move(message)) {}

  [[nodiscard]] bool ok() const noexcept { return code_ == Code::kOk; }
  [[nodiscard]] Code code() const noexcept { return code_; }
  [[nodiscard]] const std::string& message() const noexcept { return message_; }

  static Status ok_status() { return Status(); }

  static Status invalid_argument(std::string msg) { return Status(Code::kInvalidArgument, std::move(msg)); }
  static Status out_of_range(std::string msg) { return Status(Code::kOutOfRange, std::move(msg)); }
  static Status not_found(std::string msg) { return Status(Code::kNotFound, std::move(msg)); }
  static Status io_error(std::string msg) { return Status(Code::kIoError, std::move(msg)); }
  static Status permission_denied(std::string msg) { return Status(Code::kPermissionDenied, std::move(msg)); }
  static Status parse_error(std::string msg) { return Status(Code::kParseError, std::move(msg)); }
  static Status corrupt_data(std::string msg) { return Status(Code::kCorruptData, std::move(msg)); }
  static Status unsupported(std::string msg) { return Status(Code::kUnsupported, std::move(msg)); }
  static Status internal(std::string msg) { return Status(Code::kInternal, std::move(msg)); }

 private:
  Code code_ = Code::kOk;
  std::string message_;
};

template <typename T>
class Result {
 public:
  Result() = delete;

  static Result ok(T value) { return Result(std::move(value)); }
  static Result err(Status status) { return Result(std::move(status)); }

  [[nodiscard]] bool ok() const noexcept { return status_.ok(); }
  [[nodiscard]] const Status& status() const noexcept { return status_; }

  [[nodiscard]] const T* value_if_ok() const noexcept { return ok() ? &(*value_) : nullptr; }
  [[nodiscard]] T* value_if_ok() noexcept { return ok() ? &(*value_) : nullptr; }

  [[nodiscard]] const T& value() const { return value_.value(); }
  [[nodiscard]] T& value() { return value_.value(); }

  [[nodiscard]] const T& operator*() const { return value(); }
  [[nodiscard]] T& operator*() { return value(); }

  [[nodiscard]] const T* operator->() const { return &value(); }
  [[nodiscard]] T* operator->() { return &value(); }

  [[nodiscard]] T take_value() { return std::move(value_.value()); }

 private:
  explicit Result(T value) : value_(std::move(value)), status_(Status::ok_status()) {}
  explicit Result(Status status) : value_(std::nullopt), status_(std::move(status)) {}

  std::optional<T> value_;
  Status status_;
};

#define WM_RETURN_IF_ERROR(expr)      \
  do {                                \
    const ::wm::Status _s = (expr);   \
    if (!_s.ok()) return _s;          \
  } while (0)

}  // namespace wm
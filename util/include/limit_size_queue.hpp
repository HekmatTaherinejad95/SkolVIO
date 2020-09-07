#ifndef LIMIT_SIZE_QUEUE_HPP_
#define LIMIT_SIZE_QUEUE_HPP_

#include "thread_safe_queue.hpp"

namespace vio {

template <typename T>
class LimitSizeQueue {
 public:
  LimitSizeQueue(size_t size) : size_(size) {}
  void Push(const T &data) {}

  void Push(T &&data) {}

 private:
  size_t size_;
  ThreadSafeQueue queue_;
};
}

#endif  // LIMIT_SIZE_QUEUE_HPP_

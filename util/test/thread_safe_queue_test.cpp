#include "gtest/gtest.h"

#include <iostream>
#include <thread>
#include <vector>

#include "thread_safe_queue.hpp"

class ThreadSafeQueueTest : public ::testing::Test {
 public:
  void AddElementToQueue(int element) {
    // TODO: Good to have locked pipe for logging.
    std::cout << element << std::endl;
    queue.Push(element);
  }

  void PopElementFromQueue() {
    int element = queue.Pop();
    std::cout << "-" << element << std::endl;
  }

  vio::ThreadSafeQueue<int> queue;
};

TEST_F(ThreadSafeQueueTest, TestIntQueue) {
  int num_threads = 10;
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    threads.push_back(
        std::thread(&ThreadSafeQueueTest::AddElementToQueue, this, i));
    threads.push_back(
        std::thread(&ThreadSafeQueueTest::PopElementFromQueue, this));
  }
  for (int i = 0; i < num_threads; ++i) {
    threads[i * 2].join();
    threads[i * 2 + 1].join();
  }
}

class ThreadSafeQueueTestUniquePtr : public ::testing::Test {
 public:
  void AddElementToQueue(std::unique_ptr<int> element) {
    // TODO: Good to have locked pipe for logging.
    std::cout << *element << std::endl;
    queue.Push(std::move(element));
  }

  void PopElementFromQueue() {
    std::unique_ptr<int> element = std::move(queue.Pop());
    std::cout << "-" << *element << std::endl;
  }

  vio::ThreadSafeQueue<std::unique_ptr<int>> queue;
};

TEST_F(ThreadSafeQueueTestUniquePtr, TestIntQueue) {
  int num_threads = 10;
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    std::unique_ptr<int> new_int = std::unique_ptr<int>(new int(i));
    threads.push_back(
        std::thread(&ThreadSafeQueueTestUniquePtr::AddElementToQueue, this,
                    std::move(new_int)));

    threads.push_back(
        std::thread(&ThreadSafeQueueTestUniquePtr::PopElementFromQueue, this));
  }
  for (int i = 0; i < num_threads; ++i) {
    threads[i * 2].join();
    threads[i * 2 + 1].join();
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

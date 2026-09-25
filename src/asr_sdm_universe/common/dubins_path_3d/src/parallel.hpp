// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#ifndef PARALLEL_HPP_
#define PARALLEL_HPP_

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <functional>
#include <thread>
#include <vector>

namespace dubins_path_3d
{

/// Runs `body(index)` for every index in [0, count) across worker threads.
///
/// Indices are handed out one at a time so that the uneven cost of the
/// inverse-kinematics calls balances itself. `num_threads` of zero requests the
/// hardware concurrency. The first exception thrown by any worker is rethrown on
/// the calling thread once all workers have finished.
inline void parallelFor(
  std::size_t count, int num_threads, const std::function<void(std::size_t)> & body)
{
  if (count == 0) {
    return;
  }

  unsigned int workers = (num_threads > 0) ?
    static_cast<unsigned int>(num_threads) :
    std::thread::hardware_concurrency();
  if (workers == 0) {
    workers = 1;
  }
  workers = std::min<unsigned int>(workers, static_cast<unsigned int>(count));

  if (workers == 1) {
    for (std::size_t i = 0; i < count; ++i) {
      body(i);
    }
    return;
  }

  std::atomic<std::size_t> next{0};
  std::vector<std::exception_ptr> failures(workers, nullptr);
  std::vector<std::thread> threads;
  threads.reserve(workers);

  for (unsigned int worker = 0; worker < workers; ++worker) {
    threads.emplace_back(
      [&, worker]() {
        try {
          for (std::size_t i = next.fetch_add(1); i < count; i = next.fetch_add(1)) {
            body(i);
          }
        } catch (...) {
          failures[worker] = std::current_exception();
        }
      });
  }
  for (std::thread & thread : threads) {
    thread.join();
  }

  for (const std::exception_ptr & failure : failures) {
    if (failure) {
      std::rethrow_exception(failure);
    }
  }
}

}  // namespace dubins_path_3d

#endif  // PARALLEL_HPP_

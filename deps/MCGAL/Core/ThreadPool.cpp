#include "include/ThreadPool.h"
namespace MCGAL {
// ThreadPool::ThreadPool(int num_threads, bool isStop = false) {
//     this->stop = isStop;
//     for (size_t i = 0; i < num_threads; ++i) {
//         workers.emplace_back([this] {
//             for (;;) {
//                 std::function<void()> task;
//                 {
//                     std::unique_lock<std::mutex> lock(queue_mutex);
//                     condition.wait(lock, [this] { return stop || !tasks.empty(); });
//                     if (stop && tasks.empty())
//                         return;
//                     task = std::move(tasks.front());
//                     tasks.pop();
//                 }
//                 task();
//             }
//         });
//     }
// }

// template <typename F> void ThreadPool::enqueue(F&& f) {
//     {
//         std::unique_lock<std::mutex> lock(queue_mutex);
//         tasks.emplace(std::forward<F>(f));
//     }
//     condition.notify_one();
// }

// ThreadPool::~ThreadPool() {
//     {
//         std::unique_lock<std::mutex> lock(queue_mutex);
//         stop = true;
//     }
//     condition.notify_all();
//     for (std::thread& worker : workers)
//         worker.join();
// }
}  // namespace MCGAL
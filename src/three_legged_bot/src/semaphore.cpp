#include "semaphore.h"

Semaphore::Semaphore(int count) : count_(count) {}

void Semaphore::acquire() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (count_ == 0) {
        condition_.wait(lock);
    }
    count_--;
}

void Semaphore::release() {
    std::lock_guard<std::mutex> lock(mutex_);
    count_++;
    condition_.notify_one();
}

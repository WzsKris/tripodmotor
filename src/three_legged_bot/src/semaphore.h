#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <mutex>
#include <condition_variable>

class Semaphore {
public:
    Semaphore(int count = 1);

    void acquire();
    void release();

private:
    int count_;
    std::mutex mutex_;
    std::condition_variable condition_;
};

#endif // SEMAPHORE_H

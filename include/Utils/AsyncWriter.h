#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Writes files on a small pool of worker threads.
// The queue is bounded: when the disk cannot keep up, new jobs are dropped
// (and counted) instead of spawning an unbounded number of threads.
class AsyncWriter
{
public:
    static AsyncWriter& instance();

    // Returns false when the job was dropped because the queue is full.
    bool enqueue(std::function<void()> job);

    // Blocks until all queued jobs are written.
    void flush();

    size_t pending() const;
    size_t dropped() const;

    ~AsyncWriter();

private:
    explicit AsyncWriter(size_t nWorkers = 4, size_t maxQueue = 200);
    void worker();

    mutable std::mutex mutex;
    std::condition_variable jobAvailable;
    std::condition_variable idle;
    std::deque<std::function<void()>> jobs;
    std::vector<std::thread> workers;
    size_t maxQueue;
    size_t busy = 0;
    size_t droppedJobs = 0;
    bool stopping = false;
};

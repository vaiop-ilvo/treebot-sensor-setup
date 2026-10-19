#include <Utils/AsyncWriter.h>

#include <exception>
#include <iostream>

AsyncWriter& AsyncWriter::instance()
{
    static AsyncWriter writer;
    return writer;
}

AsyncWriter::AsyncWriter(size_t nWorkers, size_t maxQueue) :
    maxQueue(maxQueue)
{
    for (size_t i = 0; i < nWorkers; ++i) {
        workers.emplace_back([this]() { worker(); });
    }
}

AsyncWriter::~AsyncWriter()
{
    // Finish all pending writes before exiting so no file is truncated
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = true;
    }
    jobAvailable.notify_all();
    for (auto& t : workers) {
        if (t.joinable()) {
            t.join();
        }
    }
}

bool AsyncWriter::enqueue(std::function<void()> job)
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (jobs.size() >= maxQueue) {
            droppedJobs++;
            return false;
        }
        jobs.push_back(std::move(job));
    }
    jobAvailable.notify_one();
    return true;
}

void AsyncWriter::flush()
{
    std::unique_lock<std::mutex> lock(mutex);
    idle.wait(lock, [this]() { return jobs.empty() && busy == 0; });
}

size_t AsyncWriter::pending() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return jobs.size() + busy;
}

size_t AsyncWriter::dropped() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return droppedJobs;
}

void AsyncWriter::worker()
{
    while (true) {
        std::function<void()> job;
        {
            std::unique_lock<std::mutex> lock(mutex);
            jobAvailable.wait(lock, [this]() { return stopping || !jobs.empty(); });
            if (jobs.empty()) {
                return; // stopping and nothing left to write
            }
            job = std::move(jobs.front());
            jobs.pop_front();
            busy++;
        }

        try {
            job();
        } catch (const std::exception& e) {
            std::cerr << "AsyncWriter: write failed: " << e.what() << std::endl;
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            busy--;
            if (jobs.empty() && busy == 0) {
                idle.notify_all();
            }
        }
    }
}

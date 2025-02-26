#ifndef RTIS_PREEMPT_EXECUTOR_HPP
#define RTIS_PREEMPT_EXECUTOR_HPP

#include "simple_timer/rt-sched.hpp"
#include <atomic>
#include <map>
#include <memory>
#include <rclcpp/any_executable.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/timer.hpp>
#include <set>
#include <string>
#include <thread>

enum ExecutableType
{
  SUBSCRIPTION,
  SERVICE,
  CLIENT,
  TIMER,
  WAITABLE,
  UNKNOWN
};
enum ModeSwitchMode
{
  LO,
  HI
};

#include <condition_variable>
#include <mutex>

// https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads
class Semaphore
{
public:
  Semaphore(int count_ = 0) : count(count_) {}

  inline void notify()
  {
    std::unique_lock<std::mutex> lock(mtx);
    count++;
    cv.notify_one();
  }

  inline void wait()
  {
    std::unique_lock<std::mutex> lock(mtx);

    while (count == 0)
    {
      cv.wait(lock);
    }
    count--;
  }

private:
  std::mutex mtx;
  std::condition_variable cv;
  int count;
};

struct PreemptableCallback
{
  bool initialized = false;
  bool tid_initialized = false;
  pid_t tid = -1;
  std::thread thread;
  // if true, the executor won't try to pass data to the thread
  std::atomic<bool> running;
  // std::binary_semaphore sem;
  // released by executor to start the thread
  std::shared_ptr<Semaphore> sem = std::make_shared<Semaphore>(0);
  // Semaphore sem;
  // set by executor before releasing the semaphore. the thread will delete it
  // once it's done
  std::shared_ptr<rclcpp::AnyExecutable> incoming_executable = nullptr;
  bool enabled = true;
  bool paused = false;
  bool params_updated = false;
};

struct RTParams
{
  bool use_rt = false;
  uint64_t period_us;
  uint64_t duration_us;
  uint64_t deadline_us;
  std::string name;
};

class PreemptExecutor : public rclcpp::Executor
{
private:
  // std::map<const void *, std::shared_ptr<std::thread>> threads;
  std::map<const void *, std::shared_ptr<PreemptableCallback>> threads;
  std::map<const void *, std::shared_ptr<RTParams>> thread_params;
  std::set<rclcpp::TimerBase::SharedPtr> scheduled_timers;
  void exec_in_thread(rclcpp::AnyExecutable &any_executable);
  void thread_task(std::shared_ptr<PreemptableCallback> cb,
                   std::shared_ptr<RTParams> params);
  std::string describe_executable(rclcpp::AnyExecutable &any_executable);

public:
  void spin();

  // not virtual in parent class
  void execute_any_executable(rclcpp::AnyExecutable &any_exec);

  void set_params(const void *handle, uint64_t period_us, uint64_t duration_us,
                  uint64_t deadline_us, std::string name);
  std::shared_ptr<RTParams> get_params(const void *handle);
  static const void *getHandle(rclcpp::AnyExecutable &any_executable);
  void dropTask(const void *handle);

  ModeSwitchMode requested_mode = ModeSwitchMode::LO;
  // user's function we call during a mode switch - we'll pass the mode to the
  // callback
  std::function<void(ModeSwitchMode)> mode_switch_callback = nullptr;
  bool requesting_mode_switch = false;
  uint request_mode_switch_time;

  std::mutex logger_mutex;
  node_time_logger logger;
};

#endif // RTIS_PREEMPT_EXECUTOR_HPP
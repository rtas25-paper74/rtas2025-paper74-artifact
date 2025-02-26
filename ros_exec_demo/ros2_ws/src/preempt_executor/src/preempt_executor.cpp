#include "preempt_executor/preempt_executor.hpp"
#include "simple_timer/rt-sched.hpp"
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/magic.h>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <ratio>
#include <rcl/timer.h>
#include <rclcpp/any_executable.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/utilities.hpp>
#include <sched.h>
#include <sstream>
#include <string>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/vfs.h>
#include <thread>
#include <unistd.h>

std::mutex timer_sched_mutex;

ExecutableType getCallbackType(rclcpp::AnyExecutable &any_executable)
{
  if (any_executable.subscription)
  {
    return ExecutableType::SUBSCRIPTION;
  }
  else if (any_executable.timer)
  {
    return ExecutableType::TIMER;
  }
  else if (any_executable.service)
  {
    return ExecutableType::SERVICE;
  }
  else if (any_executable.client)
  {
    return ExecutableType::CLIENT;
  }
  else if (any_executable.waitable)
  {
    return ExecutableType::WAITABLE;
  }
  else
  {
    return ExecutableType::UNKNOWN;
  }
}

const void *PreemptExecutor::getHandle(rclcpp::AnyExecutable &any_executable)
{
  if (any_executable.subscription)
  {
    return any_executable.subscription->get_subscription_handle().get();
  }
  if (any_executable.timer)
  {
    return any_executable.timer->get_timer_handle().get();
  }
  if (any_executable.service)
  {
    return any_executable.service->get_service_handle().get();
  }
  if (any_executable.client)
  {
    return any_executable.client->get_client_handle().get();
  }
  if (any_executable.waitable)
  {
    return any_executable.waitable.get();
  }
  throw std::runtime_error("Unknown executable type");
}

std::string
PreemptExecutor::describe_executable(rclcpp::AnyExecutable &any_exec)
{
  std::shared_ptr<RTParams> params = get_params(getHandle(any_exec));
  if (params != nullptr)
  {
    return params->name;
  }

  if (any_exec.subscription)
  {
    return std::string("sub_") + any_exec.subscription->get_topic_name();
  }
  if (any_exec.timer)
  {
    int64_t period;
    rcl_ret_t ret =
      rcl_timer_get_period(any_exec.timer->get_timer_handle().get(), &period);
    if (ret != RCL_RET_OK)
    {
      return std::string("timer_unknown");
    }
    period /= 1000000; // to ms
    return std::string("timer_") + std::to_string(period);
  }
  if (any_exec.service)
  {
    return std::string("service_") + any_exec.service->get_service_name();
  }
  if (any_exec.client)
  {
    return std::string("client_") + any_exec.client->get_service_name();
  }
  if (any_exec.waitable)
  {
    return std::string("waitable");
  }
  return std::string("unknown_executable");
}

void PreemptExecutor::set_params(const void *handle, uint64_t period_us,
                                 uint64_t duration_us, uint64_t deadline_us,
                                 std::string name)
{
  std::cerr << "set_params" << std::endl;
  if (thread_params.count(handle) == 0)
  {
    thread_params[handle] = std::make_shared<RTParams>();
  }
  thread_params[handle]->period_us = period_us;
  thread_params[handle]->duration_us = duration_us;
  thread_params[handle]->deadline_us = deadline_us;
  thread_params[handle]->use_rt = true;
  thread_params[handle]->name = name;

  // if that thread is already running, update the params
  if (threads.count(handle) != 0)
  {
    std::shared_ptr<PreemptableCallback> cb = threads[handle];
    cb->params_updated = true;
    cb->paused = false;
    if (!cb->tid_initialized)
    {
      std::cerr << "tid not initialized" << std::endl;
      return;
    }
    struct sched_attr attr;
    int ret = sched_getattr(cb->tid, &attr, sizeof(attr), 0);
    if (ret != 0)
    {
      std::cerr << "Error calling sched_getattr: " << ret << std::endl;
      int err_no = errno;
      std::cerr << "errno: " << err_no << std::endl;
      std::cerr << strerror(err_no) << std::endl;
    }
    attr.sched_runtime = duration_us;
    attr.sched_period = period_us;
    attr.sched_deadline = deadline_us;
    ret = sched_setattr(cb->tid, &attr, 0);
    if (ret != 0)
    {
      std::cerr << "Error calling sched_setattr: " << ret << std::endl;
      int err_no = errno;
      std::cerr << "errno: " << err_no << std::endl;
      std::cerr << strerror(err_no) << std::endl;
    }
  }
  else
  {
    std::cerr << "thread not running" << std::endl;
  }
  std::cerr << "set_params done" << std::endl;
}

std::shared_ptr<RTParams> PreemptExecutor::get_params(const void *handle)
{
  if (thread_params.count(handle) == 0)
  {
    return nullptr;
  }
  return thread_params[handle];
}

void PreemptExecutor::execute_any_executable(rclcpp::AnyExecutable &any_exec)
{
  if (!spinning.load())
  {
    return;
  }
  if (any_exec.timer)
  {
    execute_timer(any_exec.timer);
  }
  if (any_exec.subscription)
  {
    execute_subscription(any_exec.subscription);
  }
  if (any_exec.service)
  {
    execute_service(any_exec.service);
  }
  if (any_exec.client)
  {
    execute_client(any_exec.client);
  }
  if (any_exec.waitable)
  {
    any_exec.waitable->execute();
  }
  // Reset the callback_group, regardless of type
  //   any_exec.callback_group->can_be_taken_from().store(true);
  // Wake the wait, because it may need to be recalculated or work that
  // was previously blocked is now available.
  rcl_ret_t ret = rcl_trigger_guard_condition(&interrupt_guard_condition_);
  if (ret != RCL_RET_OK)
  {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "Failed to trigger guard condition from execute_any_executable");
  }
}

void PreemptExecutor::thread_task(
  std::shared_ptr<PreemptableCallback> callback_data,
  std::shared_ptr<RTParams> params)
{
  std::string desc = describe_executable(*callback_data->incoming_executable);
  // std::cout << "created thread for " << desc << std::endl;
  if (params != nullptr && params->use_rt)
  {
    struct sched_attr attr;
    int ret = sched_getattr(0, &attr, sizeof(attr), 0);
    if (ret != 0)
    {
      std::cerr << "Error calling sched_getattr: " << ret << std::endl;
      int err_no = errno;
      std::cerr << "errno: " << err_no << std::endl;
      std::cerr << strerror(err_no) << std::endl;
    }
    attr.sched_runtime = params->duration_us;
    attr.sched_period = params->period_us;
    attr.sched_deadline = params->deadline_us;
    attr.sched_policy = SCHED_DEADLINE;
    ret = sched_setattr(0, &attr, 0);
    if (ret != 0)
    {
      std::cerr << "Error calling sched_setattr: " << ret << std::endl;
      int err_no = errno;
      std::cerr << "errno: " << err_no << std::endl;
      std::cerr << strerror(err_no) << std::endl;
    }
    // std::cout << "set params for " << desc << std::endl;
  }
  else
  {
    std::cerr << "no params for " << desc << std::endl;
    if (params == nullptr)
    {
      std::cerr << "params is nullptr" << std::endl;
    }
    else
    {
      std::cerr << "params->use_rt is false" << std::endl;
    }
  }

  // get tid of the thread
  callback_data->tid = gettid();
  callback_data->tid_initialized = true;
  std::shared_ptr<rclcpp::AnyExecutable> incoming_executable =
    callback_data->incoming_executable;
  uint64_t last_start_time = 0;
  while (rclcpp::ok())
  {
    if (callback_data->paused)
    {
      std::this_thread::sleep_for(std::chrono::nanoseconds(params->period_us));
      continue;
    }
    else if (params == nullptr || !params->use_rt ||
             incoming_executable == nullptr || true)
    {
      // wait on the semaphore
      callback_data->sem->wait();
      incoming_executable = callback_data->incoming_executable;
    }
    else
    {
      // i know it says period_us, but turns out it's nanoseconds
      // std::this_thread::sleep_for(std::chrono::nanoseconds(params->period_us));
      uint64_t now = get_time_us();
      uint64_t diff = now - last_start_time;
      if (diff < params->period_us)
      {
        uint64_t sleep_time = params->period_us - diff;
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
      }
      incoming_executable = callback_data->incoming_executable;
      last_start_time = get_time_us();
    }

    if (!callback_data->enabled)
    {
      // std::cout << "thread " << desc << " disabled" << std::endl;
      return;
    }

    if (callback_data->params_updated)
    {
      desc = params->name;
      // sched data is set by caller
      // callback_data->params_updated = false;
      desc = describe_executable(*incoming_executable);
      callback_data->params_updated = false;
    }

    std::stringstream ss;
    ss << "{\"type\":\"exec_start\",\"desc\":\"" << desc << "\"}";
    {
      std::lock_guard<std::mutex> lock(timer_sched_mutex);
      log_entry(logger, ss.str());
    }
    u64 start_time = get_time_us();
    const void *handle = getHandle(*incoming_executable);
    if (handle == nullptr)
    {
      throw std::runtime_error("handle is nullptr");
    }
    // if (incoming_executable->timer) {
    //   // std::cout << "thread_task " << desc << std::endl;
    // }

    execute_any_executable(*incoming_executable);
    ExecutableType type = getCallbackType(*incoming_executable);
    if (type == TIMER)
    {
      const std::lock_guard<std::mutex> lock(timer_sched_mutex);
      scheduled_timers.erase(incoming_executable->timer);
    }
    // callback_data->incoming_executable->callback_group.reset();
    incoming_executable->callback_group.reset();

    u64 end_time = get_time_us();
    // if (type == TIMER) {
    //   std::cout << "executed " << desc << " in "
    //             << (end_time - start_time) / 1000.0 << " ms" << std::endl;
    // }
    {
      uint64_t duration = end_time - start_time;
      uint64_t duration_ms = duration / 1000;
      ss.str("");
      ss << "{\"type\":\"exec_end\",\"desc\":\"" << desc
         << "\",\"duration\":" << duration_ms << "}";
      std::lock_guard<std::mutex> lock(timer_sched_mutex);
      log_entry(logger, ss.str());
    }
    // free the executable
    // callback_data->incoming_executable.reset();
    callback_data->running.store(false);
    // sched_yield();
  }
  // std::cout << "thread exiting" << std::endl;
  return;
}

void PreemptExecutor::exec_in_thread(rclcpp::AnyExecutable &any_executable)
{
  const void *handle = getHandle(any_executable);

  // ExecutableType type = getCallbackType(any_executable);
  // if (type == TIMER) {
  //   // guard against multiple threads running the same timer
  //   // get scheduled timer mutex
  //   const std::lock_guard<std::mutex> lock(timer_sched_mutex);
  //   if (scheduled_timers.count(any_executable.timer) != 0) {
  //     // std::cout << "skipping timer" << std::endl;
  //     if (any_executable.callback_group != nullptr) {
  //       any_executable.callback_group->can_be_taken_from().store(true);
  //     }
  //     return;
  //   }
  //   // std::cout << "inserting timer" << std::endl;
  //   scheduled_timers.insert(any_executable.timer);
  // } else {
  // }

  // std::string desc = describe_executable(any_executable);
  // std::cout << "trying to run " << desc << std::endl;

  std::shared_ptr<PreemptableCallback> existing_thread;
  if (threads.count(handle) != 0)
  {
    existing_thread = threads[handle];
  }
  if (existing_thread != nullptr)
  {
    if (existing_thread->running.load())
    {
      // std::cout << "thread already running" << std::endl;

      return;
    }
    if (existing_thread->paused)
    {
      return;
    }
    // std::cout << "reusing existing thread" << std::endl;
    // set the new executable
    existing_thread->incoming_executable =
      std::make_shared<rclcpp::AnyExecutable>(any_executable);
    // release the semaphore
    existing_thread->running.store(true);
    // std::cout << "thread reused" << std::endl;
    existing_thread->sem->notify();
  }
  else
  {
    // std::cout << "creating a new thread" << std::endl;
    threads[handle] = std::make_shared<PreemptableCallback>();
    threads[handle]->incoming_executable =
      std::make_shared<rclcpp::AnyExecutable>(any_executable);
    threads[handle]->running.store(true);
    std::shared_ptr<RTParams> params = get_params(handle);
    threads[handle]->thread = std::thread(
      &PreemptExecutor::thread_task, this,
      std::shared_ptr<PreemptableCallback>(threads[handle]), params);
    threads[handle]->initialized = true;
    // std::cout << "thread created" << std::endl;
    threads[handle]->sem->notify();
  }
}

void PreemptExecutor::spin()
{
  std::cout << "spinning" << std::endl;
  logger = create_logger();
  if (spinning.exchange(true))
  {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false););
  while (rclcpp::ok(this->context_) && spinning.load())
  {
    if (requesting_mode_switch)
    {
      // check if all threads are idle
      bool all_idle = true;
      for (auto &thread : threads)
      {
        if (thread.second->running.load())
        {
          all_idle = false;
          // std::cout<<"not performing request yet - not idle"<<std::endl;
          break;
        }
      }
      if (all_idle)
      {
        // std::cout << "performing request - switching to " << requested_mode
        //           << std::endl;
        std::stringstream ss;
        ss << "{\"type\":\"starting_mode_switch\",\"mode\":" << requested_mode
           << "}";
        log_entry(logger, ss.str());
        if (mode_switch_callback != nullptr)
        {
          // int now = get_time_us() / 1000;
          // int time_diff = now - request_mode_switch_time;
          mode_switch_callback(requested_mode);
          sched_yield();
        }
        requesting_mode_switch = false;
        continue;
      }
    }

    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable))
    {
      exec_in_thread(any_executable);
      // sched_yield();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void PreemptExecutor::dropTask(const void *handle)
{
  // this function should be called during idle, so we can kill the thread
  if (threads.count(handle) != 0)
  {
    std::shared_ptr<PreemptableCallback> cb = threads[handle];
    // cb->enabled = false;
    // cb->sem->notify();
    // if (cb->thread.joinable()) {
    //   cb->thread.join();
    // }
    // threads.erase(handle);
    cb->paused = true;
  }
  else
  {
    std::cerr << "dropTask: handle not found" << std::endl;
  }

  // if (thread_params.count(handle) != 0) {
  //   thread_params.erase(handle);
  // }
}
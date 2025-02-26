#include "preempt_executor/preempt_executor.hpp"
#include "preempt_executor/primes_workload.hpp"
#include "preempt_executor/test_nodes.hpp"
#include "simple_timer/rt-sched.hpp"
#include <fstream>
#include <memory>
#include <ostream>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include <unistd.h>
#include <vector>

#define PUB1_PERIOD 200
#define PUB1_PERIOD_HI 100
#define PUB1_RUNTIME 40
#define WORKER1_RUNTIME 40

#define PUB2_PERIOD 200
#define PUB2_RUNTIME 40
#define WORKER2_RUNTIME 40

#define MS_TO_NS 1000000

#define USE_WORKERS 1

std::shared_ptr<PreemptExecutor> executor;
std::shared_ptr<rclcpp::Node> host_node;
std::shared_ptr<PublisherNode> publisher1;
std::shared_ptr<PublisherNode> publisher2;
#if USE_WORKERS
std::shared_ptr<WorkerNode> worker1;
std::shared_ptr<WorkerNode> worker2;
#endif
node_time_logger logger;

rclcpp::TimerBase::SharedPtr modeSwitchTimer;

ModeSwitchMode current_mode = ModeSwitchMode::LO;
size_t switches = 0;

void modeSwitch(ModeSwitchMode mode)
{
  if (mode == ModeSwitchMode::LO)
  {
    std::cout << "Mode switch to LO" << std::endl;
    // LO: normal operation - restore publisher2
    publisher2->setTimerPeriod(PUB2_PERIOD, host_node.get());
    // and set publisher1 back to 100ms
    publisher1->setTimerPeriod(PUB1_PERIOD, host_node.get());

    // and inform the executor
    // TODO: automate this
    executor->set_params(publisher1->timer_->get_timer_handle().get(),
                         PUB1_PERIOD * NS_TO_MS, PUB1_RUNTIME * NS_TO_MS,
                         PUB1_PERIOD * NS_TO_MS, "publisher1");
#if USE_WORKERS
    executor->set_params(worker1->sub_->get_subscription_handle().get(),
                         PUB1_PERIOD * NS_TO_MS, WORKER1_RUNTIME * NS_TO_MS,
                         PUB1_PERIOD * NS_TO_MS, "worker1");
#endif
    executor->set_params(publisher2->timer_->get_timer_handle().get(),
                         PUB2_PERIOD * NS_TO_MS, PUB2_RUNTIME * NS_TO_MS,
                         PUB2_PERIOD * NS_TO_MS, "publisher2");
  }
  else
  {
    std::cout << "Mode switch to HI" << std::endl;
    // HI: reduce the workload by stopping publisher2
    executor->dropTask(publisher2->timer_->get_timer_handle().get());
    publisher1->setTimerPeriod(PUB1_PERIOD_HI, host_node.get());

    executor->set_params(publisher1->timer_->get_timer_handle().get(),
                         PUB1_PERIOD_HI * NS_TO_MS, PUB1_RUNTIME * NS_TO_MS,
                         PUB1_PERIOD_HI * NS_TO_MS, "publisher1");
#if USE_WORKERS
    executor->set_params(worker1->sub_->get_subscription_handle().get(),
                         PUB1_PERIOD_HI * NS_TO_MS, WORKER1_RUNTIME * NS_TO_MS,
                         PUB1_PERIOD_HI * NS_TO_MS, "worker1");
#endif
  }
}

int main(int argc, char **argv)
{
  logger = create_logger();
  // cpu_set_t cpuset;
  // CPU_ZERO(&cpuset);
  // CPU_SET(1, &cpuset);
  // int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t),
  // &cpuset); if (rc != 0)
  // {
  //   std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
  // }

  int rc;
  // set to very high priority
  struct sched_param param;
  param.sched_priority = 99;
  rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (rc != 0)
  {
    std::cerr << "Error calling pthread_setschedparam: " << rc << std::endl;
  }

  int dummy_load_calib = 1;
  while (1)
  {
    timeval ctime, ftime;
    int duration_us;
    gettimeofday(&ctime, NULL);
    dummy_load(100); // 100ms
    gettimeofday(&ftime, NULL);
    duration_us =
      (ftime.tv_sec - ctime.tv_sec) * 1000000 + (ftime.tv_usec - ctime.tv_usec);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "dummy_load_calib: %d (duration_us: %d ns)", dummy_load_calib,
                duration_us);
    if (abs(duration_us - 100 * 1000) < 500)
    { // error margin: 500us
      break;
    }
    dummy_load_calib = 100 * 1000 * dummy_load_calib / duration_us;
    if (dummy_load_calib <= 0)
      dummy_load_calib = 1;
    DummyLoadCalibration::setCalibration(dummy_load_calib);
  }
  DummyLoadCalibration::setCalibration(dummy_load_calib * 0.85);
  param.sched_priority = 0;
  rc = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
  if (rc != 0)
  {
    std::cerr << "Error calling pthread_setschedparam: " << rc << std::endl;
  }

  rc = nice(-20);
  if (rc == -1)
  {
    std::cerr << "Error calling nice: " << rc << std::endl;
  }
  rclcpp::init(argc, argv);
  host_node = std::make_shared<rclcpp::Node>("test_nodes");
  // two chains, each with one publisher and one subscriber
  // publisher: 10ms run, 100ms period
  publisher1 = std::make_shared<PublisherNode>(
    "publisher1", "publisher1", PUB1_RUNTIME, PUB1_PERIOD, 0, host_node);
  // publisher: 10ms run, 100ms period
  publisher2 = std::make_shared<PublisherNode>(
    "publisher2", "publisher2", PUB2_RUNTIME, PUB2_PERIOD, 1, host_node);
#if USE_WORKERS
  // subscriber 1: 10ms runtime, 100ms period
  worker1 =
    std::make_shared<WorkerNode>("worker1", "publisher1", "worker1",
                                 WORKER1_RUNTIME, PUB1_PERIOD, 0, host_node);
  // subscriber 2: 10ms runtime, 100ms period
  worker2 =
    std::make_shared<WorkerNode>("worker2", "publisher2", "worker2",
                                 WORKER2_RUNTIME, PUB2_PERIOD, 0, host_node);
#endif

  std::vector<std::shared_ptr<CaseStudyNode>> callbacks;
  callbacks.push_back(publisher1);
  callbacks.push_back(publisher2);
#if USE_WORKERS
  callbacks.push_back(worker1);
  callbacks.push_back(worker2);
#endif

  modeSwitchTimer = host_node->create_wall_timer(
    std::chrono::milliseconds(1000),
    []()
    {
      std::ostringstream oss;

      oss << "{\"operation\": \"start_work\", "
             "\"chain\": "
          << 2

          << ", \"node\": \""
          << "mode_switch_timer"
          << "\", \"count\": " << switches << "}";
      log_entry(logger, oss.str());
      oss.str("");
      RCLCPP_INFO(rclcpp::get_logger("mode_switch_timer"),
                  "Mode switch timer callback");
      //  executor->requested_mode =
      //    ModeSwitchMode::HI;
      switches++;
      if (switches > 10)
      {
        rclcpp::shutdown();
      }
      if (current_mode == ModeSwitchMode::LO)
      {
        std::cout << "requesting mode switch to HI" << std::endl;
        current_mode = ModeSwitchMode::HI;
        executor->requested_mode = ModeSwitchMode::HI;
      }
      else
      {
        std::cout << "requesting mode switch to LO" << std::endl;
        current_mode = ModeSwitchMode::LO;
        executor->requested_mode = ModeSwitchMode::LO;
      }
      executor->requesting_mode_switch = true;
      int now = get_time_us() / 1000;
      executor->request_mode_switch_time = now;
      oss << "{\"operation\": \"end_work\", "
             "\"chain\": "
          << 2

          << ", \"node\": \""
          << "mode_switch_timer"
          << "\", \"count\": " << switches << "}";
      log_entry(logger, oss.str());
    });

  executor = std::make_shared<PreemptExecutor>();

  executor->add_node(host_node);

  executor->set_params(publisher1->timer_->get_timer_handle().get(),
                       PUB1_PERIOD * NS_TO_MS, PUB1_RUNTIME * NS_TO_MS,
                       PUB1_PERIOD * NS_TO_MS, "publisher1");
  executor->set_params(publisher2->timer_->get_timer_handle().get(),
                       PUB2_PERIOD * NS_TO_MS, PUB2_RUNTIME * NS_TO_MS,
                       PUB2_PERIOD * NS_TO_MS, "publisher2");
#if USE_WORKERS
  executor->set_params(worker1->sub_->get_subscription_handle().get(),
                       PUB1_PERIOD * NS_TO_MS, WORKER1_RUNTIME * NS_TO_MS,
                       PUB1_PERIOD * NS_TO_MS, "worker1");
  executor->set_params(worker2->sub_->get_subscription_handle().get(),
                       PUB2_PERIOD * NS_TO_MS, WORKER2_RUNTIME * NS_TO_MS,
                       PUB2_PERIOD * NS_TO_MS, "worker2");
#endif

  executor->set_params(modeSwitchTimer->get_timer_handle().get(),
                       1000 * NS_TO_MS, 1 * NS_TO_MS, 1000 * NS_TO_MS,
                       "mode_switch_timer");

  executor->mode_switch_callback = modeSwitch;

  executor->spin();

  std::cout << "spinning done" << std::endl;

  std::ofstream output_file;
  output_file.open("/root/ros2_ws/results/preempt_executor.txt");
  output_file << "[" << std::endl;
  for (auto &callback : callbacks)
  {
    for (auto &entry : *callback->logger.recorded_times)
    {
      output_file << "{\"entry\": " << entry.first
                  << ", \"time\": " << entry.second << "}," << std::endl;
    }
  }
  for (auto &entry : *logger.recorded_times)
  {
    output_file << "{\"entry\": " << entry.first
                << ", \"time\": " << entry.second << "}," << std::endl;
  }
  // remove the last comma
  output_file.seekp(-2, std::ios_base::end);
  output_file << "]" << std::endl;
  output_file.close();

  std::cout << "writing to file done" << std::endl;
  rclcpp::shutdown();

  return 0;
}
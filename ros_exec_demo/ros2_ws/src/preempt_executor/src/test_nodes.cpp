#include "preempt_executor/test_nodes.hpp"
#include "preempt_executor/primes_workload.hpp"
#include "simple_timer/rt-sched.hpp"
#include <functional>
#include <rcl/timer.h>

std::string CaseStudyNode::get_name() { return name; }

#define BUFFER_LENGTH 0

rclcpp::QoS get_qos()
{
  if (BUFFER_LENGTH == 0)
  {
    return rclcpp::QoS(rclcpp::KeepAll());
  }
  else
  {
    return rclcpp::QoS(rclcpp::KeepLast(BUFFER_LENGTH));
  }
}

PublisherNode::PublisherNode(std::string name, std::string publish_topic,
                             double runtime, int period, int chain,
                             rclcpp::Node::SharedPtr node,
                             rclcpp::CallbackGroup::SharedPtr callback_group)
    : CaseStudyNode(name, node), runtime_(runtime), period_(period),
      chain_(chain)
{
  logger = create_logger();
  pub_ = node->create_publisher<std_msgs::msg::Int32>(publish_topic, get_qos());
  if (callback_group != nullptr)
  {
    callback_group_ = callback_group;
  }
  else
  {
    callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }
  timer_ = node->create_wall_timer(
    std::chrono::milliseconds(period),
    std::bind(&PublisherNode::timer_callback, this), callback_group_);
}
void PublisherNode::timer_callback()
{
  std::ostringstream ss;
  ss << "{\"operation\": \"start_work\", \"chain\": " << chain_
     << ", \"node\": \"" << get_name() << "\", \"count\": " << node_counter;
  std::chrono::nanoseconds time_until_trigger = timer_->time_until_trigger();
  std::chrono::microseconds time_until_trigger_us =
    std::chrono::duration_cast<std::chrono::microseconds>(time_until_trigger);
  ss << ", \"next_release_us\": " << time_until_trigger_us.count() << "}";
  log_entry(logger, ss.str());
  // nth_prime_silly(runtime_);
  dummy_load(runtime_);
  std_msgs::msg::Int32 msg;
  msg.data = node_counter;
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
  pub_->publish(msg);
  // TEST: can we take more than one msg from DDS?
  // pub_->publish(msg);
  ss.str("");
  ss << "{\"operation\": \"end_work\", \"chain\": " << chain_
     << ", \"node\": \"" << get_name() << "\", \"count\": " << node_counter
     << "}";
  log_entry(logger, ss.str());
  node_counter++;
  // std::cout<<"count_max_ = "<<count_max_<<std::endl;
  if (node_counter >= count_max_)
  {
    rclcpp::shutdown();
  }
};

void PublisherNode::setTimerPeriod(int period_ms, rclcpp::Node *node)
{
  // timer_->cancel();
  // timer_ = node->create_wall_timer(
  //     std::chrono::milliseconds(period_ms),
  //     std::bind(&PublisherNode::timer_callback, this), callback_group_);
  // if (timer_->is_canceled()) {
  //   timer_ = node->create_wall_timer(
  //       std::chrono::milliseconds(period_ms),
  //       std::bind(&PublisherNode::timer_callback, this), callback_group_);
  // } else {
  //   int64_t old_period;
  //   int64_t new_period_ns = period_ms * 1000000;
  //   rcl_ret_t result = rcl_timer_exchange_period(
  //       timer_->get_timer_handle().get(), new_period_ns, &old_period);
  //   if (result != RCL_RET_OK) {
  //     RCLCPP_ERROR(node->get_logger(), "Failed to exchange timer period");
  //   } else {
  //     RCLCPP_INFO(node->get_logger(), "Exchanged timer period from %ld to
  //     %ld",
  //                 old_period, new_period_ns);
  //   }
  // }
  int64_t old_period;
  int64_t new_period_ns = period_ms * 1000000;
  rcl_ret_t result = rcl_timer_exchange_period(timer_->get_timer_handle().get(),
                                               new_period_ns, &old_period);
  if (result != RCL_RET_OK)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to exchange timer period");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Exchanged timer period from %ld to %ld",
                old_period, new_period_ns);
  }
}

WorkerNode::WorkerNode(std::string name, std::string subscribe_topic,
                       std::string publish_topic, double runtime, int period,
                       int chain, rclcpp::Node::SharedPtr node,
                       rclcpp::CallbackGroup::SharedPtr callback_group)
    : CaseStudyNode(name, node), runtime_(runtime), period_(period),
      chain_(chain)
{
  logger = create_logger();
  auto callback = [this](const std_msgs::msg::Int32::SharedPtr msg) -> void
  {
    // prevent unused variable warning
    (void)msg;

    std::ostringstream ss;
    ss << "{\"operation\": \"start_work\", \"chain\": " << chain_
       << ", \"node\": \"" << get_name() << "\", \"count\": " << msg->data
       << "}";
    log_entry(logger, ss.str());
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    // nth_prime_silly(runtime_);
    dummy_load(runtime_);
    // RCLCPP_INFO(this->get_logger(), "Result: %d", result);
    auto new_msg = std_msgs::msg::Int32();
    new_msg.data = msg->data;
    pub_->publish(new_msg);
    ss.str("");
    ss << "{\"operation\": \"end_work\", \"chain\": " << chain_
       << ", \"node\": \"" << get_name() << "\", \"count\": " << msg->data
       << "}";
    log_entry(logger, ss.str());
  };
  rclcpp::SubscriptionOptions sub_options;
  if (callback_group)
  {
    callback_group_ = callback_group;
  }
  else
  {
    callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  sub_options.callback_group = callback_group_;
  sub_ = node->create_subscription<std_msgs::msg::Int32>(
    subscribe_topic, get_qos(), callback, sub_options);
  pub_ = node->create_publisher<std_msgs::msg::Int32>(publish_topic, get_qos());
}

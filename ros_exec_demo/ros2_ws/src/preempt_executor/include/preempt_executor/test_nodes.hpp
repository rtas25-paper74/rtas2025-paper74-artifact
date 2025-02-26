#ifndef RTIS_TEST_NODES
#define RTIS_TEST_NODES
#include "simple_timer/rt-sched.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

class CaseStudyNode
{
public:
  node_time_logger logger;
  // inherit constructor
  CaseStudyNode(std::string name, rclcpp::Node::SharedPtr node)
  {
    this->name = name;
    this->node = node;
  }

  std::string get_name();
  rclcpp::CallbackGroup::SharedPtr callback_group_;

private:
  rclcpp::Node::SharedPtr node;
  std::string name;
};

// #define COUNT_MAX 100

class PublisherNode : public CaseStudyNode
{
public:
  PublisherNode(std::string name, std::string publish_topic, double runtime,
                int period, int chain, rclcpp::Node::SharedPtr node,
                rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);
  rclcpp::TimerBase::SharedPtr timer_;
  int node_counter = 0;

  void setTimerPeriod(int period, rclcpp::Node *node);
  void timer_callback();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  double runtime_;
  int period_;
  int chain_;
  int count_max_ = 9999;
};

class WorkerNode : public CaseStudyNode
{
public:
  WorkerNode(std::string name, std::string subscribe_topic,
             std::string publish_topic, double runtime, int period, int chain,
             rclcpp::Node::SharedPtr node,
             rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  int count_max_ = 0;

private:
  double runtime_;
  int period_;
  int chain_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

#endif // RTIS_TEST_NODES
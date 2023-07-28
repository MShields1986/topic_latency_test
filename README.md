# topic_latency_test
Package to test the latency of a ROS topic

This test includes delays due to:
- clock differences between the computers
- network latency
- latency between the header timestamp being populated and any other operations prior to it being sent (populating other fields etc.)
- queuing of messages on the receiver
- more?

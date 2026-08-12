#ifndef PX4_ROS_COM__TRACE_CONTROL_HPP_
#define PX4_ROS_COM__TRACE_CONTROL_HPP_

// Compatibility aliases.  The owning public contract is in the independent
// livox_trace_control_interfaces package, never in px4_ros_com.
#include <trace_control.hpp>

namespace px4_ros_com
{
using DurableTraceState = livox_trace_control::State;
using DurableTraceTerminal = livox_trace_control::Terminal;
using DurableTraceAck = livox_trace_control::Ack;
using DurableTraceControl = livox_trace_control::Control;
using livox_trace_control::escape;
using livox_trace_control::hex_decode;
using livox_trace_control::hmac_sha256;
using livox_trace_control::sha256;
using livox_trace_control::stage_counts_json;
inline std::string trace_json_escape(const std::string & value) {return escape(value);}
inline std::string trace_hex_decode(const std::string & value) {return hex_decode(value);}
inline std::string trace_hmac_sha256(const std::string & value, const std::string & key) {return hmac_sha256(value,key);}
inline std::string trace_sha256(const std::string & value) {return sha256(value);}
inline std::string trace_stage_counts_json(const std::map<std::string,std::uint64_t> & value) {return stage_counts_json(value);}
}

#endif

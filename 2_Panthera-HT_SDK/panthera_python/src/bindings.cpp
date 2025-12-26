#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "robot.hpp"
#include "motor.hpp"
#include "serial_driver.hpp"
#include "parse_robot_params.hpp"
#include "serial_struct.hpp"

namespace py = pybind11;
using namespace hightorque_robot;

PYBIND11_MODULE(_hightorque_robot, m) {
    m.doc() = "高扭矩机器人电机控制Python接口";

    // ==================== 枚举类型 ====================

    // 电机类型枚举
    py::enum_<motor_type>(m, "MotorType")
        .value("M3536_32", motor_type::m3536_32)
        .value("M4538_19", motor_type::m4538_19)
        .value("M5046_20", motor_type::m5046_20)
        .value("M5047_09", motor_type::m5047_09)
        .value("M5047_36", motor_type::m5047_36)
        .value("M5047_36_2", motor_type::m5047_36_2)
        .value("M4438_30", motor_type::m4438_30)
        .value("M4438_32", motor_type::m4438_32)
        .value("M6056_36", motor_type::m6056_36)
        .value("M5043_20", motor_type::m5043_20)
        .value("M7256_35", motor_type::m7256_35)
        .value("M60SG_35", motor_type::m60sg_35)
        .value("M60BM_35", motor_type::m60bm_35)
        .value("MGENERAL", motor_type::mGeneral)
        .export_values();

    // 位置/速度转换类型
    py::enum_<pos_vel_convert_type>(m, "PosVelConvertType")
        .value("RADIAN_2PI", pos_vel_convert_type::radian_2pi, "弧度制 (0-2π)")
        .value("ANGLE_360", pos_vel_convert_type::angle_360, "角度制 (0-360)")
        .value("TURNS", pos_vel_convert_type::turns, "圈数")
        .export_values();

    // ==================== 数据结构 ====================

    // 电机状态结构体
    py::class_<motor_back_t>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("time", &motor_back_t::time, "时间戳")
        .def_readwrite("ID", &motor_back_t::ID, "电机ID")
        .def_readwrite("mode", &motor_back_t::mode, "运行模式")
        .def_readwrite("fault", &motor_back_t::fault, "故障码")
        .def_readwrite("position", &motor_back_t::position, "位置 (弧度)")
        .def_readwrite("velocity", &motor_back_t::velocity, "速度 (弧度/秒)")
        .def_readwrite("torque", &motor_back_t::torque, "力矩 (N·m)")
        .def("__repr__", [](const motor_back_t &s) {
            return "<MotorState ID=" + std::to_string(s.ID) +
                   " pos=" + std::to_string(s.position) +
                   " vel=" + std::to_string(s.velocity) +
                   " torque=" + std::to_string(s.torque) + ">";
        });

    // 电机版本信息
    py::class_<cdc_rx_motor_version_s>(m, "MotorVersion")
        .def(py::init<>())
        .def_readwrite("id", &cdc_rx_motor_version_s::id, "电机ID")
        .def_readwrite("major", &cdc_rx_motor_version_s::major, "主版本号")
        .def_readwrite("minor", &cdc_rx_motor_version_s::minor, "次版本号")
        .def_readwrite("patch", &cdc_rx_motor_version_s::patch, "补丁版本号")
        .def("__repr__", [](const cdc_rx_motor_version_s &v) {
            return "<MotorVersion " + std::to_string(v.id) + " v" +
                   std::to_string(v.major) + "." +
                   std::to_string(v.minor) + "." +
                   std::to_string(v.patch) + ">";
        });

    // 配置参数结构体
    py::class_<MotorParams>(m, "MotorParams")
        .def(py::init<>())
        .def_readwrite("type", &MotorParams::type)
        .def_readwrite("id", &MotorParams::id)
        .def_readwrite("name", &MotorParams::name)
        .def_readwrite("num", &MotorParams::num)
        .def_readwrite("pos_limit_enable", &MotorParams::pos_limit_enable)
        .def_readwrite("pos_upper", &MotorParams::pos_upper)
        .def_readwrite("pos_lower", &MotorParams::pos_lower)
        .def_readwrite("tor_limit_enable", &MotorParams::tor_limit_enable)
        .def_readwrite("tor_upper", &MotorParams::tor_upper)
        .def_readwrite("tor_lower", &MotorParams::tor_lower);

    py::class_<CANPortParams>(m, "CANPortParams")
        .def(py::init<>())
        .def_readwrite("serial_id", &CANPortParams::serial_id)
        .def_readwrite("motor_num", &CANPortParams::motor_num)
        .def_readwrite("motors", &CANPortParams::motors);

    py::class_<CANBoardParams>(m, "CANBoardParams")
        .def(py::init<>())
        .def_readwrite("CANport_num", &CANBoardParams::CANport_num)
        .def_readwrite("CANports", &CANBoardParams::CANports);

    py::class_<RobotParams>(m, "RobotParams")
        .def(py::init<>())
        .def_readwrite("motor_timeout_ms", &RobotParams::motor_timeout_ms)
        .def_readwrite("robot_name", &RobotParams::robot_name)
        .def_readwrite("Serial_Type", &RobotParams::Serial_Type)
        .def_readwrite("Seial_baudrate", &RobotParams::Seial_baudrate)
        .def_readwrite("CANboard_num", &RobotParams::CANboard_num)
        .def_readwrite("CANboards", &RobotParams::CANboards);

    // ==================== Motor类 ====================

    py::class_<motor>(m, "Motor")
        // 注意: motor类的构造函数需要很多参数，不适合直接从Python创建
        // 应该从robot对象获取电机引用

        // 基础控制方法
        .def("position", &motor::position, py::arg("position"),
             "位置控制\n\n参数:\n  position: 目标位置 (弧度)")
        .def("velocity", &motor::velocity, py::arg("velocity"),
             "速度控制\n\n参数:\n  velocity: 目标速度 (弧度/秒)")
        .def("torque", &motor::torque, py::arg("torque"),
             "力矩控制\n\n参数:\n  torque: 目标力矩 (N·m)")
        .def("voltage", &motor::voltage, py::arg("voltage"),
             "电压控制\n\n参数:\n  voltage: 目标电压 (V)")
        .def("current", &motor::current, py::arg("current"),
             "电流控制\n\n参数:\n  current: 目标电流 (A)")

        // 混合控制方法
        .def("pos_vel_MAXtqe", &motor::pos_vel_MAXtqe,
             py::arg("position"), py::arg("velocity"), py::arg("torque_max"),
             "位置+速度+最大力矩控制\n\n"
             "参数:\n"
             "  position: 目标位置 (弧度)\n"
             "  velocity: 目标速度 (弧度/秒)\n"
             "  torque_max: 最大力矩限制 (N·m)")
        .def("pos_vel_tqe_kp_kd", &motor::pos_vel_tqe_kp_kd,
             py::arg("position"), py::arg("velocity"), py::arg("torque"),
             py::arg("kp"), py::arg("kd"),
             "五参数控制: 位置+速度+力矩+Kp+Kd\n\n"
             "参数:\n"
             "  position: 目标位置 (弧度)\n"
             "  velocity: 目标速度 (弧度/秒)\n"
             "  torque: 前馈力矩 (N·m)\n"
             "  kp: PID比例参数\n"
             "  kd: PID微分参数")
        .def("pos_vel_kp_kd", &motor::pos_vel_kp_kd,
             py::arg("position"), py::arg("velocity"), py::arg("kp"), py::arg("kd"),
             "位置+速度+PID参数控制")
        .def("pos_vel_acc", &motor::pos_vel_acc,
             py::arg("position"), py::arg("velocity"), py::arg("acc"),
             "位置+速度+加速度控制")

        // 操作方法
        .def("stop", &motor::stop, "停止电机")
        .def("brake", &motor::brake, "刹车电机")
        .def("reset", &motor::reset, "重启电机")
        .def("send_state_cmd", &motor::send_state_cmd, "发送状态查询命令")

        // 查询方法
        .def("get_motor_id", &motor::get_motor_id, "获取电机ID")
        .def("get_motor_enum_type", &motor::get_motor_enum_type, "获取电机类型")
        .def("get_motor_num", &motor::get_motor_num, "获取电机编号")
        .def("get_motor_name", &motor::get_motor_name, "获取电机名称")
        .def("get_current_motor_state", &motor::get_current_motor_state,
             py::return_value_policy::reference,
             "获取当前电机状态\n\n返回: MotorState对象")
        .def("get_version", &motor::get_version,
             py::return_value_policy::reference,
             "获取电机版本信息\n\n返回: MotorVersion对象")

        // 限制标志 (只读)
        .def_readonly("pos_limit_flag", &motor::pos_limit_flag,
                     "位置限制标志: 0=正常, 1=超出上限, -1=超出下限")
        .def_readonly("tor_limit_flag", &motor::tor_limit_flag,
                     "力矩限制标志: 0=正常, 1=超出上限")

        .def("__repr__", [](motor &m) {
            return "<Motor ID=" + std::to_string(m.get_motor_id()) +
                   " name='" + m.get_motor_name() + "'>";
        });

    // ==================== Robot类 ====================

    py::class_<robot>(m, "Robot")
        .def(py::init<>(),
             "创建机器人实例 (使用默认配置)")
        .def(py::init<const std::string&>(),
             py::arg("config_path"),
             "创建机器人实例\n\n"
             "参数:\n"
             "  config_path: YAML配置文件路径")

        // 电机控制
        .def("motor_send_cmd", &robot::motor_send_cmd,
             "发送电机控制命令到所有电机\n"
             "必须在设置电机控制参数后调用此方法")
        .def("set_stop", &robot::set_stop,
             "停止所有电机")
        .def("set_reset", &robot::set_reset,
             "重启所有电机")
        .def("send_get_motor_state_cmd", &robot::send_get_motor_state_cmd,
             "发送状态查询命令到所有电机")
        .def("send_get_motor_version_cmd", &robot::send_get_motor_version_cmd,
             "发送状态查询命令到所有电机")
        .def("set_reset_zero",
             static_cast<void (robot::*)()>(&robot::set_reset_zero),
             "重置所有电机零点")
        .def("set_reset_zero_motors",
             static_cast<void (robot::*)(std::initializer_list<int>)>(&robot::set_reset_zero),
             py::arg("motor_ids"),
             "重置指定电机零点\n\n参数:\n  motor_ids: 电机ID列表")
        .def("set_timeout",
             static_cast<void (robot::*)(int16_t)>(&robot::set_timeout),
             py::arg("timeout_ms"),
             "设置电机超时时间\n\n参数:\n  timeout_ms: 超时时间 (毫秒)")

        // LCM相关
        .def("lcm_enable", &robot::lcm_enable,
             "启用LCM消息发布\n"
             "启动后台线程发布机器人状态到LCM")
        .def("publishJointStates", &robot::publishJointStates,
             "手动发布一次关节状态到LCM")

        // 高级功能
        .def("motor_version_detection", &robot::motor_version_detection,
             "检测所有电机的版本信息")
        .def("canboard_fdcan_reset", &robot::canboard_fdcan_reset,
             "重新初始化FDCAN通信")

        // 访问器
        .def("get_motors", [](robot &r) {
            return r.Motors;
        }, py::return_value_policy::reference,
           "获取所有电机对象列表\n\n返回: Motor对象列表")

        .def("get_motor_by_id", [](robot &r, int motor_id) -> motor* {
            for (auto* m : r.Motors) {
                if (m->get_motor_id() == motor_id) {
                    return m;
                }
            }
            throw std::runtime_error("找不到ID为 " + std::to_string(motor_id) + " 的电机");
        }, py::arg("motor_id"),
           py::return_value_policy::reference,
           "根据ID获取电机对象\n\n参数:\n  motor_id: 电机ID\n\n返回: Motor对象")

        .def("get_motor_by_name", [](robot &r, const std::string& name) -> motor* {
            for (auto* m : r.Motors) {
                if (m->get_motor_name() == name) {
                    return m;
                }
            }
            throw std::runtime_error("找不到名为 '" + name + "' 的电机");
        }, py::arg("name"),
           py::return_value_policy::reference,
           "根据名称获取电机对象\n\n参数:\n  name: 电机名称\n\n返回: Motor对象")

        .def_readonly("robot_params", &robot::robot_params,
                     "机器人配置参数 (RobotParams对象)")
        .def_readonly("motor_timeout_ms", &robot::motor_timeout_ms,
                     "电机超时时间 (毫秒)")

        .def("__repr__", [](robot &r) {
            return "<Robot '" + r.robot_params.robot_name +
                   "' with " + std::to_string(r.Motors.size()) + " motors>";
        });

    // ==================== 辅助函数 ====================

    m.def("parse_robot_params", &parseRobotParams,
          py::arg("file_path"),
          "从YAML文件解析机器人参数\n\n"
          "参数:\n"
          "  file_path: YAML配置文件路径\n\n"
          "返回: RobotParams对象");

    // 版本信息
    m.attr("__version__") = "1.0.0";
    m.attr("__cpp_sdk_version__") = "4.4.7";
}

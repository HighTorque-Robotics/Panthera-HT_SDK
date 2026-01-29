#ifndef _SIGNAL_HANDLER_HPP_
#define _SIGNAL_HANDLER_HPP_

#include <atomic>
#include <functional>
#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

namespace panthera
{

/**
 * @brief 信号处理辅助类
 *
 * 提供跨平台的信号处理功能，用于优雅地处理 Ctrl+C 等信号
 */
class SignalHandler
{
public:
    /**
     * @brief 获取单例实例
     * @return SignalHandler实例的引用
     */
    static SignalHandler& getInstance()
    {
        static SignalHandler instance;
        return instance;
    }

    /**
     * @brief 初始化信号处理器
     * @param user_callback 用户自定义回调函数（可选）
     */
    void init(std::function<void()> user_callback = nullptr)
    {
        user_callback_ = user_callback;
        std::signal(SIGINT, SignalHandler::signalHandler);
        std::signal(SIGTERM, SignalHandler::signalHandler);
    }

    /**
     * @brief 检查是否应该退出
     * @return true表示应该退出
     */
    bool shouldExit() const
    {
        return exit_flag_.load();
    }

    /**
     * @brief 重置退出标志
     */
    void reset()
    {
        exit_flag_.store(false);
    }

    /**
     * @brief 设置退出标志
     * @param flag 退出标志值
     */
    void setExitFlag(bool flag)
    {
        exit_flag_.store(flag);
    }

    /**
     * @brief 等待退出信号（阻塞）
     */
    void waitForExit()
    {
        while (!exit_flag_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 删除拷贝构造和赋值操作
    SignalHandler(const SignalHandler&) = delete;
    SignalHandler& operator=(const SignalHandler&) = delete;

private:
    SignalHandler() : exit_flag_(false), user_callback_(nullptr) {}

    ~SignalHandler() = default;

    /**
     * @brief 静态信号处理函数
     * @param signum 信号编号
     */
    static void signalHandler(int signum)
    {
        auto& instance = getInstance();
        instance.exit_flag_.store(true);

        if (signum == SIGINT) {
            std::cout << "\n收到 SIGINT 信号 (Ctrl+C)，正在退出..." << std::endl;
        } else if (signum == SIGTERM) {
            std::cout << "\n收到 SIGTERM 信号，正在退出..." << std::endl;
        } else {
            std::cout << "\n收到信号 " << signum << "，正在退出..." << std::endl;
        }

        // 执行用户回调
        if (instance.user_callback_) {
            instance.user_callback_();
        }
    }

    std::atomic<bool> exit_flag_;
    std::function<void()> user_callback_;
};

} // namespace panthera

#endif // _SIGNAL_HANDLER_HPP_

#pragma once

#include "mbed.h"
#include <chrono>
#include <memory>
#include <string>
#include <string_view>
#include <vector>
#if !defined(__CORTEX_M0PLUS) && !defined(__CORTEX_M0)
#include <atomic>
#endif

#ifndef RISING_EDGE
#define RISING_EDGE           0x00100000U
#endif
#ifndef FALLING_EDGE
#define FALLING_EDGE          0x00200000U
#endif

extern UART_HandleTypeDef uart_handlers[];
extern "C" int8_t get_uart_index(UARTName uart_name);

namespace stm32
{
    struct CriticalContext final
    {
        CriticalContext()
        {
            __disable_irq();
        }

        ~CriticalContext()
        {
            __enable_irq();
        }
    };
	
	//! Lock/Unlock Mutex using variable scope.
	//! Ensures always unlocked regardless of method exit
	class MutexLocker final
	{
	public:
	    //! Lock the mutex
	    MutexLocker(Mutex &mutex) :
	        m_mutex(mutex)
	    {
	        m_mutex.lock();
	    }

	    //! Unlocks on destruction
	    ~MutexLocker()
	    {
	        m_mutex.unlock();
	    }
	    
	private:
	    Mutex &m_mutex;
	    // Disable copy
	    MutexLocker & operator=(const MutexLocker&) = delete;
	    MutexLocker(const MutexLocker &) = delete;
	};

    class DebouncedButton final
    {
    public:
        // debounce_time：消抖时间
        DebouncedButton(PinName pin, uint32_t trigger = FALLING_EDGE, chrono::microseconds debounce_time = 5ms, bool pull_mode = false, chrono::milliseconds min_interval = 200ms)
            : key(pin), debounce_time(debounce_time), min_interval(min_interval)
        {
            if(trigger == RISING_EDGE)
            {
                if(pull_mode)
                {
                    key.mode(PinMode::PullDown);
                }
                key.rise([this](){ this->on_rise(); });
            }
            else
            {
                if(pull_mode)
                {
                    key.mode(PinMode::PullUp);
                }
                key.fall([this](){ this->on_fall(); });
            }
        }

        bool query_and_reset()
        {
#if defined(__CORTEX_M0PLUS) || defined(__CORTEX_M0) || defined(__CORTEX_M3)
            bool pressed_ = pressed;
            pressed = false;
            return pressed_;
#else
            return pressed.exchange(false);
#endif
        }

    private:
        void on_fall()
        {
            if(enabled)
            {
                if(debounce_time != 0ms)
                {
                    key_timeout.attach([this](){
                        if(!key.read())
                        {
                            pressed = true;
                            if(min_interval != 0ms)
                            {
                                enabled = false;
                                key_timeout.attach([this](){ enabled = true; }, min_interval);
                            }
                        }
                    }, debounce_time);
                }
                else
                {
                    pressed = true;
                    if(min_interval != 0ms)
                    {
                        enabled = false;
                        key_timeout.attach([this](){ enabled = true; }, min_interval);
                    }
                }
            }
        }

        void on_rise()
        {
            if(enabled)
            {
                if(debounce_time != 0ms)
                {
                    key_timeout.attach([this](){
                        if(key.read())
                        {
                            pressed = true;
                            if(min_interval != 0ms)
                            {
                                enabled = false;
                                key_timeout.attach([this](){ enabled = true; }, min_interval);
                            }
                        }
                    }, debounce_time);
                }
                else
                {
                    pressed = true;
                    if(min_interval != 0ms)
                    {
                        enabled = false;
                        key_timeout.attach([this](){ enabled = true; }, min_interval);
                    }
                }
            }
        }

        InterruptIn key;
        Timeout key_timeout;
        const chrono::microseconds debounce_time;
        const chrono::milliseconds min_interval;
#if defined(__CORTEX_M0PLUS) || defined(__CORTEX_M0)
        __IO bool pressed = false;
        __IO bool enabled = true;
#else
        std::atomic<bool> pressed { false };
        std::atomic<bool> enabled { true };
#endif
    };

    // 自带直线缓冲区的串口类
    class LinearBufferedSerial : 
        public SerialBase,
        public FileHandle,
        private NonCopyable<LinearBufferedSerial>
    {
    public:
        LinearBufferedSerial(PinName tx, PinName rx, int bauds = 115200, size_t buffer_size = 256, PinName direction = PinName::NC)
            : SerialBase(tx, rx, bauds), buffer_size(buffer_size), rx_buffer(buffer_size), rs485_wait_time(((20736000L / bauds) + 999) / 1000)
        {
            if(direction != PinName::NC)
            {
                p_direction = std::make_unique<DigitalOut>(direction);
                p_direction->write(0); // 默认低电平，接收模式
            }
            attach(callback(this, &LinearBufferedSerial::on_rx_interrupt), ::SerialBase::RxIrq);
        }

        LinearBufferedSerial(const serial_pinmap_t &static_pinmap, int bauds = 115200, size_t buffer_size = 256, PinName direction = PinName::NC)
            : SerialBase(static_pinmap, bauds), buffer_size(buffer_size), rx_buffer(buffer_size), rs485_wait_time(((20736000L / bauds) + 999) / 1000)
        {
            if(direction != PinName::NC)
            {
                p_direction = std::make_unique<DigitalOut>(direction);
                p_direction->write(0); // 默认低电平，接收模式
            }
            attach(callback(this, &LinearBufferedSerial::on_rx_interrupt), ::SerialBase::RxIrq);
            if(STM_PIN_OD(static_pinmap.tx_function))
            {
                half_duplex_mode = true;
                int uart_index = get_uart_index(static_cast<UARTName>(static_pinmap.peripheral));
                uart_handle = &uart_handlers[uart_index];
                HAL_HalfDuplex_Init(uart_handle);
                HAL_HalfDuplex_EnableReceiver(uart_handle);
            }
        }

        int get_direction()
        {
            if(p_direction)
                return p_direction->read();
            else
                return 0;
        }

        bool is_empty() const
        {
            return current_length == 0;
        }

        size_t get_length() const
        {
            return current_length;
        }

        size_t get_buffer_size() const
        {
            return buffer_size;
        }

        void clear()
        {
            current_length = 0;
        }

        size_t length() const
        {
            return current_length;
        }

        std::string& append_and_clear(std::string& str)
        {
            if(current_length != 0)
            {
                CriticalContext ctx;
#if defined(__CORTEX_M0PLUS) || defined(__CORTEX_M0) || defined(__CORTEX_M3)
                size_t length = current_length;
                current_length = 0;
#else
                size_t length = current_length.exchange(size_t(0));
#endif
                str += std::string_view(&rx_buffer[0], length);
            }
            return str;
        }

        bool find_and_clear(const char ch)
        {
            bool found = false;
            if(current_length != 0)
            {
                CriticalContext ctx;
#if defined(__CORTEX_M0PLUS) || defined(__CORTEX_M0) || defined(__CORTEX_M3)                
                size_t length = current_length;
                current_length = 0;
#else
                size_t length = current_length.exchange(size_t(0));
#endif
                for(size_t i = 0; i < length; i++)
                {
                    if(rx_buffer[i] == ch)
                    {
                        found = true;
                        break;
                    }
                }
            }
            return found;
        }

        void write(const char ch)
        {
            if(p_direction)
            {
                p_direction->write(1);
                write(reinterpret_cast<const void*>(&ch), 1);
                ThisThread::sleep_for(rs485_wait_time);
                p_direction->write(0);
            }
            else
            {
                write(&ch, 1);
            }
        }

        void write(const char *str, size_t length)
        {
            if(p_direction)
            {
                p_direction->write(1);
                write(reinterpret_cast<const void*>(str), length);
                ThisThread::sleep_for(rs485_wait_time);
                p_direction->write(0);
            }
            else
            {
                write((const void *)str, length);
            }
        }

        void write(const std::string_view& str)
        {
            if(p_direction)
            {
                p_direction->write(1);
                write(reinterpret_cast<const void*>(&str[0]), str.length());
                ThisThread::sleep_for(rs485_wait_time);
                p_direction->write(0);
            }
            else
            {
                write((const void *)&str[0], str.length());
            }
        }

        void nl()
        {
            if(p_direction)
            {
                p_direction->write(1);
                write(reinterpret_cast<const void*>("\r\n"), 2);
                ThisThread::sleep_for(rs485_wait_time);
                p_direction->write(0);
            }
            else
            {
                write((const void *)"\r\n", 2);
            }
        }

        off_t seek(off_t offset, int whence = SEEK_SET) override
        {
            return -ESPIPE;
        }

        off_t size() override
        {
            return -EINVAL;
        }

        int isatty() override
        {
            return true;
        }

        int close() override
        {
            return 0;
        }

        int enable_input(bool enabled) override
        {
            SerialBase::enable_input(enabled);
            return 0;
        }

        int enable_output(bool enabled) override
        {
            SerialBase::enable_output(enabled);
            return 0;
        }

        short poll(short events) const override
        {
            short revents = 0;
            if ((events & POLLIN)
                && (const_cast <LinearBufferedSerial *>(this))->SerialBase::readable()
            ) {
                revents |= POLLIN;
            }
            if (
                (events & POLLOUT)
                && (const_cast <LinearBufferedSerial *>(this))->SerialBase::writeable()
            ) {
                revents |= POLLOUT;
            }
            return revents;
        }

        using SerialBase::attach;
        using SerialBase::baud;
        using SerialBase::format;
        using SerialBase::readable;
        using SerialBase::writeable;
        using SerialBase::IrqCnt;
        using SerialBase::RxIrq;
        using SerialBase::TxIrq;

    private:
        ssize_t read(void *buffer, size_t size) override
        {
            unsigned char *buf = static_cast<unsigned char *>(buffer);
            if (size == 0) {
                return 0;
            }
            lock();
            buf[0] = _base_getc();
            unlock();
            return 1;
        }

        ssize_t write(const void *buffer, size_t size) override
        {
            const unsigned char *buf = static_cast<const unsigned char *>(buffer);
            if (size == 0) {
                return 0;
            }
            bool lock_api = !core_util_in_critical_section();
            if (lock_api) {
                lock();
            }
            if(half_duplex_mode && uart_handle != nullptr)
            {
                HAL_HalfDuplex_EnableTransmitter(uart_handle);
            }
            for (size_t i = 0; i < size; i++) {
                _base_putc(buf[i]);
            }
            if(half_duplex_mode && uart_handle != nullptr)
            {
                while(!writeable());
                HAL_HalfDuplex_EnableReceiver(uart_handle);
            }
            if (lock_api) {
                unlock();
            }
            return size;
        }

        void on_rx_interrupt()
        {
            unsigned char ch;
            lock();
            while(serial_readable(&_serial))
            {
                ch = serial_getc(&_serial);
                if(current_length < buffer_size)
                {
                    rx_buffer[current_length++] = static_cast<char>(ch);
                }
            }
            unlock();
        }

        std::vector<char> rx_buffer;
        const size_t buffer_size;
        Kernel::Clock::duration_u32 rs485_wait_time;
        UART_HandleTypeDef *uart_handle = nullptr;
        bool half_duplex_mode = false;
#if defined(__CORTEX_M0PLUS) || defined(__CORTEX_M0)
        __IO size_t current_length = 0;
#else
        std::atomic<size_t> current_length { 0 };
#endif
        std::unique_ptr<DigitalOut> p_direction;
    };

    inline std::vector<std::string>& split_string(const std::string& str, std::vector<std::string>& output, const std::string& delims = " ")
    {
        output.clear();

        auto first = std::cbegin(str);
        
        while (first != std::cend(str))
        {
            const auto second = std::find_first_of(first, std::cend(str), 
                    std::cbegin(delims), std::cend(delims));

            if (first != second)
                output.emplace_back(first, second);

            if (second == std::cend(str))
                break;

            first = std::next(second);
        }

        return output;
    }

    inline std::vector<std::string_view>& split_string_view(std::string_view strv, std::vector<std::string_view>& output, std::string_view delims = " ")
    {
        output.clear();

        size_t first = 0;

        while (first < strv.size())
        {
            const auto second = strv.find_first_of(delims, first);

            if (first != second)
                output.emplace_back(strv.substr(first, second-first));

            if (second == std::string_view::npos)
                break;

            first = second + 1;
        }

        return output;
    }

    inline void hybrid_sleep(chrono::microseconds duration)
    {
        auto micros_count = duration.count();
        if(micros_count < 1000)
        {
            wait_us(micros_count);
        }
        else
        {
            Timer t;
            auto millis_count = micros_count / 1000;
            t.start();
            ThisThread::sleep_for(chrono::milliseconds(millis_count));
            t.stop();
            int micros_count = t.elapsed_time().count() - millis_count * 1000;
            if(micros_count > 0)
            {
                wait_us(micros_count);
            }
        }
    }

    template<typename SerialType>
    inline bool read_line(SerialType& serial, std::string& result, chrono::milliseconds timeout = 500ms)
    {
#if defined(__CORTEX_M0PLUS) || defined(__CORTEX_M0)
        __IO bool done = false;
#else
        std::atomic_bool done { false };
#endif
        Timeout to;
        to.attach([&done]() { done = true; }, timeout);
        char ch;
        while(!done)
        {
            if(serial.readable() && serial.read(&ch, 1) > 0)
            {
                result += ch;
                if(ch == '\n')
                {
                    to.detach();
                    return true;
                }
                continue;
            }
            ThisThread::yield();
        }
        return false;
    }

    template<typename char_t>
    inline bool starts_with(const std::basic_string<char_t> &full_string, const basic_string<char_t> &starting)
    {
        if (full_string.length() >= starting.length())
        {
            return (0 == full_string.compare(0, starting.length(), starting));
        }
        else
        {
            return false;
        }
    }

    template<typename char_t>
    inline bool starts_with(const std::basic_string_view<char_t> &full_string, const basic_string_view<char_t> &starting)
    {
        if (full_string.length() >= starting.length())
        {
            return (0 == full_string.compare(0, starting.length(), starting));
        }
        else
        {
            return false;
        }
    }

    template<typename char_t>
    inline bool starts_with(const std::basic_string_view<char_t> &full_string, const char_t *starting)
    {
        return starts_with(full_string, std::basic_string_view<char_t>(starting));
    }

    template<typename char_t, typename range_t>
    inline bool starts_with(const std::basic_string<char_t> &full_string, const range_t &starting)
    {
        std::basic_string<char_t> starting_string(starting);
        if (full_string.length() >= starting_string.length())
        {
            return (0 == full_string.compare(0, starting_string.length(), starting_string));
        }
        else
        {
            return false;
        }
    }

    template<typename char_t>
    inline bool ends_with(const basic_string<char_t> &full_string, const basic_string<char_t> &ending)
    {
        if (full_string.length() >= ending.length())
        {
            return (0 == full_string.compare(full_string.length() - ending.length(), ending.length(), ending));
        }
        else
        {
            return false;
        }
    }

    template<typename char_t>
    inline bool ends_with(const basic_string_view<char_t> &full_string, const basic_string_view<char_t> &ending)
    {
        if (full_string.length() >= ending.length())
        {
            return (0 == full_string.compare(full_string.length() - ending.length(), ending.length(), ending));
        }
        else
        {
            return false;
        }
    }

    template<typename char_t>
    inline bool ends_with(const basic_string_view<char_t> &full_string, const char_t *ending)
    {
        return ends_with(full_string, std::basic_string_view<char_t>(ending));
    }

    template<typename char_t, typename range_t>
    inline bool ends_with(const std::basic_string<char_t> &full_string, const range_t &ending)
    {
        std::basic_string<char_t> ending_string(ending);
        if (full_string.length() >= ending_string.length())
        {
            return (0 == full_string.compare(full_string.length() - ending_string.length(), ending_string.length(), ending_string));
        }
        else
        {
            return false;
        }
    }

#if MBED_CONF_FILESYSTEM_PRESENT
    inline bool directory_exists(FileSystem *_fs, const char *filename)
    {
        Dir _dir;
        int err = _dir.open(_fs, filename);
        if(err != 0)
        {
            return false;
        }
        _dir.close();
        return true;
    }

    inline bool file_exists(FileSystem *_fs, const char *filename)
    {
        struct stat _stat;
        return _fs->stat(filename, &_stat) == 0;
    }

    inline long get_file_size(FileSystem *_fs, const char *filename)
    {
        struct stat _stat;
        if(_fs->stat(filename, &_stat) != 0)
        {
            return 0;
        }
        return _stat.st_size;
    }

    inline bool copy_file(FileSystem *source_fs, const char *source_filename, FileSystem *target_fs, const char *target_filename)
    {
        struct stat _stat;
        if(source_fs->stat(source_filename, &_stat) != 0)
        {
            return false;
        }
        off_t source_size = _stat.st_size;
        File source_file(source_fs, source_filename, O_BINARY);
        File target_file;
        if(target_file.open(target_fs, target_filename, O_BINARY|O_CREAT|O_TRUNC) != 0)
        {
            return false;
        }
        constexpr off_t copy_buffer_size = 1024;
        char copy_buffer[copy_buffer_size];
        while(source_size > 0)
        {
            off_t batch_size = source_size > copy_buffer_size ? copy_buffer_size: source_size;
            source_file.read(copy_buffer, batch_size);
            target_file.write(copy_buffer, batch_size);
            source_size -= batch_size;
        }
        return true;
    }

    inline std::string join_path(const std::string_view& dir_path, const char *file_name)
    {
        std::string joined(dir_path.cbegin(), dir_path.cend());
        if(joined.empty())
        {
            joined = "/";
        }
        else if(*joined.cend() != '/')
        {
            joined.push_back('/');
        }
        joined += file_name;
        return joined;
    }
#endif

} // namespace stm32

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <numeric>
#include <signal.h>
#include <stdexcept>
#include <thread>
#include <experimental/source_location>

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <pigpio.h>

// adjust these:
constexpr const unsigned int pwm_pin = 18;
constexpr const unsigned int min_speed = 50;
constexpr const unsigned int low_temp = 30;
constexpr const unsigned int high_temp = 50;
constexpr const unsigned int tacho_pin = 0;


using namespace std::chrono_literals;
constexpr const auto check_timeout = 2s;
constexpr const unsigned int temp_average_interval = 5;
constexpr const unsigned int pwm_range = 100;

constexpr bool is_hw_pwm_pin(unsigned int pin)
{
    return pin == 12 or pin == 13 or pin == 18 or pin == 19;
    //return false;
}

int gpio_throw_on_error(int error_code, const std::experimental::source_location location = std::experimental::source_location::current())
{
    if (error_code < 0)
    {
        std::runtime_error(
          fmt::format(
            "GPIO error with code {} at \nfile: {}({}:{}) `{}`: \n",
            error_code,
            location.file_name(),
            location.line(),
            location.column(),
            location.function_name()));
    }
    return error_code;
}

template<unsigned int size>
struct Temperature_Average
{
    Temperature_Average(float initial_value)
    {
        std::fill_n(begin(values), size, initial_value);
    }

    std::array<float, size> values;
    unsigned int index = 0;

    float new_value(float value)
    {
        values[index] = value;
        index = (index + 1) % size;
        return std::accumulate(values.begin(), values.end(), 0.f) / size;
    }
};

struct PIGPIOHandler
{
    PIGPIOHandler()
    {
        // gpio_throw_on_error(
        //   gpioCfgClock(10, 1, 0));
        uint32_t cfg = gpioCfgGetInternals();
        cfg |= PI_CFG_NOSIGHANDLER;
        cfg |= 1;
        gpioCfgSetInternals(cfg);
        gpio_throw_on_error(
          gpioInitialise());
        errno = 0; // pigpio checks its lock file, which will not exist and does not clean errno
    }

    ~PIGPIOHandler()
    {
        gpioTerminate();
    }
};

struct Pin
{
    Pin(unsigned int id_in) : id(id_in)
    {}

    virtual ~Pin()
    {
        gpio_throw_on_error(
          gpioSetMode(id, PI_INPUT));

        gpio_throw_on_error(
          gpioSetPullUpDown(id, PI_PUD_OFF));
    }

    unsigned int id;
};

struct PWM_Pin : public Pin
{
    PWM_Pin() : Pin(pwm_pin)
    {
        if constexpr (!is_hw_pwm_pin(pwm_pin))
        {
            gpio_throw_on_error(
              gpioSetPWMfrequency(id, 25000));
            gpio_throw_on_error(
              gpioSetPWMrange(id, pwm_range));
        }
        set(100);
    }

    ~PWM_Pin() override = default;

    void set(unsigned int value)
    {
        if constexpr (is_hw_pwm_pin(pwm_pin))
        {
            gpio_throw_on_error(
              gpioHardwarePWM(id, static_cast<unsigned int>(25000), value*10000));
        }
        else
        {
            gpio_throw_on_error(
                gpioPWM(id, value));
        }
    }
};

std::atomic<int> rpm_pulses = 0;
void inc_pulses(int, int, uint32_t)
{
    rpm_pulses++;
}
struct Tacho_Pin : public Pin
{
    explicit Tacho_Pin(unsigned int id_in) : Pin(id_in)
    {
        gpio_throw_on_error(
          gpioSetMode(id, PI_INPUT));
        gpio_throw_on_error(
          gpioSetPullUpDown(id, PI_PUD_OFF));
        gpio_throw_on_error(
          gpioSetISRFunc(id, FALLING_EDGE, 500, inc_pulses));
        last_measurement = std::chrono::steady_clock::now();
    }

    int read()
    {
        constexpr const double revolutions_per_pulse = 0.5;
        int pulses = rpm_pulses.exchange(0);
        auto this_measurement = std::chrono::steady_clock::now();
        std::chrono::nanoseconds duration = this_measurement - last_measurement;
        double seconds = static_cast<double>(duration.count()) * 1e-9;
        last_measurement = this_measurement;
        auto pulses_per_second = pulses / seconds;
        return static_cast<int>(pulses_per_second * 60 * revolutions_per_pulse);
    }

    std::chrono::time_point<std::chrono::steady_clock> last_measurement;
};

struct CPU_Temperature_Sensor
{
    std::ifstream temperature_device = std::ifstream("/sys/class/thermal/thermal_zone0/temp");

    float operator()()
    {
        float temp;
        temperature_device >> temp;
        temperature_device.seekg(0);
        return temp / 1000;
    }
};

std::condition_variable cond;
std::mutex m;
bool running = true;
int main()
{

    PIGPIOHandler pigpio_handler;

    struct sigaction sigHandler;
    memset(&sigHandler, 0, sizeof(sigHandler));
    sigHandler.sa_handler = [](int /*signal*/) {
        running = false;
        cond.notify_one();
    };
    sigfillset(&sigHandler.sa_mask);

    sigaction(SIGINT, &sigHandler, NULL);
    sigaction(SIGSEGV, &sigHandler, NULL);
    sigaction(SIGILL, &sigHandler, NULL);
    sigaction(SIGFPE, &sigHandler, NULL);
    sigaction(SIGABRT, &sigHandler, NULL);
    sigaction(SIGFPE, &sigHandler, NULL);
    sigaction(SIGTERM, &sigHandler, NULL);

    auto calc_pwm_duty_from_temperature = [&]() {
        const float low_temp_f = static_cast<float>(low_temp);
        const int linear_range = pwm_range - min_speed;
        const float temp_diff_range_factor = static_cast<float>(linear_range / (high_temp - low_temp));

        return [=](float temp) {
            float diff = temp - low_temp_f;
            auto linear_part = static_cast<unsigned int>(diff * temp_diff_range_factor);
            return std::min(std::max(linear_part, static_cast<unsigned int>(0)) + min_speed, pwm_range);
        };
    }();
    try
    {
        CPU_Temperature_Sensor temp_sens;
        auto pwm_pin_obj = PWM_Pin();
        std::unique_ptr<Tacho_Pin> tacho;
        if(tacho_pin > 0)
            tacho = std::make_unique<Tacho_Pin>(tacho_pin);
        Temperature_Average<temp_average_interval> average(temp_sens());
        while (running)
        {
            auto average_temp = average.new_value(temp_sens());
            auto pwm_duty = calc_pwm_duty_from_temperature(average_temp);
            pwm_pin_obj.set(pwm_duty);
            if (errno)
                throw std::runtime_error(fmt::format("errno {}, {}\n", errno, std::strerror(errno)));
            fmt::print("Temp: {:.2f} | Set PWM: {:2d}", average_temp, pwm_duty);
            if(tacho)
            {
                auto actual_pwm = tacho->read();
                fmt::print(" | Act PWM {:4d}", actual_pwm);
            }
            fmt::print("\n");
            std::unique_lock<std::mutex> lk(m);
            cond.wait_for(lk, check_timeout, [] { return !running; });
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}

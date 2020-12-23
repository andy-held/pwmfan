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

#include <docopt.h>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <wiringPi.h>
#include <softPwm.h>

static const char USAGE[] =
R"(pwmfan

    Usage:
      pwmfan [--pwm-pin=<pin>] [--min-speed=<%>] [--low-temp=<°C>] [--high-temp=<°C>] [--tacho-pin=<pin>]
      pwmfan (-h | --help)

    Options:
      -h --help         Show this screen.
      --pwm-pin=<pin>   BCM pin to use for PWM [default: 18].
      --min-speed=<%>   Minimal fan speed [default: 50].
      --low-temp=<°C>   If CPU temperatur is higher than this fan speed will increase [default: 30].
      --high-temp=<°C>  If CPU temperature is at least this hight the fan will run at full speed [default: 50].
      --tacho-pin=<pin> BCM pin to use for fan tachometer input. If it is not set, the tachometer will not be used.
)";

using namespace std::chrono_literals;
constexpr const auto check_timeout = 2s;
constexpr const unsigned int temp_average_interval = 5;
constexpr const int pwm_range = 100;

constexpr bool is_hw_pwm_pin(int pin)
{
    return pin == 12 or pin == 13 or pin == 18 or pin == 19;
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

struct Pin
{
    Pin(int id_in): id(id_in)
    {}

    virtual ~Pin()
    {
        pinMode(id, INPUT);
        pullUpDnControl(id, PUD_OFF);
    }

    int id;
};

struct Output_Pin: public Pin
{
    Output_Pin(int id_in): Pin(id_in) {}
    ~Output_Pin() override = default;

    virtual void set(int value) = 0;
};

struct HW_PWM_Pin: public Output_Pin
{
    HW_PWM_Pin(int id_in): Output_Pin(id_in)
    {
        pinMode(id, PWM_OUTPUT);
        pwmSetClock(768);// Set PWM divider of base clock 19.2Mhz to 25Khz (Intel's recommendation for PWM FANs)
        pwmSetRange(pwm_range);
        set(pwm_range);// Setting to the max PWM
    }

    ~HW_PWM_Pin() override = default;

    void set(int value) override
    {
        pwmWrite(id, value);
    }
};

struct SW_PWM_Pin: public Output_Pin
{
    SW_PWM_Pin(int id_in): Output_Pin(id_in)
    {
        if (softPwmCreate(id, pwm_range, pwm_range))
            throw std::runtime_error(fmt::format("could not create soft pwm on pin {}", std::to_string(id)));
    }

    ~SW_PWM_Pin() override
    {
        softPwmStop(id);
    }

    void set(int value) override
    {
        softPwmWrite(id, value);
    }
};

std::unique_ptr<Output_Pin> create_pwm_pin(int id)
{
    if(id == 12 or id == 13 or id == 18 or id == 19)
        return std::make_unique<HW_PWM_Pin>(id);
    return std::make_unique<SW_PWM_Pin>(id);
}

std::atomic<int> rpm_pulses = 0;
void inc_pulses()
{
    rpm_pulses++;
}
struct Tacho_Pin: public Pin
{
    explicit Tacho_Pin(int id_in): Pin(id_in)
    {
        pinMode(id, INPUT);
        pullUpDnControl(id, PUD_UP);
        wiringPiISR(id, INT_EDGE_FALLING, inc_pulses);
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
int main(int argc, char** argv)
{
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, { argv + 1, argv + argc });
    for(auto const& arg : args) {
        fmt::print("{} {}\n", arg.first, arg.second);
    }
    struct sigaction sigHandler;
    memset(&sigHandler, 0, sizeof(sigHandler));
    sigHandler.sa_handler = [](int /*signal*/)
    {
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

    auto calc_pwm_duty_from_temperature = [&]()
    {
        const int min_speed = args["--min-speed"].asLong();
        const int low_temp = args["--low-temp"].asLong();
        const float low_temp_f = static_cast<float>(low_temp);
        const int high_temp = args["--high-temp"].asLong();
        const int linear_range = pwm_range - min_speed;
        const float temp_diff_range_factor = static_cast<float>(linear_range / (high_temp - low_temp));

        return [=](float temp)
        {
            float diff = temp - low_temp_f;
            int linear_part = static_cast<int>(diff * temp_diff_range_factor);
            return std::min(std::max(linear_part, 0) + min_speed, pwm_range);
        };
    }();

    if (wiringPiSetupGpio())
        throw std::runtime_error("setup failed");
    try
    {
        CPU_Temperature_Sensor temp_sens;
        std::unique_ptr<Output_Pin> pwm_pin = create_pwm_pin(args["--pwm-pin"].asLong());
        std::unique_ptr<Tacho_Pin> tacho;
        if(args["--tacho-pin"])
            tacho = std::make_unique<Tacho_Pin>(args["--tacho-pin"].asLong());
        Temperature_Average<temp_average_interval> average(temp_sens());
        while (running)
        {
            auto average_temp = average.new_value(temp_sens());
            auto pwm_duty = calc_pwm_duty_from_temperature(average_temp);
            pwm_pin->set(pwm_duty);
            if (errno)
                throw std::runtime_error(std::strerror(errno));
            fmt::print("Temp: {:.2f} | Set PWM: {:2d}", average_temp, pwm_duty);
            if(tacho)
            {
                auto actual_pwm = tacho->read();
                fmt::print(" | Act PWM {:4d}", actual_pwm);
            }
            fmt::print("\n");
            std::unique_lock<std::mutex> lk(m);
            cond.wait_for(lk, check_timeout, []{return !running;});
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}

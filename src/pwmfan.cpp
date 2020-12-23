#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <wiringPi.h>
#include <softPwm.h>

constexpr const int pwm_pin_number = 5;
constexpr const int pwm_range = 100;
constexpr const int tacho_pin = 16;
constexpr const int min_speed = 40;
constexpr const int low_temp = 30; // Lowest temperature, if lowest of this, the FAN is on min_speed
constexpr const int high_temp = 50; // Higher than it, the FAN is on full speed

using namespace std::chrono_literals;
constexpr const auto check_sec = 2s;

constexpr bool is_hw_pwm_pin(int pin)
{
    return pin == 12 or pin == 13 or pin == 18 or pin == 19;
}

template<int pin>
struct PWM_Pin
{
    PWM_Pin()
    {
        if constexpr (is_hw_pwm_pin(pin))
        {
            std::cout << "creating hardware pwm on pin " << pin << "\n";
            wiringPiSetupGpio();
            pinMode(pin, PWM_OUTPUT);
            pwmSetClock(768); // Set PWM divider of base clock 19.2Mhz to 25Khz (Intel's recommendation for PWM FANs)
            pwmSetRange(pwm_range);
            set(pwm_range); // Setting to the max PWM
        }
        else
        {
            std::cout << "creating software pwm on pin " << pin << "\n";
            wiringPiSetupGpio();
            if (softPwmCreate(pin, pwm_range, pwm_range))
                throw std::runtime_error("could not create soft pwm on pin" + std::to_string(pin));
        }
    }

    ~PWM_Pin()
    {
        if constexpr (is_hw_pwm_pin(pin))
        {
            pinMode(pin, INPUT);
        }
        else
        {
            softPwmStop(pin);
        }
    }

    void set(int value)
    {
        if constexpr (is_hw_pwm_pin(pin))
        {
            pwmWrite(pin, value);
        }
        else
        {
            softPwmWrite(pin, value);
        }
    }
};

int rpm_pulses = 0;
struct Tacho_Pin
{
    explicit Tacho_Pin(int pin_id):
        id(pin_id)
    {
        wiringPiSetupGpio();
        pinMode(id, INPUT);
        pullUpDnControl(tacho_pin, PUD_UP);
        wiringPiISR(id, INT_EDGE_FALLING, []()
        {
            rpm_pulses++;
        });
        last_measurement = std::chrono::steady_clock::now();
    }

    ~Tacho_Pin()
    {
        pinMode(id, INPUT);
    }

    int operator() ()
    {
        constexpr const double revolutions_per_pulse = 0.5;
        int pulses = 0;
        std::swap(pulses, rpm_pulses);
        auto this_measurement = std::chrono::steady_clock::now();
        std::chrono::nanoseconds duration = this_measurement - last_measurement;
        last_measurement = this_measurement;
        auto pulses_per_second = pulses/static_cast<double>(duration.count());
        return static_cast<int>(pulses_per_second*60*revolutions_per_pulse);
    }

    std::chrono::time_point<std::chrono::steady_clock> last_measurement;
    int id;
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

int calc_pwm_duty_from_temperature(float temp)
{
    constexpr const int linear_range = pwm_range - min_speed;
    constexpr const int temp_diff_range_factor = linear_range / (high_temp - low_temp);

    float diff = temp - low_temp;
    int linear_part = static_cast<int>(diff * temp_diff_range_factor);
    return std::min(std::max(linear_part, 0) + min_speed, pwm_range);
}

int main()
{
    CPU_Temperature_Sensor temp_sens;
    PWM_Pin<pwm_pin_number> pwm_pin;
    Tacho_Pin tacho(16);
    while (true)
    {
        auto temp = temp_sens();
        auto pwm_duty = calc_pwm_duty_from_temperature(temp);
        pwm_pin.set(pwm_duty);
        auto actual_pwm = tacho();
        std::cout << "Temp: " << temp << " Set PWM: " << pwm_duty << " Act PWM: " << actual_pwm << "\n";
        if(errno)
            std::cout << std::strerror(errno) << "\n";
        std::this_thread::sleep_for(check_sec);
    }
    return 0;
}

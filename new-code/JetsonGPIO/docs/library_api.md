
# Complete library API

*JetsonGPIO* (C++) provides almost same APIs as those of *Jetson.GPIO* (Python).
The following discusses the use of each API:


#### 1. Include the library

To include *JetsonGPIO* use:
```cpp
#include <JetsonGPIO.h>
```

All public APIs are declared in namespace `GPIO`. If you want to make your code shorter, you can use:
```cpp
using namespace GPIO; // optional
```

#### 2. Pin numbering

*JetsonGPIO* provides four ways of numbering the I/O pins. The first
two correspond to the modes provided by the *RPi.GPIO*, which is a Python GPIO library for Raspberry Pi, i.e BOARD and BCM
which refer to the pin number of the 40 pin GPIO header and the Broadcom SoC
GPIO numbers respectively. The remaining two modes, CVM and TEGRA_SOC use
strings instead of numbers which correspond to signal names on the CVM/CVB
connector and the Tegra SoC respectively.

To specify which mode you are using (mandatory), use the following function
call:
```cpp
GPIO::setmode(GPIO::BOARD);
// or
GPIO::setmode(GPIO::BCM);
// or
GPIO::setmode(GPIO::CVM);
// or
GPIO::setmode(GPIO::TEGRA_SOC);
```

To check which mode has been set, you can call:
```cpp
GPIO::NumberingModes mode = GPIO::getmode();
```
This function returns an instance of enum class `GPIO::NumberingModes`. The mode must be one of `GPIO::BOARD`, `GPIO::BCM`, `GPIO::CVM`, `GPIO::TEGRA_SOC` or `GPIO::NumberingModes::None`.

#### 3. Warnings

It is possible that the GPIO you are trying to use is already being used
external to the current application. In such a condition, *JetsonGPIO* will warn you if the GPIO being used is configured to anything but the
default direction (input). It will also warn you if you try cleaning up before
setting up the mode and channels. To disable warnings, call:
```cpp
GPIO::setwarnings(false);
```

#### 4. Set up a channel

The GPIO channel must be set up before use as input or output. To configure
the channel as input, call:
```cpp
// (where channel is based on the pin numbering mode discussed above)
GPIO::setup(channel, GPIO::IN); // channel must be int or std::string
```

To set up a channel as output, call:
```cpp
GPIO::setup(channel, GPIO::OUT);
```

It is also possible to specify an initial value for the output channel:
```cpp
GPIO::setup(channel, GPIO::OUT, GPIO::HIGH);
```

Setting up multiple channels is also supported. 
```cpp
// setup multiple channels as input
GPIO::setup({chan1, chan2}, GPIO::IN);

// setup multiple output channels. The initial value(GPIO::HIGH) is optional. 
std::vector<int> channels = {chan3, chan4, chan5};  // or std::vector<std::string>
GPIO::setup(channels, GPIO::OUT, GPIO::HIGH);

// setup multiple output channels with multiple initial values. The number of channels and number of values must be equal. 
GPIO::setup({chan6, chan7}, GPIO::OUT, {GPIO::HIGH, GPIO::LOW});
```


#### 5. Input

To read the value of a channel, use:

```cpp
int value = GPIO::input(channel);
```

This will return either `GPIO::LOW`(== 0) or `GPIO::HIGH`(== 1).

#### 6. Output

To set the value of a pin configured as output, use:

```cpp
GPIO::output(channel, state);
```

where state can be `GPIO::LOW`(== 0) or `GPIO::HIGH`(== 1).  
  
You can also output to multiple channels:
```cpp
std::vector<int> channels = { 18, 12, 13 };   // or std::vector<std::string>
GPIO::output(channels, GPIO::HIGH); // or GPIO::LOW
// set the first channel to LOW and rest to HIGH
GPIO::output(channels, {GPIO::LOW, GPIO::HIGH, GPIO::HIGH});
```

#### 7. Clean up

At the end of the program, it is good to clean up the channels so that all pins
are set in their default state. To clean up all channels used, call:

```cpp
GPIO::cleanup();
```

If you don't want to clean all channels, it is also possible to clean up
individual channels:

```cpp
GPIO::cleanup(chan1); // cleanup only chan1
GPIO::cleanup({chan1, chan2}); // cleanup only chan1 and chan2
```

#### 8. Jetson Board Information and library version

To get information about the Jetson module, use/read:

```cpp
std::string info = GPIO::JETSON_INFO;
// or
std::string info = GPIO::JETSON_INFO();
```

To get the model name of your Jetson device, use/read:

```cpp
std::string model = GPIO::model;
// or
std::string model = GPIO::model();
```

To get information about the library version, use/read:

```cpp
std::string version = GPIO::VERSION;
```

This provides a string with the X.Y.Z version format.

#### 9. Interrupts

Aside from busy-polling, *JetsonGPIO* provides three additional ways of monitoring an input event:

__The wait_for_edge() function__

This function blocks the calling thread until the provided edge(s) is detected. The function can be called as follows:

```cpp
GPIO::wait_for_edge(channel, GPIO::RISING);
```
The second parameter specifies the edge to be detected and can be `GPIO::RISING`, `GPIO::FALLING` or `GPIO::BOTH`. If you only want to limit the wait to a specified amount of time, a timeout can be optionally set:

```cpp
// timeout is in milliseconds__
// debounce_time set to 10ms
GPIO::WaitResult result = GPIO::wait_for_edge(channel, GPIO::RISING, 10, 500);
```
The function returns a `GPIO::WaitResult` object that contains the channel name for which the edge was detected. 

To check if the event was detected or a timeout occurred, you can use `.is_event_detected()` method of the returned object or just simply cast it to `bool` type.
The returned object is implicitly convertible to `bool` and its value is equal to the return value of `.is_event_detected()`:
```cpp
// returns the channel name for which the edge was detected ("None" if a timeout occurred)
std::string eventDetectedChannel = result.channel();

if(result.is_event_detected()){ /*...*/ }
// or 
if(result){ /*...*/ } // is equal to if(result.is_event_detected())
```

__The event_detected() function__

This function can be used to periodically check if an event occurred since the last call. The function can be set up and called as follows:

```cpp
// set rising edge detection on the channel
GPIO::add_event_detect(channel, GPIO::RISING);
run_other_code();
if(GPIO::event_detected(channel))
    do_something();
```

As before, you can detect events for `GPIO::RISING`, `GPIO::FALLING` or `GPIO::BOTH`.

__A callback function run when an edge is detected__

This feature can be used to run a second thread for callback functions. Hence, the callback function can be run concurrent to your main program in response to an edge. This feature can be used as follows:

```cpp
// define callback function
void callback_fn(const std::string& channel) 
{
    std::cout << "Callback called from channel " << channel << std::endl;
}

// add rising edge detection
GPIO::add_event_detect(channel, GPIO::RISING, callback_fn);
```

Any object that satisfies the following requirements can be used as a callback function. 

- Callable with a `const std::string&` type argument (for the channel name) **OR** without any argument. The return type must be `void`.
> [!NOTE]
> If the callback object is *not only* callable with a `const std::string&` type argument *but also* callable without any argument, the method with a `const std::string&` type argument will be used as a callback function. 
- Copy-constructible 
- Equality-comparable with same type (ex> `func0 == func1`)  

Here is a user-defined type callback example:

```cpp
// define callback object
class MyCallback
{
public:
    MyCallback(const std::string& name) : name(name) {}
    MyCallback(const MyCallback&) = default; // Copy-constructible

    void operator()(const std::string& channel) // Callable with one string type argument
    {
        std::cout << "A callback named " << name;
        std::cout << " called from channel " << channel << std::endl;
    }

    bool operator==(const MyCallback& other) const // Equality-comparable
    {
        return name == other.name;
    }

    bool operator!=(const MyCallback& other) const 
    {
        return !(*this == other);
    }
    
private:
    std::string name;
};

// create callback object
MyCallback my_callback("foo");
// add rising edge detection
GPIO::add_event_detect(channel, GPIO::RISING, my_callback);
```


More than one callback can also be added if required as follows:

```cpp
// you can also use callbacks witout any argument
void callback_one() 
{
    std::cout << "First Callback" << std::endl;
}

void callback_two() 
{
    std::cout << "Second Callback" << std::endl;
}

GPIO::add_event_detect(channel, GPIO::RISING);
GPIO::add_event_callback(channel, callback_one);
GPIO::add_event_callback(channel, callback_two);
```

> [!NOTE]
> The two callbacks in this case are run sequentially, not concurrently since there is only one event thread running all callback functions.

In order to prevent multiple calls to the callback functions by collapsing multiple events in to a single one, a debounce time can be optionally set:

```cpp
// bouncetime set in milliseconds
GPIO::add_event_detect(channel, GPIO::RISING, callback_fn, 200);
```

If one of the callbacks are no longer required it may then be removed:

```cpp
GPIO::remove_event_callback(channel, callback_two);
```

Similarly, if the edge detection is no longer required it can be removed as follows:

```cpp
GPIO::remove_event_detect(channel);
```

#### 10. Check function of GPIO channels  

This feature allows you to check the function of the provided GPIO channel:

```cpp
GPIO::Directions direction = GPIO::gpio_function(channel);
```

The function returns either `GPIO::IN` or `GPIO::OUT` which are the instances of enum class `GPIO::Directions`.

#### 11. PWM  
> [!NOTE]
> *JetsonGPIO* supports PWM *only* on pins with attached hardware PWM controllers. Unlike *RPi.GPIO*, *JetsonGPIO* does *not* implement software emulated PWM. 

> [!IMPORTANT]
> The system pinmux must be configured to connect the hardware PWM controlller(s) to the relevant pins. If the pinmux is not configured, PWM signals will not reach the pins! *JetsonGPIO* does *not* dynamically modify the pinmux configuration to achieve this. Read the [L4T documentation](https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html) for details on how to configure the pinmux.


See `samples/simple_pwm.cpp` for details on how to use PWM channels.

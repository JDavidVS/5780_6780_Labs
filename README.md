![Main](/assets/images/embedded.jpg)

# ECE 6780 Embedded System Design - Labs
University of Utah

David Venegas

# Lab 1: [General-purpose I/Os (GPIO)](6780_lab1_GPIO)
These exercises explore two basic operations of the GPIO: Blinking LEDs, and reading the state of a
pushbutton.

### 1.5.1.a Configuring a GPIO Pin to Output and Blink an LED
Recreate the blinking demo using the green and orange LEDs on the Discovery board:

- [Blinking with ORANGE and GREEN LED's main.c](<6780_lab1_GPIO/Main files/Main ORANGE and GREEN/main.c>)

### 1.5.1.b Changing to the New LEDs
Recreate the blinking demo using the red and blue LEDs on the Discovery board:

- [Blinking with RED and BLUE LED's main.c](<6780_lab1_GPIO/Main files/Main RED and BLUE/main.c>)

### 1.5.2 Configuring a GPIO Pin to Input and Reading a Button
Change your version of the flashing LED program so that each button press toggles the LEDs instead
of the delay:

- [Input and reading a button main.c](<6780_lab1_GPIO/Main files/Main Button/main.c>)

# Lab 2: [Interrupts](6780_lab2_Interrupts)
This lab introduces the concept of interrupt-driven programming and guides through the configuration of interrupt-oriented peripherals; the exercises herein provide a foundation for utilizing interrupts in an embedded application. They introduce the practice of enabling, configuring parameters and writing handler routines to service peripheral interrupt requests.

### 2.6.3 Setting up Interrupt Handler and 2.6.5 and Interrupt Nesting
Although in some cases it may be infeasible, normally you want to keep interrupt handlers as short as possible to avoid starving parts of your program. This exercise demonstrates how a long running interrupt impacts the main application loop.

- [main.c](<6780_lab2_Interrupts/Core/Src/main.c>)

- [stm32f0xx_it.c](<6780_lab2_Interrupts/Core/Src/stm32f0xx_it.c>)

![lab2](6780_lab2_Interrupts/Sources/Delay3.png)

**From the oscilloscope/logic analyzer: ~1.8 sec of delay**

**$D_{15}$** - Button

**$D_{14}$** - Blue LED

**$D_{13}$** - Red LED                        

**$D_{12}$** - Green LED

**$D_{11}$** - Orange LED

# Lab 3: [Timers](6780_lab3_Timers)
This lab explores when deciding on a timer to use for an application, it is helpful to understand their capabilities and limits to determine their suitability for the task. It also explores PWM and GPIO Alternate Functions.

### 3.1 Using Timer Interrupts
Set up a timer such that the update event (UEV) triggers an interrupt at 4 Hz. Timer peripherals allow for greater flexibility in choosing an interrupt period over manually counting in the SysTick handler.

- [main.c](<6780_lab3_Timers/Sources/First_Experiment/main.c>)

- [stm32f0xx_it.c](<6780_lab3_Timers/Sources/First_Experiment/stm32f0xx_it.c>)

![lab31](6780_lab3_Timers/Sources/First_Experiment/scope_0.png)

**From the oscilloscope/logic analyzer: f = 4Hz or T = 250ms**

**$D_{15}$** - Green LED

**$D_{14}$** - Orange LED


### 3.2 Configuring Timer Channels to PWM Mode, 3.3 Configuring Pin Alternate Functions, and 3.4 Measuring PWM Output
Set up a timer such that the update event (UEV) triggers an interrupt at 4 Hz. Timer peripherals allow for greater flexibility in choosing an interrupt period over manually counting in the SysTick handler.

- [main.c](<6780_lab3_Timers/Sources/CCRx/main.c>)

- [stm32f0xx_it.c](<6780_lab3_Timers/Sources/CCRx/stm32f0xx_it.c>)

![lab32](6780_lab3_Timers/Sources/CCRx/20percent.png)

**From the oscilloscope/logic analyzer: For CCRx at 20% of ARR f = 800Hz or T = 1.25ms**

**$D_{15}$** - Blue LED

**$D_{14}$** - Red LED


![lab33](6780_lab3_Timers/Sources/CCRx/1percent.png)

**From the oscilloscope/logic analyzer: For CCRx at 1% of ARR f = 800Hz or T = 1.25ms**

**$D_{15}$** - Blue LED -----> Dimmer light

**$D_{14}$** - Red LED  -----> Brighter light


![lab34](6780_lab3_Timers/Sources/CCRx/125.png)

**From the oscilloscope/logic analyzer: For CCRx at 100% of ARR f = 800Hz or T = 1.25ms**

**$D_{15}$** - Blue LED -----> Brighter light

**$D_{14}$** - Red LED  -----> Dimmer light




![Main](/assets/images/embedded.jpg)

# ECE 6780 Embedded System Design - Labs
University of Utah

David Venegas

## Lab 1: [General-purpose I/Os (GPIO)](6780_lab1_GPIO)
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

## Lab 2: [Interrupts](6780_lab2_Interrupts)
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



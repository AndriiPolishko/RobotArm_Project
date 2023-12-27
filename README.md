

# Sem work <mark>Robo Hand</mark>: <mark>stm32f411</mark>
Authors (team): <mark>Kateryna Kovalchuk, Sydor Nataliia, Polishko Andrii, Bronetskyu Volodymyr</mark><br>
## Prerequisites

<mark>STM Kube ide, Prosthesis, RoboGlove</mark>

### Usage

<mark>
  This project allows the prosthesis to be controlled by potentiometers and respond to the movement of the RoboGlove. 
</mark>

### Results
<mark>
  How does it work?  We send a signal to the potentiometer, this signal is collected by the DMA, then the ADC converts the signal into a digital signal and the PWM sends this signal to the Timers, which in turn send it to the Servo. 
</mark>


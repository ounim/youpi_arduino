# youpi_arduino
This is an arduino code base to run the youpi arm


#
a command is received to make the motor turn.
the timer is scheduled to interrupt when the next step has to run.
the maximum acceptable speed of the motor is 44 pulse per second.
A pulse is 2 commands so 88 commands per seconds.
There are 6 motors to pilot so 528 commands per seconds so 9600 baud is well above the maximum speed possible.

Arduino run at 16MHz. so it means it count up to (16 10e6)/(9.6 10e3) = 1.6 10e3 before sending each command.

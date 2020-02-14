# #lighthack

These are projects based original on the #lighthack project from ETC https://github.com/ETCLabs/lighthack<br>
You find further informations also in the ETC forum https://community.etcconnect.com/etclabs/f/lighthack<br>
All projects uses an Arduino **Arduino UNO**, but also an **Arduino MEGA** gives you connection to your console. On nomad you can use other boards, but you have to figure out which works.

## box1
Is a cleaned version of the original box1

## box2A
Is a modifcation of the original box1 and uses the former Next and Last buttons to step thru a parameter list.<br>
Because of the limited RAM of the Arduino UNO, the parameter list
is limited and depends on the size of the parameter names.<br>
On an Arduino UNO the is maximum of 14 parameter names.<br>
On an Arduino MEGA more than 14 parameters can be used, also
on other boards like Teensy 3.x or 4<br>
For Teensy you need to fake the PID/VID of the USB connection to
work with a console, have a look to the forum.<br>
Added a keyword „none“ for gaps in the parameter list, former titeled as „empty“.

## box2B
Extends the box2A with extra buttons for Next / Last keys<br>
It uses the buttons of the encoder for posting the Home position

## faderwing
Is a small faderwing with 6 faders and 12 buttons for Stop / Go

## macrobox
Is a box with 12 buttons to fire Macros, actual 101 thru 112 but this can easily changed.

# What’s next
There are some things to do because of heap fragmentation using many parameters on microcontrollers with less RAM. This is caused of dynamic memory allocation with malloc/new and free/delete in the Arduino String library and the OSC library from CNMAT.
- eleminating Arduino String library calls using the classic C string.h 
- replace the OSC library with a simplified one<br>

And for future a dedicated library for EOS.





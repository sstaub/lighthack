# #lighthack

These are projects based original on the #lighthack project from ETC https://github.com/ETCLabs/lighthack<br>
You find further informations also in the ETC forum https://community.etcconnect.com/etclabs/f/lighthack<br>
All projects uses an Arduino **Arduino UNO**, but also an **Arduino MEGA** gives you connection to your console. On nomad you can use other boards, but you have to figure out which works.

## box2A
is a modifcation of the original box1 and uses the Next and Last buttons to step thru a parameter list.<br>
Because of the limited RAM of the Arduino UNO, the parameter list
is limited and depends on the size of the parameter names.<br>
On an Arduino UNO the is maximum of 16 parameter names.<br>
On an Arduino MEGA more than 16 parameters can be used, also
on other boards like Teensy 3.x or 4<br>
For Teensy you need to fake the PID/VID of the USB connection to
work with a console, have a look to the forum.

## box2B
extends the box2A with extra buttons for Parameter Next / Last<br>
It uses the buttons of the encoder for posting the Home position

## faderwing
is a small faderwing with 6 faders and 12 buttons for Stop / Go

## macrobox
is a box with 12 buttons to fire Macros, actual 101 thru 112 but this can easily changed.





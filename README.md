ArduFloppy
==========
Interfacing with a floppy drive on Arduino

This is an attempt to read/write data on a floppy disk using Arduino. It's impossible (or nearly impossible) to read standard PC formatted floppies using Arduino because it's way too slow, but you can read your own data written at a slower rate. The code uses a rather crude method of encoding the data but it works (almost) perfectly!

Made on Arduino MEGA 2560 with some random floppy drive from and old computer.

Some useful resources
---------------------
https://arduino.stackexchange.com/questions/3702/controlling-floppy-disk-drive-with-arduino
http://www.hermannseib.com/documents/floppy.pdf

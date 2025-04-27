This repo contains:
- `/non-steering-test-code` -- folder with code to test packages (not functional stuff)
- `electrical_motor_code.py` -- code that controls motors. Contains all of the functions that use encoder values (not properly tested as of April 2025), and time-based driving functions.
- `remote_control.py` -- code with motor control, specifically for use with remote controlling the robot using a computer keyboard. 

### Running the code
From any machine, ssh into the raspberry pi (must be powered on). Ask someone on software for the IP address and password.

Then, navigate to the Desktop folder using `cd Desktop`.
From there:
1. Run `git pull`. If there's any local changes that aren't committed you can probably stash them.
2. Currently, our code isn't made for just running a script, so to run the code, open a python terminal by typing `python3`. Your command line should now start with `>>>`.
3. Figure out which code file you are trying to run, and type `from [name of file, without the .py] import *`. Now, you can call individual functions to do things.

If you're using `electrical_motor_code.py`:
There are several functions that do individual things, look at function specifications for what you need.

If you're running `remote_control.py`:
After running `from remote_control import *`, call the function `keyboardControl()` (do not exit the python shell). 
- To move forward/backward, press the up/down key once. To stop, press the same key again.
- To turn, press the left/right key once to initiate turning. Press the same key again to stop turning. You'll need to "turn" the wheels the other way to straighten the robot again.
- Press "q" or "esc" to quit the program.
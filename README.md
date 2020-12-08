# EQUALIZER: A Gyro-controlled ASCII Game
Welcome to Group 2's project for ECSE 444! This file will outline everything you need to know about running EQUALIZER, our gyro-controlled ASCII game.

## Requirements
Since our system is using ANSI escape characters, the game may not work with all serial monitors. For us, the one that worked the best was [PuTTY](https://www.putty.org/), a free application that you can download on Windows 10. Unfortunately, we have not found a serial monitor that works with our game for MacOS as of yet, but feel free to try it with your monitor of choice!

### Setting up PuTTY
Once you have downloaded PuTTY, run the application. The `Session` category should be selected. On this window, for the `Connection type`, select "Serial". Then, type in the appropriate `Serial line` (e.g. `COM4`), the set the `Speed` to 115200. Once you have done that, give a name to your configuration by typing a name in the `Saved Sessions` input box, then click `Save`.

Once you have done that, using the menu on the left-hand side, select `Terminal`. This should change the view on the right-hand side. Here, make sure that `Implicit CR in every LF` and `Use background colour to erase screen` are both checked.

Finally, to get the best possible EQUALIZER experience, we're going to want to change the colour of the cursor to black. To do so, use the left-hand menu to select `Window > Colours`. In the window titled `Select a colour to adjust`, select `Cursor Colour`. Then, set the RGB values to be all zeroes.

In order to save this configuration, click on the `Session` entry on the left-hand side, select the name of the session you had saved previously and click `Save`. You are now ready to play EQUALIZER!

## Development Board Orientation
EQUALIZER uses the orientation of the board to control your character on-screen. In order for the orientation portion to work properly, ensure you're rotating the board as indicated in this diagram:
```
                               |
  -----------------------------|-----------------------------
 |                             |                             |
 |  /\                         |                             |  <---- (simplified) development board
 |  \/                         |                             |
 |     <-- pushbuttons         |                             |
 |  /\                         |                             |
 |  \/                         |                             |
 |                             |                             |
 |                             |                             |
 |                             |                             |
 |                             |                             |
  -----------------------------|-----------------------------
                               | <---- rotate around this axis
```

## Playing the game
When you start a new game of EQUALIZER, please lay the board flat on a table until the game begins. This is because the board runs through a calibration sequence before beginning the game.

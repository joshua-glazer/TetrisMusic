# TetrisMusic

The music code for a Tetris game programmed for an STM32F401RE. The full project is elsewhere...

---------------
How it works:
---------------

I vary the frequency of my notes by controlling a PWM signal: I modify the prescaler of timer 4 of the STM32. Knowing the clock speed, I picked a value for the Auto Reload Register so that I'd have a good range of values for the prescaler.

In order to make the code more elegant and easier, I just set the prescaler to 0 when I don't want the signal to be audible. Rather than just turning the signal off, I make the signal's frequency so high that we can't hear it.

I created a bunch of macros to control the duration of notes, and I have some slightly more annoying code for dotted notes.

---------------
Current status:
---------------

Well, it works. The sound grates the ears, though, if you listen to it for too long. Ideally I'll be able to reproduce a sine wave rather than just outputting a square wave. I'll have to rewrite my code in order to do that, but it should allow me to do more interesting stuff, like adding sound envelopes and possibly synthesizing chords.
# test if fluidsynth is working, plays a chord for 1 second
import time
import fluidsynth

fs = fluidsynth.Synth()
fs.start(driver='coreaudio') # might need to change for Windows

sfid = fs.sfload("SalC5Light2.sf2")
fs.program_select(0, sfid, 0, 0)

fs.noteon(0, 60, 90)
fs.noteon(0, 67, 90)
fs.noteon(0, 76, 90)

time.sleep(1.0)

fs.noteoff(0, 60)
fs.noteoff(0, 67)
fs.noteoff(0, 76)

time.sleep(1.0)

fs.delete()
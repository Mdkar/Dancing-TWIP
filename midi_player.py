import time
import serial
import argparse
import mido
import fluidsynth

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-s', '--song', action='store', dest='song',help='song file') # MIDI file
	parser.add_argument('-d', '--debug', action='store_true', help='print midi messages') # Prints MIDI messages to standard output
	parser.add_argument('-p', '--pace', action='store',dest='pace', help='set song pace multiplier, 1 is normal speed, 2 is double etc.', default=1) # Set song pace
	args = parser.parse_args()

	arduino = serial.Serial(port='/dev/cu.usbmodem14101', baudrate=1000000, timeout=.1)
	# Change port to whatever your serial port's name is, e.g. COM3, COM4, bluetooth...

	# Standard piano notes correspond to MIDI notes 21-108

	print ('Press Ctrl-C to quit.')
	try:
		fs = fluidsynth.Synth()
		fs.start(driver='coreaudio')
		sfid = fs.sfload("SalC5Light2.sf2")
		fs.program_select(0, sfid, 0, 0)
		midi = mido.MidiFile(args.song)
		if args.debug:
			print(midi.tracks)

		for msg in midi:
			try:
				time.sleep(msg.time*(1/float(args.pace)))
				if args.debug:
					print(msg)
				if msg.type == 'note_on':
					if msg.velocity >0: # Sometimes a 'note_on' message with velocity 0 is used instead of a 'note_off' message.
						fs.noteon(0, msg.note, 100)
						if (msg.note >= 21 and msg.note <= 108 and msg.time != 0): # possibly condition on msg.channel
							if (msg.time < 1): # make big to tilt more often
								arduino.write(bytes('n', 'utf-8'))
							elif (msg.time < 0): # make big to spin more often
								arduino.write(bytes('t', 'utf-8'))
							else :
								arduino.write(bytes('s', 'utf-8')) # balance normally
							print(msg.time)

					else:
						fs.noteoff(0, msg.note)

				if msg.type  == 'note_off':
					fs.noteoff(0, msg.note)
			except Exception as e:
				print(e)
		arduino.write(bytes('s', 'utf-8'))
	except KeyboardInterrupt:
		arduino.write(bytes('s', 'utf-8'))
		exit()
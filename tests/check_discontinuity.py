#!/usr/bin/python3

"""Play a sine signal."""
import argparse
import sys
import traceback
import queue

import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt


def int_or_str(text):
	"""Helper function for argument parsing."""
	try:
		return int(text)
	except ValueError:
		return text


parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
	'-l', '--list-devices', action='store_true',
	help='show list of audio devices and exit')
args, remaining = parser.parse_known_args()
if args.list_devices:
	print(sd.query_devices())
	parser.exit(0)
parser = argparse.ArgumentParser(
	description=__doc__,
	formatter_class=argparse.RawDescriptionHelpFormatter,
	parents=[parser])
parser.add_argument(
	'frequency', nargs='?', metavar='FREQUENCY', type=float, default=555,
	help='frequency in Hz (default: %(default)s)')
parser.add_argument(
	'-d', '--device', type=int_or_str,
	help='output device (numeric ID or substring)')
parser.add_argument(
	'-a', '--amplitude', type=float, default=0.2,
	help='amplitude (default: %(default)s)')
args = parser.parse_args(remaining)

start_idx = 0
audioQueue = queue.Queue()

try:
	samplerate = sd.query_devices(args.device, 'output')['default_samplerate']

	def callback_generate_sine(outdata, frames, time, status):
		if status:
			print(status, file=sys.stderr)
		global start_idx
		t = (start_idx + np.arange(frames)) / samplerate
		t = t.reshape(-1, 1)
		outdata[:] = args.amplitude * np.sin(2 * np.pi * args.frequency * t)
		start_idx += frames
		
	def callback_check_sine(data: np.ndarray, frames: int, time, status):
		if frames == 0:
			return
		audioQueue.put(data.copy())

	with sd.OutputStream(device=args.device, channels=1, callback=callback_generate_sine,
						 samplerate=samplerate, blocksize=4096):
		with sd.InputStream(device=args.device, channels=1, callback=callback_check_sine,
							samplerate=samplerate, blocksize=4096):
			while True:
				data: np.ndarray = audioQueue.get()

				input_data = data[:, 0]
				y = np.diff(input_data)
				try:
					y = np.absolute(y)
					average = np.average(y)
					# Add 1.5 ratio for margin
					expected_max = average / ( 2 / np.pi) * 1.5

					if(np.max(y) > expected_max):
						print("glitch")
						plt.clf()
						x = np.linspace(0, len(y)-1, len(y))
						plt.plot(x, input_data[:-1])
						plt.plot(x, y)
						plt.axhline(y=expected_max, color='r', linestyle='-')
						plt.show()
						with audioQueue.mutex:
							audioQueue.queue.clear()
				except Exception as e:
					print(traceback.format_exc())
					print(y)
					print(data[0])
except KeyboardInterrupt:
	parser.exit('')
except Exception as e:
	parser.exit(type(e).__name__ + ': ' + str(e))
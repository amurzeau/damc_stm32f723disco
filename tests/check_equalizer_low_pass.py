#!/usr/bin/python3

"""Play a sine signal."""
import argparse
import sys
import traceback
import queue
import time
import math

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

def multitone_generator(blocksize, frequency_number):
	frequencies = 2 * np.pi * np.logspace(math.log10(20.0), math.log10(20000.0), num=frequency_number)

	start_idx = 0
	def multitone(outdata, frames):
		outdata[:] = 0
		nonlocal start_idx
		indexes = np.linspace(np.ones(frequency_number) * (start_idx/48000,), ((start_idx+frames)/48000,), num=frames, endpoint=False)

		data = args.amplitude / frequency_number * np.sin(frequencies * indexes)
		outdata[:] = data.sum(axis=1, keepdims=True)
		start_idx += frames

	return multitone

try:
	blocksize = 48000
	multitone = multitone_generator(blocksize, 32)
	samplerate = sd.query_devices(args.device, 'output')['default_samplerate']

	def callback_generate_sine(outdata, frames, time, status):
		if status:
			print(status, file=sys.stderr)
		multitone(outdata, frames)
		
	def callback_check_sine(data: np.ndarray, frames: int, time, status):
		if frames == 0:
			return
		audioQueue.put(data.copy())

	with sd.OutputStream(device=args.device, channels=1, callback=callback_generate_sine,
						 samplerate=samplerate, blocksize=blocksize):
		with sd.InputStream(device=args.device, channels=1, callback=callback_check_sine,
							samplerate=samplerate, blocksize=blocksize):
			while True:
				data: np.ndarray = audioQueue.get()

				input_data = data[:, 0]

				plt.clf()
				x = np.linspace(0, input_data.size, num=input_data.size, endpoint=False)
				plt.plot(x, input_data)
				plt.show()

				y = np.fft.rfft(input_data)

				plt.clf()
				x = np.fft.rfftfreq(input_data.size, d=1./samplerate)
				plt.plot(x, np.abs(y))
				plt.show()

				with audioQueue.mutex:
					audioQueue.queue.clear()
except KeyboardInterrupt:
	parser.exit('')
except Exception as e:
	parser.exit(type(e).__name__ + ': ' + str(e))
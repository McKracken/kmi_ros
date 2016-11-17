#!/usr/bin/env python

import sys
import argparse

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Argparse Test')
	parser.add_argument("-e", "--echo", help="echo the string you use here")
	args = parser.parse_args()
	print args.echo
	

	

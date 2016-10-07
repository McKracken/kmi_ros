#!/usr/bin/env python

from Tkinter import Tk
from Tkinter import Frame
from Tkinter import Button
import tkFont
import tkMessageBox

class ActivityManager(object):

	def init_tk(self):
		self.top = Tk()
		self.top.geometry('400x400')
		helv36 = tkFont.Font(family='Helvetica', size=36, weight='bold')  # you don't have to use Helvetica or bold, this is just an example

		f = Frame(self.top, height=200, width=200)
		f.pack_propagate(0) # don't shrink
		f.pack()
		B = Button(f, text ="OK", command = self.helloCallBack)
		B['font'] = helv36
		B.pack()
		self.top.mainloop()


	def helloCallBack(self):
		self.top.destroy()

 #  tkMessageBox.showinfo( "Hello Python", "Hello World")

if __name__ == "__main__":
	for i in range(0,3):
		ac = ActivityManager()
		ac.init_tk()
		print "updating"
		i += 1

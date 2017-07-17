"""
Interactive Drawing Interface for taking in line inputs. Allows
users to draw on a surface and then save to a file. 
"""
import Tkinter

trace = 0 

# XXX top left is (0, 0). Do we need to correct the frame?

class CanvasEventsDemo(object):
    """
    Interactive Drawing Interface for geting line input
    """
    def __init__(self, parent=None):
        """
        Initialization for GUI
        """
        @param parent parent object
        self.root = Tkinter.Tk()
        self.height = 500
        self.width = 500
        canvas = Tkinter.Canvas(width=self.width,
                    height=self.height, bg='black')
        canvas.grid(row=0, column=0, columnspan=3)
        self.addButtons()

        canvas.bind('<ButtonPress-1>', self.onStart)
        canvas.bind('<ButtonRelease-1>', self.onFinish)
        canvas.bind('<B1-Motion>', self.onGrow)
        self.canvas = canvas
        self.drawn = None

        self.lines = []

    def addButtons(self):
        """
        Add buttons for file name entry, the 'save and exit'
        button and 'reset' button
        """
        frame = Tkinter.Frame()
        self.filename_entry = Tkinter.Entry(width=30)
        self.filename_entry.insert(0, 'fileName')
        exit_button = Tkinter.Button(frame, text="Save and Exit",
                                     command=self.quit)
        reset_button = Tkinter.Button(frame, text="Clear",
                                      command=self.onClear)

        frame.grid(row=1, column=0)
        self.filename_entry.grid(row=1, column=2)
        exit_button.grid(row=1, column=0)
        reset_button.grid(row=1, column=1)

    def onStart(self, event):
        """
        Start the line drawing
        """
        self.start = event
        self.drawn = None

    def onGrow(self, event):
        """
        Continue the drawing
        """
        canvas = event.widget
        if self.drawn:
            canvas.delete(self.drawn)
        objectId = canvas.create_line(self.start.x, self.start.y, event.x,
                              event.y, fill='white', width=2)
        self.drawn = objectId
 
    def onFinish(self, event):
        """
        Upon finishing a line, record the coordinates
        @param event End line position for logging
        """
        self.lines.append([self.start.x, self.start.y, event.x, event.y])

    def onClear(self):
        """
        Delete all the lines
        """
        self.canvas.delete('all')
        self.lines = []

    def quit(self):
        """
        Quit and save all the lines
        """
        filename = self.filename_entry.get()
        writeLines(self.height, self.width, filename, self.lines)
        self.root.destroy()

def writeLines(height, width, filename, lines):
    """
    Write lines to a file
    @param height Height of the drawing space
    @param width Width of the drawing space
    @param filename Name of the file to save
    @param lines List of drawn lines to save 
    """
    target = open('../inputs/{}.txt'.format(filename), 'w')
    target.write('0 0 {} {}'.format(height, width))

    for i in xrange(len(lines)):
        target.write('\nL {} {} {} {}'.format(lines[i][0],
          lines[i][1], lines[i][2], lines[i][3]))
    target.close()
    

if __name__ == '__main__':
    CanvasEventsDemo()
    Tkinter.mainloop()

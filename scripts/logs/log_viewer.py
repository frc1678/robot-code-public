#!/usr/bin/python
from collections import OrderedDict
from Tkinter import Tk, BOTH, N, S, E, W, Toplevel, StringVar, IntVar, Frame, HORIZONTAL, VERTICAL
from ttk import Button, Style, Treeview, Scrollbar, Radiobutton, Checkbutton
import tkFileDialog, tkMessageBox
from hackplotlib import HackPlot
import sys, csv, os.path

# "foo.bar.biz" -> ["foo", "foo.bar", "foo.bar.biz"], etc.
def ancestor_names(name):
    if name == "":
        return []
    else:
        return ancestor_names(".".join(name.split(".")[:-1])) + [name]

class Example(Frame):
    def __init__(self):
        Frame.__init__(self)
        self.style = Style()
        self.style.theme_use("default")
        self.master.title("Log viewer")
        self.pack(fill=BOTH, expand=True)

        self.used = []  # List of currently plotted series ([str])
        self.series = {}  # List of all series ({str -> [number]})
        self.names = []  # List of all nodes in tree view ([str])
        self.queues = [] # List of all queues ([str])
        self.logs = {} # List of all text logs ({str -> [str]})

        self.rowconfigure(1, weight=1)
        self.columnconfigure(6, weight=3)
        self.columnconfigure(11, weight=1)

        # Series selection takes row 1-2, col 0-2
        self.series_ui = Treeview(self)
        self.series_ui.grid(row=1, column=0, columnspan=2, rowspan=2, sticky=N+S)
        self.series_ui.configure(show="tree")
        self.series_ui.bind("<Double-Button-1>", self.onselect)
        self.series_ui.tag_configure("graphall", foreground="#070")
        self.series_ui.tag_configure("graphnone", foreground="#000")
        self.series_ui.tag_configure("graphsome", foreground="#007")
        series_ui_scroll = Scrollbar(self, command=self.series_ui.yview, orient=VERTICAL)
        series_ui_scroll.grid(row=1, column=2, rowspan=2, sticky=N+S)
        self.series_ui["yscrollcommand"] = series_ui_scroll.set

        # The plot takes row 1-2, col 3-6
        move_mode = StringVar()
        move_mode.set("pan")
        show_path = IntVar()
        show_path.set(0)
        event_bars = IntVar()
        event_bars.set(1)
        show_debug = IntVar()
        show_debug.set(1)
        show_error = IntVar()
        show_error.set(1)
        show_warning = IntVar()
        show_warning.set(1)
        show_info = IntVar()
        show_info.set(1)
        self.plot = HackPlot(self, move_mode, show_path, event_bars,
                [(show_debug, "[DEBUG]"), (show_error, "[ERROR]"), (show_warning, "[WARNING]"), (show_info, "[INFO]")])
        self.plot.canvas.grid(row=1, column=3, columnspan=4, rowspan=2, sticky=N+S+E+W)
        # Text logs take row 1-2, col 7-12
        self.plot.listbox.grid(row=1, column=7, columnspan=5, sticky=N+S+E+W)
        listbox_yscroll = Scrollbar(self, command=self.plot.listbox.yview, orient=VERTICAL)
        listbox_yscroll.grid(row=1, column=12, sticky=N+S)
        self.plot.listbox["yscrollcommand"] = listbox_yscroll.set
        listbox_xscroll = Scrollbar(self, command=self.plot.listbox.xview, orient=HORIZONTAL)
        listbox_xscroll.grid(row=2, column=7, columnspan=5, sticky=E+W)
        self.plot.listbox["xscrollcommand"] = listbox_xscroll.set


        # Controls take row 0, col 0-12
        Button(self, text="Load Directory", command=self.loaddir).grid(row=0, column=0)
        Button(self, text="Load File", command=self.loadfile).grid(row=0, column=1)
        Button(self, text="Fit X", command=self.plot.fit_x).grid(row=0, column=3, sticky=W)
        Button(self, text="Fit Y", command=self.plot.fit_y).grid(row=0, column=4, sticky=W)
        # Plot controls in a subframe to manage padding so it doesn't look awful
        move_mode_control = Frame(self, padx=10)
        Radiobutton(move_mode_control, text="Pan", value="pan", variable=move_mode).grid(row=0, column=0, sticky=W)
        Radiobutton(move_mode_control, text="Zoom In", value="zoomin", variable=move_mode).grid(row=0, column=1, sticky=W)
        Radiobutton(move_mode_control, text="Zoom Out", value="zoomout", variable=move_mode).grid(row=0, column=2, sticky=W)
        move_mode_control.grid(row=0, column=5, sticky=W)
        Checkbutton(self, text="Event Bars", variable=event_bars, command=self.plot.show_textlogs).grid(row=0, column=6, sticky=W)
        Checkbutton(self, text="Debug", variable=show_debug, command=self.plot.show_textlogs).grid(row=0, column=7, sticky=W)
        Checkbutton(self, text="Error", variable=show_error, command=self.plot.show_textlogs).grid(row=0, column=8, sticky=W)
        Checkbutton(self, text="Warning", variable=show_warning, command=self.plot.show_textlogs).grid(row=0, column=9, sticky=W)
        Checkbutton(self, text="Info", variable=show_info, command=self.plot.show_textlogs).grid(row=0, column=10, sticky=W)
        Checkbutton(self, text="Directories", variable=show_path, command=self.plot.show_textlogs).grid(row=0, column=11, sticky=E)

    # Open Directory button clicked
    def loaddir(self):
        dirname = tkFileDialog.askdirectory(initialdir=log_dir)
        if dirname != "" and dirname != (): # Cancel and (x) not pressed
            # Empty the data
            self.used = []
            self.series = {}
            self.names = []
            self.queues = []
            self.logs = {}
            self.plot.reset()
            for node in self.series_ui.get_children():
                self.series_ui.delete(node)
            # For every csv file in the directory, checking recursively and alphabetically
            for subdirname, subsubdirnames, filenames in os.walk(dirname):
                subsubdirnames.sort()
                filenames.sort()
                for filename in filenames:
                    if filename.endswith(".csv") or filename.endswith(".log"):
                        # The name of the directory without the name of the directory selected,
                        # and the name of the file without the extension, separated by "."s
                        # For example if directory selected is /tmp/logs, subdirname is /tmp/logs/beta/666,
                        # and filename is foo.csv, nodeprefix is "beta.666.foo".
                        nodeprefix = ".".join(subdirname[len(dirname)+1:].split("/") + [filename[:-4]])
                        if nodeprefix.startswith("."):
                            nodeprefix = nodeprefix[1:]
                        # Add the file's data
                        self.readfile(subdirname + "/" + filename, nodeprefix)
            for name in self.names:
                # Add name to (name with everything after last dot removed), as the last child,
                # with text (everything after last dot of name) and tags graphnone and (whether name represents data)
                self.series_ui.insert(".".join(name.split(".")[0:-1]), "end", name,
                        text=name.split(".")[-1], tags=["graphnone"])

    # Open File button clicked
    def loadfile(self):
        filename = tkFileDialog.askopenfilename(filetypes=[("CSV Files", "*.csv"), ("Text log file", "*.log")], initialdir=log_dir)
        if filename != "" and filename != (): # Cancel and (x) not pressed
            # Empty the data
            self.used = []
            self.series = {}
            self.names = []
            self.queues = []
            self.logs = {}
            self.plot.reset()
            for node in self.series_ui.get_children():
                self.series_ui.delete(node)
            # Add the file's data
            self.readfile(filename, "")
            for name in self.names:
                # Add name to (name with everything after last dot removed), as the last child,
                # with text (everything after last dot of name) and tags graphnone and (whether name represents data)
                self.series_ui.insert(".".join(name.split(".")[0:-1]), "end", name,
                        text=name.split(".")[-1], tags=["graphnone"])

    # Add a file's data
    # nodeprefix is a string to add before every name in the file, represents file's name in the tree
    def readfile(self, filename, nodeprefix):
        try:
            if filename.endswith(".csv"):
                # For csv files this will always be concatenated with something
                if nodeprefix != "":
                    nodeprefix += "."
                csvfile = open(filename, "rb")
                reader = csv.DictReader(csvfile)
                series = {}
                self.queues.append(nodeprefix[:-1])
                for name in reader.fieldnames:
                    # Add ancestor_names to names, without creating duplicates
                    self.names = list(OrderedDict.fromkeys(self.names + ancestor_names(nodeprefix + name)))
                    # Create a series for this field
                    series[nodeprefix + name] = []
                for row in reader:
                    for name in reader.fieldnames:
                        try:
                            # Add cell to series if it represents a number
                            series[nodeprefix + name].append(float(row[name]))
                        except ValueError:
                            # Not a number, no problem, could be game_specific_string or something
                            pass
                self.series.update(series)
            else:
                self.names.append(nodeprefix)
                self.logs[nodeprefix] = open(filename, "r").readlines()
        except IOError:
            tkMessageBox.showerror(message="Could not open file: " + str(filename))

    # Tree element was double clicked
    def onselect(self, _):
        series = self.series_ui.focus()
        # Set it to graph if no children are graphed, and not to graph if all or some are
        self.setgraphed(series, self.series_ui.tag_has("graphnone", series))
        self.checkgraphed(series)

    # Set a node and its children to be either all or none graphed
    def setgraphed(self, node, shouldgraph):
        if shouldgraph:
            self.series_ui.item(node, tags=["graphall"])
            # If the node represents data and it isn't already graphed, graph it
            if node in self.series and node not in self.used:
                self.used.append(node)
                # Timestamp is queue that contains node + ".timestamp"
                timestamp_name = [queue for queue in self.queues if node.startswith(queue)][0] + ".timestamp"
                self.plot.add_plot(self.series[timestamp_name], self.series[node], node)
            if node in self.logs and node not in self.used:
                self.used.append(node)
                self.plot.add_textlog(self.logs[node], node)
            # If nothing else is plotted, fit the plot to this
            if len(self.used) == 1:
                self.plot.fit_x()
                self.plot.fit_y()
            for child in self.series_ui.get_children(node):
                self.setgraphed(child, True)
        else:
            # Set tags to be (whether node represents data) and graphnone
            self.series_ui.item(node, tags=["graphnone"])
            if node in self.used:
                self.used.remove(node)
                if node in self.logs:
                    self.plot.remove_textlog(node)
                if node in self.series:
                    self.plot.remove_plot(node)
            for child in self.series_ui.get_children(node):
                self.setgraphed(child, False)

    # Update the tags (and color) on a node and its ancestors, should always be called after setgraphed
    def checkgraphed(self, node):
        # Top level nodes are children of ""
        if node == "":
            return
        # True unless a child doesn't share this tag or there are no children
        graphall = True
        graphnone = True
        for child in self.series_ui.get_children(node):
            if not self.series_ui.tag_has("graphall", child):
                graphall = False
            if not self.series_ui.tag_has("graphnone", child):
                graphnone = False
        graphtag = ""
        if graphall and graphnone:
            # There are no children, check the used list instead
            graphtag = "graphall" if node in self.used else "graphnone"
        elif graphall:
            graphtag = "graphall"
        elif graphnone:
            graphtag = "graphnone"
        else:
            graphtag = "graphsome"
        # Set tags to be (whether node represents data) and the computed status
        self.series_ui.item(node, tags=[graphtag])
        # Now that the status of this node is known, check the parent
        self.checkgraphed(self.series_ui.parent(node))

log_dir = "/tmp" # Defult value, overriden by first command line argument

if __name__ == '__main__':
    if len(sys.argv) > 1:
        log_dir = sys.argv[1]
    root = Tk()
    root.geometry("1200x500+300+300")
    app = Example()
    root.mainloop()

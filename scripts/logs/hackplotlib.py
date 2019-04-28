from Tkinter import Canvas, NE, NW, Listbox, END, DISABLED, NORMAL
import math
import itertools
import re


class Plot(object):
    def __init__(self):
        self.yvals = []
        self.xvals = []
        self.points = []
        self.color = ""


colors = ["blue", "red", "green", "cyan", "magenta", "yellow", "black"]


class HackPlot(object):
    def __init__(self, parent, move_mode, show_path, use_event_bars,
                 log_levels):
        self.canvas = Canvas(parent, bg="white")
        self.listbox = Listbox(
            parent,
            width=20,
            bg="white",
            state="disabled",
            disabledforeground="black")
        self.canvas.bind("<Configure>", self.redraw)
        self.canvas.bind("<ButtonPress-1>", self.mousedown)
        self.canvas.bind("<ButtonRelease-1>", self.mouseup)
        self.canvas.bind("<Motion>", self.mousemove)
        self.canvas.bind("<Leave>", self.mouseleave)
        self.canvas.bind("<ButtonPress-3>", self.cancelzoom)
        self.move_mode = move_mode
        self.show_path = show_path
        self.use_event_bars = use_event_bars
        self.log_levels = log_levels
        self.reset()

    def redraw(self, event):
        self.canvas.delete("all")
        self.legend = []
        self.draw_boundary()
        self.zoom_trace = None
        self.show_textlogs()
        for name in self.plots:
            self.draw_plot(name)
        self.draw_legend()

    def reset(self):
        self.x_low = -0.5
        self.x_high = 0.5
        self.y_low = -0.5
        self.y_high = 0.5
        self.mouse_start = None
        self.color_index = 0
        self.compute_width_height()
        self.draw_boundary()
        self.plots = {}
        self.legend = []
        self.event_bars = []
        self.logs = {}
        self.coord_text = None
        self.zoom_trace = None
        self.redraw(None)

    def compute_width_height(self):
        self.plot_width = self.x_high - self.x_low
        self.plot_height = self.y_high - self.y_low

    # Draw outline of plot area and axis ticks
    def draw_boundary(self):
        for x in self.get_ticks(self.x_low, self.x_high):
            screen_x, _ = self.screen_coords(x, 0)
            self.canvas.create_line(
                screen_x,
                self.canvas.winfo_height() * 0.1,
                screen_x,
                self.canvas.winfo_height() * 0.9,
                fill="light grey")
            self.canvas.create_text(
                screen_x, self.canvas.winfo_height() * 0.95, text=str(x))
        for y in self.get_ticks(self.y_low, self.y_high):
            _, screen_y = self.screen_coords(0, y)
            self.canvas.create_line(
                self.canvas.winfo_width() * 0.1,
                screen_y,
                self.canvas.winfo_width() * 0.9,
                screen_y,
                fill="light grey")
            self.canvas.create_text(
                self.canvas.winfo_width() * 0.05, screen_y, text=str(y))
        self.canvas.create_line(self.canvas.winfo_width() * 0.1,
                                self.canvas.winfo_height() * 0.1,
                                self.canvas.winfo_width() * 0.9,
                                self.canvas.winfo_height() * 0.1,
                                self.canvas.winfo_width() * 0.9,
                                self.canvas.winfo_height() * 0.9,
                                self.canvas.winfo_width() * 0.1,
                                self.canvas.winfo_height() * 0.9,
                                self.canvas.winfo_width() * 0.1,
                                self.canvas.winfo_height() * 0.1)

    def draw_legend(self):
        legend_width = 0
        legend_height = 0
        # Clear the legend, since new plots don't necessarily appear last
        for oid in self.legend:
            self.canvas.delete(oid)
        for name in self.plots:
            # Width = widest text, increase height with number of plots
            legend_width = max(legend_width, len(name) * 6)
            legend_height += 15
        # Add some padding
        legend_width += 10
        legend_height += 10
        if self.plots != {}:
            self.legend.append(
                self.canvas.create_rectangle(
                    self.canvas.winfo_width() - legend_width,
                    0,
                    self.canvas.winfo_width(),
                    legend_height,
                    fill="white"))
        index = 0
        for name in self.plots:
            self.legend.append(
                self.canvas.create_text(
                    self.canvas.winfo_width() - 5,
                    index * 15 + 5,
                    text=name,
                    fill=self.plots[name].color,
                    anchor=NE))
            index += 1

    def get_ticks(self, bound_low, bound_high):
        size = bound_high - bound_low
        # step is size rounded down to next power of 10
        step = 10**math.floor(math.log10(size))
        # Adjust the step so there are good number of ticks
        if size / step > 6:
            step *= 2
        elif size / step < 1.2:
            step /= 5
        elif size / step < 2:
            step /= 2
        return [
            i * step for i in range(
                int(math.ceil(bound_low /
                              step)), int(math.ceil(bound_high / step + 1e-3)))
        ]

    def screen_coords(self, x, y):
        screen_x = ((x - self.x_low) / self.plot_width * 0.8 +
                    0.1) * self.canvas.winfo_width()
        screen_y = ((self.y_high - y) / self.plot_height * 0.8 +
                    0.1) * self.canvas.winfo_height()
        return (screen_x, screen_y)

    def plot_coords(self, x, y):
        plot_x = (float(x) / self.canvas.winfo_width() -
                  0.1) / 0.8 * self.plot_width + self.x_low
        plot_y = self.y_high - (float(y) / self.canvas.winfo_height() -
                                0.1) / 0.8 * self.plot_height
        return (plot_x, plot_y)

    def add_plot(self, xvals, yvals, name):
        self.plots[name] = Plot()
        self.plots[name].xvals = xvals
        self.plots[name].yvals = yvals
        self.plots[name].color = colors[self.color_index]
        self.color_index = (self.color_index + 1) % len(colors)
        self.draw_plot(name)
        self.draw_legend()

    def remove_plot(self, name):
        for oid in self.plots[name].points:
            self.canvas.delete(oid)
        self.plots.pop(name)
        self.draw_legend()

    def add_textlog(self, strings, name):
        # Each entry is pair of (first number in string) and (string with ending newline removed)
        self.logs[name] = [(int(re.search("\\d+", string).group()),
                            string[:-1]) for string in strings]
        self.show_textlogs()

    def remove_textlog(self, name):
        self.logs.pop(name)
        self.show_textlogs()

    def draw_plot(self, name):
        color = self.plots[name].color
        points = []
        for x, y in itertools.izip(self.plots[name].xvals,
                                   self.plots[name].yvals):
            if x >= self.x_low and x <= self.x_high and y >= self.y_low and y <= self.y_high:
                screen_x, screen_y = self.screen_coords(x, y)
                points.append(
                    self.canvas.create_rectangle(
                        screen_x - 1,
                        screen_y - 1,
                        screen_x + 1,
                        screen_y + 1,
                        outline=color))

        self.plots[name].points = points

    def show_textlogs(self):
        for oid in self.event_bars:
            self.canvas.delete(oid)
        self.event_bars = []
        # Combine all logs into chronological order
        logs = []
        for _, log in self.logs.iteritems():
            logs += log
        logs.sort()
        # Needs to be normal state to add and remove elements
        self.listbox["state"] = NORMAL
        self.listbox.delete(0, END)
        for timestamp, string in logs:
            # Check whether the checkbox matching the start of the string is off
            level_included = True
            for level in self.log_levels:
                if string.startswith(level[1]) and level[0].get() == 0:
                    level_included = False
            if timestamp >= self.x_low and timestamp <= self.x_high and level_included:
                if self.show_path.get() == 0:
                    # Log format is [level]time:/path/to/file.cpp:line: message here
                    # To remove directory name, replace first ':' to last '/' before a ':' with a single ':'
                    string = ":".join(re.split(":[^:]*/", string))
                self.listbox.insert(END, string)
                screen_x, _ = self.screen_coords(float(timestamp), 0)
                if self.use_event_bars.get():
                    self.event_bars.append(
                        self.canvas.create_line(
                            screen_x,
                            self.canvas.winfo_height() * 0.1,
                            screen_x,
                            self.canvas.winfo_height() * 0.9,
                            fill="grey"))
        self.listbox["state"] = DISABLED

    def fit_x(self):
        self.x_low = 1e100
        self.x_high = -1e100

        # For x range, check data and text logs
        for _, plot in self.plots.iteritems():
            self.x_low = min(self.x_low, min(plot.xvals))
            self.x_high = max(self.x_high, max(plot.xvals))
        for _, logs in self.logs.iteritems():
            self.x_low = min(self.x_low, logs[0][0])
            self.x_high = max(self.x_high, logs[-1][0])
        if self.x_low == self.x_high:
            # Add to range to avoid dividing bx 0
            self.x_low -= 0.5
            self.x_high += 0.5
        elif self.x_low > self.x_high:
            # There is no data, reset x range
            self.x_low = -0.5
            self.x_high = 0.5
        self.compute_width_height()
        self.redraw(None)

    def fit_auto(self):
        self.x_low = 1e100
        self.x_high = -1e100
        # For x range, check data and text logs
        mode = self.plots['driver_station_status.mode'].yvals
        got_one = False
        finished = False
        for i in range(0, len(mode)):
            if got_one == False:
                if mode[i] == 1.0:
                    got_one = True
                    self.x_low = self.plots[
                        'driver_station_status.mode'].xvals[i]
            if finished == False and got_one == True:
                if mode[i] == 0.0:
                    self.x_high = self.plots[
                        'driver_station_status.mode'].xvals[i - 1]
                    finished = True

                    self.compute_width_height()
                    self.redraw(None)
                    return
        self.fit_x()

    def fit_tele(self):
        self.x_low = 1e100
        self.x_high = -1e100
        # For x range, check data and text logs
        mode = self.plots['driver_station_status.mode'].yvals
        got_one = False
        finished = False
        for i in range(0, len(mode)):
            if got_one == False:
                if mode[i] == 2.0:
                    got_one = True
                    self.x_low = self.plots[
                        'driver_station_status.mode'].xvals[i]
            if finished == False and got_one == True:
                if mode[i] == 0.0:
                    self.x_high = self.plots[
                        'driver_station_status.mode'].xvals[i - 1]
                    self.compute_width_height()
                    self.redraw(None)
                    return

        self.fit_x()


    def fit_y(self):
        self.y_low = 1e100
        self.y_high = -1e100
        for _, plot in self.plots.iteritems():
            self.y_low = min(self.y_low, min(plot.yvals))
            self.y_high = max(self.y_high, max(plot.yvals))
        if self.y_low == self.y_high:
            # Add to range to avoid dividing by 0
            self.y_low -= 0.5
            self.y_high += 0.5
        elif self.y_low > self.y_high:
            # There is no data, reset y range
            self.y_low = -0.5
            self.y_high = 0.5
        self.compute_width_height()
        self.redraw(None)

    def mousedown(self, event):
        self.mouse_start = self.plot_coords(event.x, event.y)

    def mouseup(self, event):
        mouse_end = self.plot_coords(event.x, event.y)
        if self.mouse_start == None:
            return
        if self.mouse_start[0] == mouse_end[0] or self.mouse_start[1] == mouse_end[1]:
            # This would require dividing by 0
            self.mouse_start = None
            return
        if self.move_mode.get() == "zoomin" and self.mouse_start != None:
            # Set plot bounds to selection bounds
            self.x_low = min(self.mouse_start[0], mouse_end[0])
            self.x_high = max(self.mouse_start[0], mouse_end[0])
            self.y_low = min(self.mouse_start[1], mouse_end[1])
            self.y_high = max(self.mouse_start[1], mouse_end[1])
        elif self.move_mode.get() == "zoomout" and self.mouse_start != None:
            selection_x_low = min(self.mouse_start[0], mouse_end[0])
            selection_x_high = max(self.mouse_start[0], mouse_end[0])
            selection_y_low = min(self.mouse_start[1], mouse_end[1])
            selection_y_high = max(self.mouse_start[1], mouse_end[1])
            # Set plot bounds such that current bounds fit in selection, ie,
            # scale zone around selection be ratio of sizes
            self.x_low -= (selection_x_low - self.x_low) * self.plot_width / (
                selection_x_high - selection_x_low)
            self.x_high += (
                self.x_high - selection_x_high) * self.plot_width / (
                    selection_x_high - selection_x_low)
            self.y_low -= (selection_y_low - self.y_low) * self.plot_height / (
                selection_y_high - selection_y_low)
            self.y_high += (
                self.y_high - selection_y_high) * self.plot_height / (
                    selection_y_high - selection_y_low)
        self.compute_width_height()
        self.redraw(None)
        self.mouse_start = None

    def mousemove(self, event):
        mouse_end = self.plot_coords(event.x, event.y)
        if self.move_mode.get() == "pan" and self.mouse_start != None:
            self.x_low += self.mouse_start[0] - mouse_end[0]
            self.x_high += self.mouse_start[0] - mouse_end[0]
            self.y_low += self.mouse_start[1] - mouse_end[1]
            self.y_high += self.mouse_start[1] - mouse_end[1]
            self.redraw(None)
        elif self.move_mode.get() in ["zoomin", "zoomout"
                                      ] and self.mouse_start != None:
            self.canvas.delete(self.zoom_trace)
            mouse_start_screen = self.screen_coords(self.mouse_start[0],
                                                    self.mouse_start[1])
            bound_left = min(mouse_start_screen[0], event.x)
            bound_right = max(mouse_start_screen[0], event.x)
            bound_top = min(mouse_start_screen[1], event.y)
            bound_bottom = max(mouse_start_screen[1], event.y)
            self.zoom_trace = self.canvas.create_rectangle(
                bound_left, bound_top, bound_right, bound_bottom)
        if self.coord_text != None:
            self.canvas.delete(self.coord_text)
        self.coord_text = self.canvas.create_text(
            10,
            10,
            text="({:.6}, {:.6})".format(mouse_end[0], mouse_end[1]),
            anchor=NW)

    def mouseleave(self, event):
        if self.coord_text != None:
            self.canvas.delete(self.coord_text)
            self.coord_text = None
        self.cancelzoom(None)

    def cancelzoom(self, event):
        self.mouse_start = None
        self.canvas.delete(self.zoom_trace)

# Log Viewer User Guide

**Load Directory and Load File:** Do pretty much what they say.
  They also unload the current data, so if you want to plot from
  more than one file, you should use the Load Directory button.

**Tree View (the thing on the left):** Click on the arrow to expand
  an element. Double click to toggle whether it is plotted
  (cycles some-\>none-\>all-\>none). An element is black when no
  children are plotted, blue when some are, and green when all are.

**Fit X:** Set x range to be range of the data and text logs.

**Fit Y:** Set y range to be y range of data within x range.

**Pan:** Click and drag to move plot. Mouse keeps same coordinate.

**Zoom In:** Click and drag to scale rectangle selected to full plot.
  Move out of graph area or right click to cancel.

**Zoom Out:** Click and drag to scale such that current range fits in rectangle selected.
  Move out of graph area or right click to cancel.

**Event Bars:** When selected, a vertical bar is drawn on the plot when
  a text log occurs. Good for visualizing timing, but can get spammy.

**Debug, Warning, Error, and Info:** Whether log messages of that type will be displayed.
  Fatal errors are always displayed.

**Directories:** When unselected, the directory of the source file emitting the logs
  is hidden. This makes the text much more compact but may cause ambiguity in some cases.

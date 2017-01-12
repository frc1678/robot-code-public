class Logger(object):
    reset_color = "\033[1;m"
    red_color = "\033[1;31m"
    yellow_color = "\033[1;33m"
    blue_color = "\033[1;34m"
    grey_color = "\033[1;30m"

    loglevels = {
        0: red_color +    "[ERROR] " + reset_color + "{}",
        1: yellow_color + "[WARN]  " + reset_color + "{}",
        2: blue_color +   "[INFO]  " + reset_color + "{}",
        3: grey_color +   "[DEBUG] " + reset_color + "{}"
    }

    show_log_levels = 2

    @classmethod
    def _log(logger_class, message, loglevel):
        if loglevel <= logger_class.show_log_levels:
            print(logger_class.loglevels[loglevel].format(message))

    @classmethod
    def error(logger_class, message):
        logger_class._log(message, 0)

    @classmethod
    def warn(logger_class, message):
        logger_class._log(message, 1)

    @classmethod
    def info(logger_class, message):
        logger_class._log(message, 2)

    @classmethod
    def debug(logger_class, message):
        logger_class._log(message, 3)

#include <iostream>
#include <unistd.h>
#include <sys/wait.h>

#include "gflags/gflags.h"

pid_t code_pid = 0;

void handle_signal(int signum) {
  std::cout << "Signal (" << signum << ") received." << std::endl;
  if (code_pid > 0) {
    std::cout << "Killing robot code (PID " << code_pid << ")." << std::endl;
    kill(code_pid, SIGTERM);
  } else {
    std::cout << "Robot code not started, exiting" << std::endl;
  }
  exit(signum);
}


int main(int argc, char **argv) {
 signal(SIGINT, handle_signal);
 signal(SIGTERM, handle_signal);

 gflags::ParseCommandLineFlags(&argc, &argv, true);

 //TODO(Wesley) Check if autostart is already running

 int status;
 while (true) {
   code_pid = fork();
   switch (code_pid) {
     case -1:
       perror("fork");
       break;
     case 0:
       execv(argv[1], (argv+1));
       perror("exec");
       break;
     default:
       std::cout << "Started code with PID " << code_pid << std::endl;
       if (waitpid(code_pid, &status, 0) != -1) {
         std::cout << "Code died with status " << status << std::endl;
       } else {
         perror("waitpid");
       }
       break;
   }
 }

 return 0;
}


#ifndef StdCapture_hpp
#define StdCapture_hpp

#include <mutex>
#include <string>

class StdCapture
{
public:
    static void Init();
    
    static void BeginCapture();
    static bool IsCapturing();
    static void EndCapture();
    static std::string GetCapture();
    
private:
    enum PIPES { READ, WRITE };
    
    static int secure_dup(int src);
    static void secure_pipe(int * pipes);
    static void secure_dup2(int src, int dest);
    
    static void secure_close(int & fd);
    
    static int m_pipe[2];
    static int m_oldStdOut;
    static int m_oldStdErr;
    static bool m_capturing;
    static std::mutex m_mutex;
    static std::string m_captured;
};

#endif

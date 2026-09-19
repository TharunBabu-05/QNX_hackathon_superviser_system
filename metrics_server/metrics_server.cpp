// metrics_server -- exposes live system/process/thread metrics as JSON over
// plain HTTP, so a browser dashboard can poll real numbers off this Pi.
//
// Everything here comes from QNX's documented procfs devctl interface
// (<sys/procfs.h>/<sys/debug.h>), the same interface pidin and the IDE's
// own process views are built on -- not by shelling out to pidin and
// scraping its text output, which would be one format change away from
// silently breaking.
//
// This is a read-only system-wide observer, independent of the safety
// supervisor's sensor IPC: it walks /proc, not the supervisor's channel,
// so it has no dependency on (or influence over) the rest of this project.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <time.h>
#include <devctl.h>
#include <sys/procfs.h>
#include <sys/states.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace {

constexpr int LISTEN_PORT = 8090;

uint64_t nowNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

const char* stateName(uint8_t s) {
    switch (s) {
    case STATE_DEAD:        return "DEAD";
    case STATE_RUNNING:     return "RUNNING";
    case STATE_READY:       return "READY";
    case STATE_STOPPED:     return "STOPPED";
    case STATE_SEND:        return "SEND";
    case STATE_RECEIVE:     return "RECEIVE";
    case STATE_REPLY:       return "REPLY";
    case STATE_MQ_SEND:     return "MQ_SEND";
    case STATE_MQ_RECEIVE:  return "MQ_RECEIVE";
    case STATE_WAITPAGE:    return "WAITPAGE";
    case STATE_SIGSUSPEND:  return "SIGSUSPEND";
    case STATE_SIGWAITINFO: return "SIGWAITINFO";
    case STATE_NANOSLEEP:   return "NANOSLEEP";
    case STATE_MUTEX:       return "MUTEX";
    case STATE_CONDVAR:     return "CONDVAR";
    case STATE_JOIN:        return "JOIN";
    case STATE_INTR:        return "INTR";
    case STATE_SEM:         return "SEM";
    case STATE_WAITCTX:     return "WAITCTX";
    case STATE_RWLOCK_READ: return "RWLOCK_READ";
    case STATE_RWLOCK_WRITE:return "RWLOCK_WRITE";
    case STATE_BARRIER:     return "BARRIER";
    case STATE_PIPE:        return "PIPE";
    default:                return "UNKNOWN";
    }
}

// Minimal JSON string escaping -- process/executable paths are the only
// free-form text this ever emits, and they're not expected to contain
// anything exotic, but a stray quote/backslash shouldn't be able to
// produce broken JSON.
std::string jsonEscape(const char* s) {
    std::string out;
    for (const char* p = s; *p; ++p) {
        switch (*p) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        default:
            if (static_cast<unsigned char>(*p) < 0x20) { /* skip control chars */ }
            else out += *p;
        }
    }
    return out;
}

// Convenience extension documented in <sys/procfs.h>: equivalent to
// DCMD_PROC_INFO followed by DCMD_PROC_MAPDEBUG on the base address, i.e.
// "what executable is this process running". The struct's `path` field is
// a flexible array, so the buffer passed to devctl() has to be larger
// than sizeof(procfs_debuginfo) for the kernel to have room to write the
// path into -- the declared struct size only covers the fixed header.
std::string processExeName(int fd) {
    constexpr size_t BUF_SIZE = 512;
    uint8_t buf[BUF_SIZE] = {0};
    auto* info = reinterpret_cast<procfs_debuginfo*>(buf);
    if (devctl(fd, DCMD_PROC_MAPDEBUG_BASE, info, BUF_SIZE, nullptr) != EOK) {
        return "?";
    }
    const char* path = reinterpret_cast<const char*>(info) + offsetof(procfs_debuginfo, path);
    const char* base = strrchr(path, '/');
    return base ? (base + 1) : path;
}

struct PrevCpu { uint64_t busyNs; };

// Keyed by pid alone for process-level CPU%, and by (pid<<32|tid) for
// per-thread CPU%. Persists across requests in this long-running process
// so "CPU%" means "since the last poll", the same way top/pidin -y report
// it -- there is no instrumented-kernel idle/busy counter available here,
// so this is derived from each process/thread's own accumulated run time.
std::unordered_map<pid_t, PrevCpu> g_prevProcess;
std::unordered_map<uint64_t, PrevCpu> g_prevThread;
uint64_t g_prevGatherNs = 0;

double cpuPercent(uint64_t prevBusyNs, uint64_t busyNs, uint64_t deltaWallNs, unsigned numCpus) {
    if (deltaWallNs == 0 || busyNs < prevBusyNs) return 0.0;
    const double pct = (static_cast<double>(busyNs - prevBusyNs) / static_cast<double>(deltaWallNs)) * 100.0;
    const double clamped = pct > (100.0 * numCpus) ? (100.0 * numCpus) : pct;
    return clamped < 0.0 ? 0.0 : clamped;
}

std::string gatherMetrics(uint64_t serverStartNs) {
    const uint64_t gatherNs = nowNs();
    const uint64_t deltaWallNs = (g_prevGatherNs == 0) ? 0 : (gatherNs - g_prevGatherNs);

    const unsigned numCpus = _syspage_ptr->num_cpu;
    const long physPages = sysconf(_SC_PHYS_PAGES);
    const long pageSize  = sysconf(_SC_PAGESIZE);
    const uint64_t totalMemBytes = (physPages > 0 && pageSize > 0)
        ? static_cast<uint64_t>(physPages) * static_cast<uint64_t>(pageSize) : 0;

    std::string json;
    json.reserve(16384);
    json += "{";
    json += "\"system\":{";
    json += "\"numCpus\":" + std::to_string(numCpus) + ",";
    json += "\"totalMemBytes\":" + std::to_string(totalMemBytes) + ",";
    json += "\"pageSize\":" + std::to_string(pageSize) + ",";
    json += "\"cyclesPerSec\":" + std::to_string(SYSPAGE_ENTRY(qtime)->cycles_per_sec) + ",";
    json += "\"serverUptimeSec\":" + std::to_string((gatherNs - serverStartNs) / 1000000000ULL);
    json += "},";

    json += "\"processes\":[";
    DIR* proc = opendir("/proc");
    bool firstProc = true;
    unsigned processCount = 0, threadCount = 0;
    double aggregateBusyDeltaNs = 0;

    if (proc) {
        struct dirent* ent;
        while ((ent = readdir(proc)) != nullptr) {
            const char* name = ent->d_name;
            bool numeric = *name != '\0';
            for (const char* p = name; *p; ++p) {
                if (*p < '0' || *p > '9') { numeric = false; break; }
            }
            if (!numeric) continue;

            const pid_t pid = static_cast<pid_t>(atoi(name));
            std::string asPath = std::string("/proc/") + name + "/as";
            const int fd = open(asPath.c_str(), O_RDONLY);
            if (fd == -1) continue;

            procfs_info info{};
            if (devctl(fd, DCMD_PROC_INFO, &info, sizeof(info), nullptr) != EOK) {
                close(fd);
                continue;
            }

            const std::string exeName = processExeName(fd);
            const uint64_t busyNs = info.utime + info.stime;
            auto prevIt = g_prevProcess.find(pid);
            const double pct = (prevIt != g_prevProcess.end())
                ? cpuPercent(prevIt->second.busyNs, busyNs, deltaWallNs, numCpus) : 0.0;
            if (prevIt != g_prevProcess.end() && deltaWallNs > 0 && busyNs >= prevIt->second.busyNs) {
                aggregateBusyDeltaNs += static_cast<double>(busyNs - prevIt->second.busyNs);
            }
            g_prevProcess[pid] = {busyNs};

            if (!firstProc) json += ",";
            firstProc = false;
            processCount++;

            char numbuf[64];
            snprintf(numbuf, sizeof(numbuf), "%.1f", pct);

            json += "{\"pid\":" + std::to_string(pid) + ",";
            json += "\"name\":\"" + jsonEscape(exeName.c_str()) + "\",";
            json += "\"numThreads\":" + std::to_string(info.num_threads) + ",";
            json += "\"priority\":" + std::to_string(info.priority) + ",";
            json += "\"memPrivateBytes\":" + std::to_string(info.private_mem) + ",";
            json += "\"cpuPercent\":" + std::string(numbuf) + ",";

            json += "\"threads\":[";
            const unsigned tidLimit = info.num_threads + 16 > 128 ? 128 : info.num_threads + 16;
            unsigned foundThreads = 0;
            bool firstThread = true;
            for (unsigned tid = 1; tid <= tidLimit && foundThreads < info.num_threads; ++tid) {
                procfs_status st{};
                st.tid = tid;
                if (devctl(fd, DCMD_PROC_TIDSTATUS, &st, sizeof(st), nullptr) != EOK) continue;
                if (static_cast<unsigned>(st.tid) != tid) continue; // slot didn't resolve to a live thread
                foundThreads++;
                threadCount++;

                const uint64_t key = (static_cast<uint64_t>(pid) << 32) | tid;
                auto tPrevIt = g_prevThread.find(key);
                const double tPct = (tPrevIt != g_prevThread.end())
                    ? cpuPercent(tPrevIt->second.busyNs, st.sutime, deltaWallNs, 1) : 0.0;
                g_prevThread[key] = {st.sutime};

                char tnumbuf[64];
                snprintf(tnumbuf, sizeof(tnumbuf), "%.1f", tPct);

                if (!firstThread) json += ",";
                firstThread = false;
                json += "{\"tid\":" + std::to_string(tid) + ",";
                json += "\"state\":\"" + std::string(stateName(st.state)) + "\",";
                json += "\"priority\":" + std::to_string(st.priority) + ",";
                json += "\"policy\":" + std::to_string(st.policy) + ",";
                json += "\"lastCpu\":" + std::to_string(st.last_cpu) + ",";
                json += "\"cpuPercent\":" + std::string(tnumbuf) + "}";
            }
            json += "]}";

            close(fd);
        }
        closedir(proc);
    }
    json += "],";

    // Summed across every process's busy-time delta, over one wall-clock
    // delta -- on an N-core box this can legitimately read up to N*100%,
    // same convention pidin/top use for aggregate CPU load.
    char aggBuf[64];
    snprintf(aggBuf, sizeof(aggBuf), "%.1f",
             (deltaWallNs > 0) ? (aggregateBusyDeltaNs / static_cast<double>(deltaWallNs) * 100.0) : 0.0);

    json += "\"processCount\":" + std::to_string(processCount) + ",";
    json += "\"threadCount\":" + std::to_string(threadCount) + ",";
    json += "\"aggregateCpuPercent\":" + std::string(aggBuf);
    json += "}";

    g_prevGatherNs = gatherNs;
    return json;
}

bool sendAll(int fd, const char* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        const ssize_t n = send(fd, data + sent, len - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

void handleClient(int clientFd, uint64_t serverStartNs) {
    char reqBuf[2048];
    // Only need to drain the request enough that the client's own write
    // doesn't block on us; the response is the same JSON body regardless
    // of path, so nothing here depends on parsing it.
    recv(clientFd, reqBuf, sizeof(reqBuf) - 1, 0);

    const std::string body = gatherMetrics(serverStartNs);
    std::string response;
    response += "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: application/json\r\n";
    response += "Access-Control-Allow-Origin: *\r\n";
    response += "Cache-Control: no-store\r\n";
    response += "Connection: close\r\n";
    response += "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
    response += body;

    sendAll(clientFd, response.c_str(), response.size());
    close(clientFd);
}

}

int main() {
    const int listenFd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd == -1) {
        perror("socket failed");
        return EXIT_FAILURE;
    }

    const int yes = 1;
    setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(LISTEN_PORT);

    if (bind(listenFd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        perror("bind failed");
        return EXIT_FAILURE;
    }
    if (listen(listenFd, 8) == -1) {
        perror("listen failed");
        return EXIT_FAILURE;
    }

    printf("metrics_server: listening on 0.0.0.0:%d -- GET any path for live JSON metrics\n", LISTEN_PORT);

    const uint64_t serverStartNs = nowNs();
    for (;;) {
        struct sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        const int clientFd = accept(listenFd, reinterpret_cast<struct sockaddr*>(&clientAddr), &clientLen);
        if (clientFd == -1) {
            if (errno == EINTR) continue;
            perror("accept failed");
            continue;
        }
        handleClient(clientFd, serverStartNs);
    }
}

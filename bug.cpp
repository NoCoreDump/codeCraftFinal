#include <bits/stdc++.h>
#include <ext/pb_ds/hash_policy.hpp>
#include <ext/pb_ds/priority_queue.hpp>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

#include <sys/mman.h>

typedef uint64_t uint64;
typedef uint32_t uint32;
typedef uint16_t uint16;
#define MAXE  2500000
#define MAXN  1000000
#define MAX_NO_HASH_SIZE  4000
#define THREAD_COUNT 8
#define THREAD_COUNT_READ 8
#define PATCH_COUNT 4096

template<class T, class Alloc=std::allocator<T>>
class ReservedVector {
public:
    typedef T value_type;
    typedef value_type *iterator;
    typedef const value_type *const_iterator;
    typedef value_type &reference;
    typedef const T &const_reference;
    typedef size_t size_type;
    typedef ptrdiff_t difference_type; //distance
protected:
    std::allocator<value_type> _alloc;
    iterator _start;
    iterator _end;
    iterator _end_of_storage; //如果越过会触发扩容
    uint32 tail;
    value_type *data;
public:
    ReservedVector() {
        data = (T *) malloc(100000 * sizeof(T));
        _start = data;
        _end = _start;
        tail = 0;
    }

    ReservedVector &operator=(const ReservedVector &rhs) {
        if (this == &rhs)
            return *this;
        size_type n = rhs.cend() - rhs.cbegin();
        _start = _alloc.allocate(n);
        _end = _end_of_storage = std::uninitialized_copy(rhs.cbegin(), rhs.cend(), _start);
    }

    ~ReservedVector() {
        if (_start) {
            iterator it(_end);
            while (it != _start)
                _alloc.destroy(--it); //note end is invalid
        }
        _alloc.deallocate(_start, _end_of_storage - _start);
        _start = _end_of_storage = _end = NULL;
    }

    iterator begin() { return _start; }

    iterator end() { return _end; }

    const_iterator cbegin() const { return _start; }

    const_iterator cend() const { return _end; }

    size_type size() { return size_type(_end - _start); }  //cast to size_t
    size_type capacity() { return size_type(_end - _start); }

    inline bool empty() const { return _start == _end; }

    void swap(ReservedVector &other) {
        std::swap(_start, other._start);
        std::swap(_end, other._end);
        std::swap(_end_of_storage, other._end_of_storage);
    }

    reference front() { return *_start; }

    const_reference front() const { return *_start; }

    reference back() { return *(end() - 1); }

    reference operator[](size_type n) { return *(begin() + n); }

    const_reference top() {
        if (!empty())
            return data[tail];
        else return (T) NULL;
    }

    inline void push_back(const T &value) {
        data[tail++] = value;
        ++_end;
    }

    inline void pop_back() {
        --tail;
        --_end;
    }

    void insert(iterator pos, const value_type &elem) {
        if (pos > end()) {
            *pos = elem;
            _end = pos + 1;
            return;
        }
        iterator p = _end;
        while (p != _start && p != pos) {
            *p = *(p - 1);
        }
        *pos = elem;
        ++_end;
    }
};

struct Edge {
    uint32 to;
    uint32 w;
};
struct Node {
    uint32 l;
    uint32 r;
};

struct ThreadData {
    uint16 sigma[MAXN];
    uint16 dis_16[MAXN];
    uint32 dis_32[MAXN];
    uint64 dis_64[MAXN];
    double result[MAXN];
    double delta[MAXN];
    uint32 P[MAXN][250];
    uint32 S[MAXN];
    priority_queue<uint16, ReservedVector<uint16>, greater<uint16>> Q_16;
    priority_queue<uint32, ReservedVector<uint32>, greater<uint32>> Q_32;
    priority_queue<uint64, ReservedVector<uint64>, greater<uint64>> Q_64;
    vector<uint32> noHashIDList[MAX_NO_HASH_SIZE];
    unordered_map<uint16, vector<uint32>> disMap_16;
    unordered_map<uint32, vector<uint32>> disMap_32;
    unordered_map<uint64, vector<uint32>> disMap_64;

    inline vector<uint32> *getIDList_16(const uint16 &distance) {
        if (distance < MAX_NO_HASH_SIZE) {
            return &noHashIDList[distance];
        } else {
            return &disMap_16[distance];
        }
    }

    inline vector<uint32> *getIDList_32(const uint32 &distance) {
        if (distance < MAX_NO_HASH_SIZE) {
            return &noHashIDList[distance];
        } else {
            return &disMap_32[distance];
        }
    }

    inline vector<uint32> *getIDList_64(const uint64 &distance) {
        if (distance < MAX_NO_HASH_SIZE) {
            return &noHashIDList[distance];
        } else {
            return &disMap_64[distance];
        }
    }
};

Edge neighborsTable[MAXE];
Edge neighborsTable2[MAXE];
Node G[MAXN];
Node G2[MAXN];
double resultGlobal[MAXN];
uint32 resultId[MAXN];
ThreadData threadDatas[THREAD_COUNT];
uint32 overlap[MAXN];
uint32 patchCount;
uint32 patchSize;
atomic_int processingId(0);
uint32 nodeData[THREAD_COUNT_READ][2 * MAXE];
uint32 index_Id[THREAD_COUNT_READ][2 * MAXE];
uint32 moneyData[THREAD_COUNT_READ][MAXE];
uint32 nodeIdxMulti[THREAD_COUNT_READ][3];
uint16 inDegree[THREAD_COUNT_READ][MAXN];
uint16 outDegree[THREAD_COUNT_READ][MAXN];
uint16 inDegree2[MAXN];
uint16 outDegree2[MAXN];
uint32 moneyMax[THREAD_COUNT_READ];
uint32 index2Id[MAXN];
uint32 nodeCnt;
uint16 bitsFlag;

void addEdge(uint32 u, uint32 v, uint32 m) {
    neighborsTable[G[u].r].to = v;
    neighborsTable[G[u].r].w = m;
    ++G[u].r;
}

void idToIndex(uint32 threadId, uint32 *nodeData, unordered_map<uint32, uint32> id_Index) {
    //id_to_index
    for (uint32 idx = 0; idx < nodeIdxMulti[threadId][0]; idx += 2) {
        nodeData[idx] = id_Index[nodeData[idx]];
        nodeData[idx + 1] = id_Index[nodeData[idx + 1]];
        ++inDegree[threadId][nodeData[idx + 1]];
        ++outDegree[threadId][nodeData[idx]];
    }
}

void mergeData(uint32 *A, uint32 m, const uint32 *B, uint32 n) {
    int pa = m - 1, pb = n - 1;
    uint32 tail = m + n - 1;
    while (pa >= 0 && pb >= 0) {
        if (A[pa] > B[pb])
            A[tail--] = A[pa--];
        else
            A[tail--] = B[pb--];
    }
    while (pb >= 0) {
        A[tail--] = B[pb--];
    }
}

void sortThread(char *buf, const char *end, uint32 *nodeData, uint32 *moneyData,
                uint32 *index_Id, uint32 threadId) {
    uint32 nodeIdx = 0, nodeIdx2 = 0, nodeIdx3 = 0;
    while (buf < end) {
        uint32 from = 0, to = 0;
        uint32 money = 0;
        while ((*buf) & 0x10) {
            from *= 10;
            from += (*buf) & 0x0f;
            buf++;
        }
        ++buf;
        while ((*buf) & 0x10) {
            to *= 10;
            to += (*buf) & 0x0f;
            buf++;
        }
        ++buf;
        while ((*buf) & 0x10) {
            money *= 10;
            money += (*buf) & 0x0f;
            buf++;
        }
        if (*buf == '\r') {
            ++buf;
        }
        ++buf;
        if (money == 0) continue;
        nodeData[nodeIdx++] = from;
        nodeData[nodeIdx++] = to;
        moneyData[nodeIdx2++] = money;
        index_Id[nodeIdx3++] = from;
        index_Id[nodeIdx3++] = to;
        if (money > moneyMax[threadId]) {
            moneyMax[threadId] = money;
        }
    }
    sort(index_Id, index_Id + nodeIdx3);
    nodeIdxMulti[threadId][0] = nodeIdx;
    nodeIdxMulti[threadId][1] = nodeIdx2;
    nodeIdxMulti[threadId][2] = nodeIdx3;
}

void readData(const char *inputFilePath) {
    unordered_map<uint32, uint32> id_Index;
    int fd = open(inputFilePath, O_RDONLY);
    int fileLen = lseek(fd, 0, SEEK_END);
    char *buf = (char *) mmap(NULL, fileLen, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    uint32 sz = fileLen / THREAD_COUNT_READ;
    char *start[] = {buf, buf + sz, buf + sz * 2, buf + sz * 3, buf + sz * 4, buf + sz * 5, buf + sz * 6, buf + sz * 7,
                     buf + fileLen};
    for (unsigned short i = 1; i < THREAD_COUNT_READ; i++) {
        while (*(start[i]++) != '\n');
    }
    thread t[THREAD_COUNT_READ];
    for (unsigned short i = 0; i < THREAD_COUNT_READ; i++) {
        t[i] = thread(sortThread, start[i], start[i + 1], nodeData[i], moneyData[i], index_Id[i], i);
    }
    for (unsigned short i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    for (unsigned short i = 0; i < 4; i++) {
        t[i] = thread(mergeData, index_Id[i], nodeIdxMulti[i][2], index_Id[i + 4], nodeIdxMulti[i + 4][2]);
    }
    for (unsigned short i = 0; i < 4; i++)
        t[i].join();
    mergeData(index_Id[0], nodeIdxMulti[0][2] + nodeIdxMulti[4][2], index_Id[1],
              nodeIdxMulti[1][2] + nodeIdxMulti[5][2]);
    mergeData(index_Id[2], nodeIdxMulti[2][2] + nodeIdxMulti[6][2], index_Id[3],
              nodeIdxMulti[3][2] + nodeIdxMulti[7][2]);
    mergeData(index_Id[0], nodeIdxMulti[0][2] + nodeIdxMulti[1][2] + nodeIdxMulti[4][2] + nodeIdxMulti[5][2],
              index_Id[2], nodeIdxMulti[2][2] + nodeIdxMulti[3][2] + nodeIdxMulti[6][2] + nodeIdxMulti[7][2]);
    uint32 nodeIdx3 = 0;
    for (unsigned short i = 0; i < THREAD_COUNT_READ; i++)
        nodeIdx3 += nodeIdxMulti[i][2];
    nodeCnt = unique(index_Id[0], index_Id[0] + nodeIdx3) - index_Id[0];
    id_Index.rehash(1024 * 1024);
    id_Index.reserve(nodeCnt);
    for (uint32 idx = 0; idx < nodeCnt; idx++) {
        id_Index[index_Id[0][idx]] = idx;
    }
    for (unsigned short i = 0; i < THREAD_COUNT_READ; i++)
        t[i] = thread(idToIndex, i, nodeData[i], id_Index);
    for (unsigned short i = 0; i < THREAD_COUNT_READ; i++)
        t[i].join();
    for (uint32 idx = 0; idx < nodeCnt; idx++) {
        outDegree[0][idx] += outDegree[1][idx] + outDegree[2][idx] + outDegree[3][idx] +
                             outDegree[4][idx] + outDegree[5][idx] + outDegree[6][idx] + outDegree[7][idx];
        inDegree[0][idx] += inDegree[1][idx] + inDegree[2][idx] + inDegree[3][idx] +
                            inDegree[4][idx] + inDegree[5][idx] + inDegree[6][idx] + inDegree[7][idx];
    }
    G[0].l = 0;
    G[0].r = 0;
    for (uint32 idx = 1; idx < nodeCnt; idx++) {
        G[idx].l = G[idx - 1].l + outDegree[0][idx - 1];
        G[idx].r = G[idx].l;
    }
    for (unsigned short i = 0; i < THREAD_COUNT_READ; i++) {
        for (uint32 idx = 0, idx2 = 0; idx < nodeIdxMulti[i][0]; idx += 2) {
            addEdge(nodeData[i][idx], nodeData[i][idx + 1], moneyData[i][idx2++]);
        }
        moneyMax[0] = moneyMax[0] > moneyMax[i] ? moneyMax[0] : moneyMax[i];
    }
}

void dijkstraThread_32(uint32 patchId, uint32 threadId) {
    ThreadData &threadData = threadDatas[threadId];
    uint32 startIdx = patchId * patchSize;
    uint32 endIdx = (patchId + 1) * patchSize;
    endIdx = nodeCnt < endIdx ? nodeCnt : endIdx;
    if (endIdx <= startIdx) return;
    uint16 *sigma = threadData.sigma;
    uint32 *dis = threadData.dis_32;
    auto P = threadData.P;
    uint32 *S = threadData.S;
    double *delta = threadData.delta;
    auto &Q = threadData.Q_32;
    double *result = threadData.result;
    for (int start = startIdx; start < endIdx; ++start) {
        if (overlap[start] == 0) continue;
        uint32 sIdx = 0;
        dis[start] = 0;
        sigma[start] = 1;
        auto &pi = *threadData.getIDList_32(0);
        pi.push_back(start);
        Q.push(0);
        while (!Q.empty()) {
            uint32 curDis = Q.top();
            auto &idList = *threadData.getIDList_32(curDis);
            Q.pop();
            for (const uint32 &u:idList) {
                if (dis[u] != curDis) continue;
                S[sIdx++] = u;
                for (uint32 it = G2[u].l; it < G2[u].r; ++it) {
                    uint32 v = neighborsTable2[it].to;
                    uint32 disSum = dis[u] + neighborsTable2[it].w;
                    if (dis[v] > disSum) {
                        dis[v] = disSum;
                        sigma[v] = sigma[u];
                        P[v][0] = 0;
                        P[v][++P[v][0]] = u;
                        auto &piv = *threadData.getIDList_32(dis[v]);
                        if (piv.empty()) {
                            Q.push(dis[v]);
                        }
                        piv.push_back(v);
                    } else if (dis[v] == disSum) {
                        if (sigma[u]) {
                            sigma[v] += sigma[u];
                            P[v][++P[v][0]] = u;
                        }
                    }
                }
            }
            idList.resize(0);
        }
        sIdx--;
        for (uint32 si = sIdx; si > 0; si--) {
            uint32 v = S[si];
            const double dv = delta[v] + 1.0 / sigma[v];
            for (int idx = 1; idx <= P[v][0]; ++idx) {
                delta[P[v][idx]] += dv;
            }
            result[v] += delta[v] * sigma[v] * overlap[start];
            result[start] += overlap[start] - 1;
            sigma[v] = 0;
            delta[v] = 0;
            dis[v] = UINT32_MAX;
        }
        sigma[start] = 0;
        dis[start] = UINT32_MAX;
        delta[start] = 0;
    }
}

void dijkstraThread_64(uint32 patchId, uint32 threadId) {
    ThreadData &threadData = threadDatas[threadId];
    uint32 startIdx = patchId * patchSize;
    uint32 endIdx = (patchId + 1) * patchSize;
    endIdx = nodeCnt < endIdx ? nodeCnt : endIdx;
    if (endIdx <= startIdx) return;
    uint16 *sigma = threadData.sigma;
    uint64 *dis = threadData.dis_64;
    auto P = threadData.P;
    uint32 *S = threadData.S;
    double *delta = threadData.delta;
    auto &Q = threadData.Q_64;
    double *result = threadData.result;
    for (int start = startIdx; start < endIdx; ++start) {
        if (overlap[start] == 0) continue;
        uint32 sIdx = 0;
        dis[start] = 0;
        sigma[start] = 1;
        auto &pi = *threadData.getIDList_64(0);
        pi.push_back(start);
        Q.push(0);
        while (!Q.empty()) {
            uint64 curDis = Q.top();
            auto &idList = *threadData.getIDList_64(curDis);
            Q.pop();
            for (const uint32 &u:idList) {
                if (dis[u] != curDis) continue;
                S[sIdx++] = u;
                for (uint32 it = G2[u].l; it < G2[u].r; ++it) {
                    uint32 v = neighborsTable2[it].to;
                    uint64 disSum = dis[u] + neighborsTable2[it].w;
                    if (dis[v] > disSum) {
                        dis[v] = disSum;
                        sigma[v] = sigma[u];
                        P[v][0] = 0;
                        P[v][++P[v][0]] = u;
                        auto &piv = *threadData.getIDList_64(dis[v]);
                        if (piv.empty()) {
                            Q.push(dis[v]);
                        }
                        piv.push_back(v);
                    } else if (dis[v] == disSum) {
                        if (sigma[u]) {
                            sigma[v] += sigma[u];
                            P[v][++P[v][0]] = u;
                        }
                    }
                }
            }
            idList.resize(0);
        }
        sIdx--;
        for (uint32 si = sIdx; si > 0; si--) {
            uint32 v = S[si];
            const double dv = delta[v] + 1.0 / sigma[v];
            for (int idx = 1; idx <= P[v][0]; ++idx) {
                delta[P[v][idx]] += dv;
            }
            result[v] += delta[v] * sigma[v] * overlap[start];
            result[start] += overlap[start] - 1;
            sigma[v] = 0;
            delta[v] = 0;
            dis[v] = UINT64_MAX;
        }
        sigma[start] = 0;
        dis[start] = UINT64_MAX;
        delta[start] = 0;
    }
}

void searchResultThread(int threadId) {
    switch (bitsFlag) {
        case 32:
            fill(threadDatas[threadId].dis_32, threadDatas[threadId].dis_32 + nodeCnt, UINT32_MAX);
            while (processingId < patchCount) {
                dijkstraThread_32(atomic_fetch_add_explicit(&processingId, 1, std::memory_order_relaxed), threadId);
            }
            break;
        case 64:
            fill(threadDatas[threadId].dis_64, threadDatas[threadId].dis_64 + nodeCnt, UINT64_MAX);
            while (processingId < patchCount) {
                dijkstraThread_64(atomic_fetch_add_explicit(&processingId, 1, std::memory_order_relaxed), threadId);
            }
            break;
    }
}

void searchResult() {
    patchCount = PATCH_COUNT < nodeCnt ? PATCH_COUNT : nodeCnt;
    patchSize = (nodeCnt + patchCount - 1) / patchCount;
    long b = (long) moneyMax[0] * 60;
    if (b < UINT32_MAX) {
        bitsFlag = 32;
    } else
        bitsFlag = 64;
    std::thread threads[THREAD_COUNT];
    for (int i = 0; i < THREAD_COUNT; ++i) {
        threads[i] = thread(searchResultThread, i);
    }
    for (auto &thread : threads) {
        thread.join();
    }
}

void writeResult(const string &outputFile) {
    for (uint32 i = 0; i < THREAD_COUNT; ++i) {
        auto &resultThread = threadDatas[i].result;
        for (int idx = 0; idx < nodeCnt; ++idx)
            resultGlobal[idx] += resultThread[idx];
    }
    for (uint32 i = 0; i < nodeCnt; i++) {
        resultId[i] = i;
    }
    auto cmp = [&](uint32 l, uint32 r) {
        if (fabs(resultGlobal[l] - resultGlobal[r]) < 1e-6)
            return l < r;
        return resultGlobal[l] > resultGlobal[r];
    };
    sort(resultId, resultId + nodeCnt, cmp);
    FILE *fp = fopen(outputFile.c_str(), "wb");
    char buf[64];
    for (uint32 i = 0; i < 100; i++) {
        uint32 idx = sprintf(buf, "%d,%.3lf\n", index2Id[resultId[i]], resultGlobal[resultId[i]]);
        fwrite(buf, idx, sizeof(char), fp);
    }
    fclose(fp);
}

uint32 bfsNode[MAXN];
bool bfsVis[MAXN];

void bfs(uint32 start) {
    if (bfsVis[start]) return;
    bfsVis[start] = true;
    queue<uint32> que;
    que.push(start);
    while (!que.empty()) {
        uint32 now = que.front();
        que.pop();
        bfsNode[++bfsNode[0]] = now;
        for (int idx = G[now].l; idx < G[now].r; idx++) {
            uint32 next = neighborsTable[idx].to;
            if (!bfsVis[next]) {
                bfsVis[next] = true;
                que.push(next);
            }
        }
    }
}

void preProcessing() {
    for (int i = 0; i < nodeCnt; i++)
        bfs(i);
    //映射
    unordered_map<uint32, uint32> idMap;
    for (int idx = 0; idx < bfsNode[0]; idx++) {
        uint32 node = bfsNode[idx+1];
        index2Id[idx] = index_Id[0][node];//映射
        idMap[node] = idx;
    }
    //重新构图
    uint tableInv = 0;
    for (int idx = 0; idx < bfsNode[0]; idx++) {
        uint32 node = bfsNode[idx + 1];
        G2[idx].l = tableInv;
        for (int idx2 = G[node].l; idx2 < G[node].r; idx2++) {
            neighborsTable2[tableInv].to = idMap[neighborsTable[idx2].to];
            neighborsTable2[tableInv].w = neighborsTable[idx2].w;
            tableInv++;
        }
        G2[idx].r = tableInv;
        inDegree2[idx] = inDegree[0][node];
        outDegree2[idx] = outDegree[0][node];
    }
    for (int i = 0; i < nodeCnt; i++) {
        if (inDegree2[i] == 0 && outDegree2[i] == 1) {
            overlap[neighborsTable2[G2[i].l].to]++;
        } else if (outDegree2[i] != 0)
            overlap[i] += 1;
    }
}

int main(int argc, char *argv[]) {
    nice(-20);
    string testFile = "/home/data13.txt";
    string outputFile = "/home/myResult.txt";
//    string testFile= "/data/test_data.txt";
//    string outputFile = "/projects/student/result.txt";
    readData(testFile.c_str());
    preProcessing();
    searchResult();
    writeResult(outputFile);
    _exit(0);
}
